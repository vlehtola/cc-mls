#include <gtest/gtest.h>
#include <parallel_q.h>
#include <m-detector/DynObjFilter.h>
#include <cluster_predict/voxel_cluster.h>
#include <limits>
#include "preprocess.h"
#include "pointcloud_fields.h"
#include <cstring>

namespace {
sensor_msgs::PointCloud2::Ptr makeTimedCloud(bool hap,bool big_endian=false) {
    sensor_msgs::PointCloud2::Ptr input(new sensor_msgs::PointCloud2);
    input->width=1; input->height=2; input->point_step=26; input->row_step=32;
    input->is_bigendian=big_endian; input->data.resize(64);
    auto add=[&](const char* name,uint32_t offset,uint8_t type) {
        sensor_msgs::PointField field; field.name=name; field.offset=offset;
        field.datatype=type; field.count=1; input->fields.push_back(field);
    };
    using F=sensor_msgs::PointField;
    add("x",0,F::FLOAT32); add("y",4,F::FLOAT32); add("z",8,F::FLOAT32);
    add("intensity",12,F::FLOAT32);
    if(hap) { add("tag",16,F::UINT8); add("line",17,F::UINT8); add("timestamp",18,F::FLOAT64); }
    else { add("ring",16,F::UINT16); add("t",18,F::UINT32); }
    auto put=[&](size_t offset,auto value) {
        uint8_t bytes[sizeof(value)]; std::memcpy(bytes,&value,sizeof(value));
        uint16_t one=1; bool host_big=*reinterpret_cast<uint8_t*>(&one)==0;
        if(host_big!=big_endian) std::reverse(bytes,bytes+sizeof(value));
        std::memcpy(input->data.data()+offset,bytes,sizeof(value));
    };
    for(size_t row=0;row<2;++row) {
        const size_t start=row*32;
        put(start,float(5+row)); put(start+12,10.0f);
        const uint32_t ns=row==0 ? 100000000 : 10000000;
        if(hap) { put(start+17,uint8_t(1)); put(start+18,double(ns)); }
        else { put(start+16,uint16_t(127)); put(start+18,ns); }
    }
    return input;
}
}
TEST(TimedCloud, OusterUint16RingWithRowPaddingAndUnsortedTime) {
    Preprocess preprocess; preprocess.lidar_type=OUST64; preprocess.N_SCANS=128;
    auto input=makeTimedCloud(false);
    PointCloudXYZI::Ptr output(new PointCloudXYZI);
    preprocess.process(input,output);
    ASSERT_EQ(output->size(),2u);
    EXPECT_FLOAT_EQ(output->front().curvature,10.0f);
    EXPECT_FLOAT_EQ(output->back().curvature,100.0f);
    EXPECT_FLOAT_EQ(output->front().x,6.0f);
}
TEST(TimedCloud, HapDriver2RelativeNanosecondsAndBigEndian) {
    Preprocess preprocess; preprocess.lidar_type=LIVOX_HAP;
    auto input=makeTimedCloud(true,true);
    PointCloudXYZI::Ptr output(new PointCloudXYZI);
    preprocess.process(input,output);
    ASSERT_EQ(output->size(),2u);
    EXPECT_FLOAT_EQ(output->front().curvature,10.0f);
    EXPECT_FLOAT_EQ(output->back().curvature,100.0f);
}
TEST(TimedCloud, MalformedStrideAndCapacityAreRejected) {
    Preprocess preprocess; preprocess.lidar_type=LIVOX_HAP;
    auto input=makeTimedCloud(true);
    PointCloudXYZI::Ptr output(new PointCloudXYZI);
    input->row_step=1;
    preprocess.process(input,output);
    EXPECT_TRUE(output->empty());
    input=makeTimedCloud(true); preprocess.max_input_points=1;
    preprocess.process(input,output);
    EXPECT_TRUE(output->empty());
}
TEST(TimedCloud, FieldOutsidePointIsRejected) {
    auto input=makeTimedCloud(true);
    input->fields.back().offset=25;
    cc_mls::CloudFields fields(*input);
    EXPECT_THROW(fields.field("timestamp"),std::invalid_argument);
}
TEST(DynamicHistory, EnforcePointMemoryBudgetBeforeAllocation) {
    MovingObjectRemoval filter;
    filter.max_history_point_bytes=1;
    pcl::PointCloud<PointType>::Ptr scan(new pcl::PointCloud<PointType>);
    PointType point{}; point.x=5; scan->push_back(point);
    M3D rotation=M3D::Identity(); V3D position=V3D::Zero();
    EXPECT_THROW(filter.filter(scan,rotation,position,1.0),std::length_error);
    EXPECT_TRUE(filter.owned_frames.empty());
    EXPECT_EQ(filter.retained_point_bytes,0u);
}

TEST(PreprocessInput, RejectMalformedLivoxCount) {
    Preprocess preprocess;
    livox_ros_driver::CustomMsg::Ptr input(new livox_ros_driver::CustomMsg);
    input->point_num=3;
    PointCloudXYZI::Ptr output(new PointCloudXYZI);
    preprocess.process(input,output);
    EXPECT_TRUE(output->empty());
}
TEST(PreprocessInput, LivoxBlindDistanceAndNonfinitePoints) {
    Preprocess preprocess;
    preprocess.blind=1.0;
    livox_ros_driver::CustomMsg::Ptr input(new livox_ros_driver::CustomMsg);
    input->point_num=4; input->points.resize(4);
    input->points[1].x=0.5f;
    input->points[2].x=2.0f;
    input->points[3].x=std::numeric_limits<float>::quiet_NaN();
    PointCloudXYZI::Ptr output(new PointCloudXYZI);
    preprocess.process(input,output);
    ASSERT_EQ(output->size(),1u);
    EXPECT_FLOAT_EQ(output->front().x,2.0f);
}
TEST(PreprocessInput, MissingOusterTimingCannotProduceScan) {
    Preprocess preprocess;
    preprocess.lidar_type=OUST64;
    sensor_msgs::PointCloud2::Ptr input(new sensor_msgs::PointCloud2);
    PointCloudXYZI::Ptr output(new PointCloudXYZI);
    output->resize(1);
    preprocess.process(input,output);
    EXPECT_TRUE(output->empty());
}

TEST(DelayedQueue, ExactCapacityDoesNotPopAndOverflowIsExplicit) {
    PARALLEL_Q<int> queue(3);
    int a=11,b=22,c=33;
    queue.push_parallel_prepare(3);
    queue.push_parallel(a,0); queue.push_parallel(b,1); queue.push_parallel(c,2);
    ASSERT_EQ(queue.size(),3);
    EXPECT_EQ(queue.front(),11);
    EXPECT_THROW(queue.push_parallel_prepare(1),std::length_error);
    EXPECT_EQ(queue.size(),3);
    queue.pop(); EXPECT_EQ(queue.front(),22);
    queue.pop(); EXPECT_EQ(queue.front(),33);
    queue.pop(); EXPECT_TRUE(queue.empty());
}
TEST(Projection, CircularHorizontalAndClampedVerticalNeighbors) {
    EXPECT_EQ(md_neighbor_position(0,1,-1,0),(md_grid_width-1)*md_grid_height+1);
    EXPECT_EQ(md_neighbor_position(1,0,0,-1),-1);
    EXPECT_EQ(md_neighbor_position(1,md_grid_height-1,0,1),-1);
}
TEST(Projection, InvalidInputIsRejectedBeforeIntegerConversion) {
    MotionPoint point;
    V3D input(std::numeric_limits<double>::quiet_NaN(),0,0);
    float horizontal=.005f,vertical=.007f;
    point.GetVec(input,horizontal,vertical);
    EXPECT_EQ(point.position,-1);
}
TEST(Voxels, AxisBoundsAndSparseConnectedComponents) {
    VOXEL_CLUSTER cluster;
    Eigen::Vector3f origin=Eigen::Vector3f::Zero();
    cluster.setVoxelResolution(1,10,10,origin);
    cluster.setExtendRange(1); cluster.setMinClusterSize(2);
    PointType outside; outside.x=10; outside.y=outside.z=0;
    EXPECT_EQ(cluster.voxelIndex(outside),-1);
    outside.x=.1;outside.y=-.1;
    EXPECT_EQ(cluster.voxelIndex(outside),-1);
    pcl::PointCloud<PointType> points;
    for(int x=0;x<3;++x) { PointType p;p.x=x+.1f;p.y=.1f;p.z=.1f;points.push_back(p); }
    cluster.setInputCloud(points);
    SparseVoxelMap map;std::unordered_set<int> used;
    cluster.createVoxelMap(map,used);
    EXPECT_EQ(map.size(),3u);
    std::vector<std::vector<int>> components;cluster.extract(components);
    ASSERT_EQ(components.size(),1u);EXPECT_EQ(components[0].size(),3u);
}
TEST(detectMotion, MajorityVotesAndSpatialConsensus) {
    MovingObjectRemoval filter;
    filter.hor_resolution_max=.005f;filter.ver_resolution_max=.007f;
    MotionPoint point;point.glob=V3D(5,0,0);point.time=1;
    point.GetVec(point.glob,filter.hor_resolution_max,filter.ver_resolution_max);
    for(int i=0;i<3;++i) {
        DepthStructure::Ptr map(new DepthStructure(M3D::Identity(),V3D::Zero(),i*.1,i));
        map->depth_tensor.static_min_depth[point.position]=10;
        map->depth_tensor.static_max_depth[point.position]=10;
        map->depth_tensor.all_min_depth[point.position]=10;
        map->depth_tensor.all_max_depth[point.position]=10;
        filter.depth_structure_window.push_back(map);
    }
    EXPECT_EQ(filter.detectMotion(point),ENTERING);
    point.reset();point.glob=V3D(15,0,0);
    EXPECT_EQ(filter.detectMotion(point),LEAVING);
    point.reset();point.glob=V3D(10.05,0,0);
    EXPECT_EQ(filter.detectMotion(point),STATIC);
    filter.depth_structure_window.pop_back();point.reset();point.glob=V3D(5,0,0);
    EXPECT_EQ(filter.detectMotion(point),STATIC); // 2 votes do not meet the configured 5-frame majority.
}
