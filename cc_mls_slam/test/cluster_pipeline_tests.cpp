#include <gtest/gtest.h>
#include <m-detector/DynObjCluster.h>
#include "reference/LegacyDynObjCluster.h"
#include <limits>
#include <stdexcept>
#include <Eigen/Geometry>

namespace {
using Cloud=pcl::PointCloud<PointType>;
PointType point(float x,float y,float z) {
    PointType p{}; p.x=x;p.y=y;p.z=z; return p;
}
void compareMasks(const Cloud& seeds,const Cloud& scan,const Eigen::Matrix3d& rotation,
                  float seed_fraction=0.7f) {
    LegacyDynObjCluster reference;
    reference.cluster_min_pixel_number=1;
    reference.thrustable_thresold=seed_fraction;
    reference.Init();
    MovingObjectCompletion pipeline;
    pipeline.parameters.min_cluster_voxels=1;
    pipeline.parameters.min_seed_fraction=seed_fraction;
    pipeline.initialize();
    std_msgs::Header header;
    header.stamp=ros::Time(1,0);header.frame_id="camera_init";
    std::vector<int> expected(scan.size(),0),actual(scan.size(),0);
    if(!scan.empty()) expected.back()=actual.back()=-1;
    reference.Clusterprocess(expected,seeds,scan,header,rotation,Eigen::Vector3d::Zero());
    pipeline.refineDynamicMask(actual,seeds,scan,header,rotation,Eigen::Vector3d::Zero());
    EXPECT_EQ(actual,expected);
    // Consecutive calls must not carry sparse-map ownership from the previous frame.
    std::vector<int> expected_again(scan.size(),0),actual_again(scan.size(),0);
    reference.Clusterprocess(expected_again,seeds,scan,header,rotation,Eigen::Vector3d::Zero());
    pipeline.refineDynamicMask(actual_again,seeds,scan,header,rotation,Eigen::Vector3d::Zero());
    EXPECT_EQ(actual_again,expected_again);
}
}

TEST(ClusterRefactor, EmptyAndOutOfBoundsScans) {
    ros::Time::init();
    Cloud empty;
    compareMasks(empty,empty,Eigen::Matrix3d::Identity());
    Cloud seeds,scan;
    seeds.push_back(point(200,0,0));scan=seeds;
    scan.push_back(point(std::numeric_limits<float>::quiet_NaN(),0,0));
    compareMasks(seeds,scan,Eigen::Matrix3d::Identity());
}
TEST(ClusterRefactor, GroundAndObjectCompletionMatchReference) {
    ros::Time::init();
    Cloud seeds,scan;
    // Ground away from the origin: compatible with the legacy n.dot(x)+1=0 fit.
    for(int x=0;x<16;++x) for(int y=0;y<16;++y)
        scan.push_back(point(4.f+x*0.12f,-1.f+y*0.12f,-1.2f));
    for(int x=0;x<4;++x) for(int y=0;y<4;++y) for(int z=0;z<6;++z) {
        auto p=point(4.6f+x*0.12f,-0.4f+y*0.12f,-1.f+z*0.16f);
        scan.push_back(p); if(z>1) seeds.push_back(p);
    }
    compareMasks(seeds,scan,Eigen::Matrix3d::Identity());
    compareMasks(seeds,scan,Eigen::AngleAxisd(0.2,Eigen::Vector3d::UnitX()).toRotationMatrix());
}
TEST(ClusterRefactor, NearbyCandidatesAndConfidenceRejection) {
    ros::Time::init();
    Cloud seeds,scan;
    for(int object=0;object<2;++object) for(int x=0;x<3;++x) for(int y=0;y<3;++y) {
        auto p=point(5.f+object*1.2f+x*0.11f,y*0.11f,0.5f);
        seeds.push_back(p); scan.push_back(p);
        scan.push_back(point(p.x+0.01f,p.y+0.01f,p.z+0.02f));
    }
    compareMasks(seeds,scan,Eigen::Matrix3d::Identity(),0.99f);
}
TEST(ClusterRefactor, RejectMaskLengthMismatch) {
    MovingObjectCompletion pipeline; pipeline.initialize();
    Cloud scan; scan.push_back(point(5,0,0));
    std::vector<int> mask;
    EXPECT_THROW(pipeline.refineDynamicMask(mask,scan,scan,std_msgs::Header(),
        Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero()),std::invalid_argument);
}
