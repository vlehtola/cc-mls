// Fusion backend: static LiDAR keyframes, Scan Context, GICP and iSAM2.
// Scan Context implementation is preserved from the supplied SC-PGO library.
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>
#include <pcl/io/pcd_io.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <Eigen/Geometry>
#include <boost/bind/bind.hpp>
#include <filesystem>
#include <fstream>
#include <set>
#include <cmath>
#include "scancontext/Scancontext.h"

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
struct Keyframe { ros::Time time; gtsam::Pose3 T_k; Cloud::Ptr static_scan; };

class Backend {
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::PointCloud2> clouds;
    message_filters::Subscriber<nav_msgs::Odometry> odometry;
    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, nav_msgs::Odometry> sync;
    ros::Publisher path_pub, map_pub, odom_pub;
    ros::Timer map_timer;
    gtsam::ISAM2 isam;
    gtsam::Values optimized_keyframe_poses;
    SCManager static_scan_context;
    std::vector<Keyframe> keyframes;
    // Paper Sec. 3.5, Eq. (16): accepted revisit and co-visibility pair sets.
    std::set<std::pair<size_t,size_t>> E_rev;
    std::set<std::pair<size_t,size_t>> E_covis;
    double keyframe_distance, keyframe_angle, voxel_size, overlap_correspondence_distance;
    double covisibility_overlap_threshold, revisit_gicp_fitness_threshold, revisit_overlap_threshold, map_period;
    int max_covisibility_constraints, map_max_points;
    int max_keyframes=5000, max_stored_scan_points=20000000;
    size_t stored_scan_points=0;
    bool covisibility_constraints_enabled;
    std::string save_directory;
    std::ofstream edge_log;
    ros::Time last_time;

    static Eigen::Matrix4f matrix(const gtsam::Pose3& pose) { return pose.matrix().cast<float>(); }
    static gtsam::Pose3 fromMatrix(const Eigen::Matrix4f& m) {
        Eigen::Matrix4d d=m.cast<double>();
        return gtsam::Pose3(gtsam::Rot3(d.block<3,3>(0,0)), gtsam::Point3(d(0,3),d(1,3),d(2,3)));
    }
    static gtsam::SharedNoiseModel noise(double angle, double translation) {
        gtsam::Vector6 sigmas;
        sigmas << angle,angle,angle,translation,translation,translation;
        return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
    }
    Cloud::Ptr downsample(const Cloud::ConstPtr& input) const {
        Cloud::Ptr output(new Cloud);
        pcl::VoxelGrid<Point> filter; filter.setLeafSize(voxel_size,voxel_size,voxel_size);
        filter.setInputCloud(input); filter.filter(*output);
        return output;
    }
    double directedOverlap(const Cloud::ConstPtr& source, const Cloud::ConstPtr& target,
                           const Eigen::Matrix4f& T_source_target) const {
        if (source->empty() || target->empty()) return 0.0;
        Cloud moved; pcl::transformPointCloud(*source, moved, T_source_target);
        pcl::KdTreeFLANN<Point> tree; tree.setInputCloud(target);
        size_t hits=0; std::vector<int> ids(1); std::vector<float> dist(1);
        for(const auto& p:moved)
            if(tree.nearestKSearch(p,1,ids,dist)>0 && dist[0] <= overlap_correspondence_distance*overlap_correspondence_distance) ++hits;
        return double(hits)/moved.size();
    }
    double overlap(const Cloud::ConstPtr& source, const Cloud::ConstPtr& target,
                   const Eigen::Matrix4f& T_source_target) const {
        return std::min(directedOverlap(source,target,T_source_target),
                        directedOverlap(target,source,T_source_target.inverse()));
    }
    void recordEdge(size_t a,size_t b,const char* type,double score) {
        if(edge_log) edge_log << a << ',' << b << ',' << type << ',' << score << '\n';
    }
    void process(const sensor_msgs::PointCloud2ConstPtr& msg,
                 const nav_msgs::OdometryConstPtr& odom) {
        if(msg->header.frame_id!="lidar" || odom->child_frame_id!="lidar") {
            ROS_ERROR_THROTTLE(1.0,"Backend expects LiDAR-frame scans and world-to-LiDAR odometry"); return;
        }
        if(!last_time.isZero() && msg->header.stamp <= last_time) {
            ROS_ERROR("Timestamp rewind: restart backend before replaying a bag"); return;
        }
        last_time=msg->header.stamp;
        const auto& o=odom->pose.pose;
        Eigen::Quaterniond q(o.orientation.w,o.orientation.x,o.orientation.y,o.orientation.z);
        if(!q.coeffs().allFinite() || q.norm()<1e-6 || !std::isfinite(o.position.x) ||
           !std::isfinite(o.position.y) || !std::isfinite(o.position.z)) return;
        q.normalize();
        gtsam::Pose3 pose(gtsam::Rot3(q.toRotationMatrix()),gtsam::Point3(o.position.x,o.position.y,o.position.z));
        if(!keyframes.empty()) {
            auto delta=keyframes.back().T_k.between(pose);
            if(delta.translation().norm()<keyframe_distance &&
               gtsam::Rot3::Logmap(delta.rotation()).norm()<keyframe_angle) return;
        }
        Cloud::Ptr scan(new Cloud); pcl::fromROSMsg(*msg,*scan);
        Cloud::Ptr finite(new Cloud);
        for(const auto& p:*scan) if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)) finite->push_back(p);
        scan=downsample(finite);
        if(scan->size()<50) { ROS_WARN("Too few static keyframe points"); return; }
        // Explicit session limits prevent unlimited graph/scan allocation.
        // End the run rather than silently freezing or discarding loop history.
        if(keyframes.size()>=static_cast<size_t>(max_keyframes) ||
           stored_scan_points+scan->size()>static_cast<size_t>(max_stored_scan_points)) {
            saveTrajectory();
            ROS_FATAL("Backend session memory limit reached; saved configured results. Start a new run or increase validated limits.");
            ros::shutdown(); return;
        }
        stored_scan_points+=scan->size();
        size_t id=keyframes.size();
        keyframes.push_back({msg->header.stamp,pose,scan});
        static_scan_context.makeAndSaveScancontextAndKeys(*scan);
        gtsam::NonlinearFactorGraph pose_graph_factors;
        gtsam::Values initial_keyframe_poses;
        // Initialize the new node in the already corrected graph coordinate frame.
        gtsam::Pose3 guess = id ? optimized_keyframe_poses.at<gtsam::Pose3>(id-1).compose(keyframes[id-1].T_k.between(pose)) : pose;
        initial_keyframe_poses.insert(id,guess);
        if(!id) pose_graph_factors.add(gtsam::PriorFactor<gtsam::Pose3>(0,pose,noise(1e-4,1e-4)));
        else {
            pose_graph_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(id-1,id,keyframes[id-1].T_k.between(pose),noise(0.02,0.1)));
            recordEdge(id-1,id,"odometry",1.0);
            // Paper-style co-visibility measurements come from LIO. They are
            // correlated with sequential odometry; fixed conservative noise is
            // an approximation, exposed as an ablation switch.
            if(covisibility_constraints_enabled) {
                int added=0;
                for(int j=int(id)-2;j>=0 && j>=int(id)-10 && added<max_covisibility_constraints;--j) {
                    const size_t i = static_cast<size_t>(j);
                    // Eq. (14): T_ij = T_i^{-1} T_j. Here current id is paper j.
                    const auto T_ij=keyframes[i].T_k.between(pose);
                    // Eq. (15): noiseless z_ij; uncertainty is carried by the factor noise model.
                    const auto z_ij = T_ij;
                    double covisibility_overlap_ratio=overlap(scan,keyframes[i].static_scan,matrix(T_ij));
                    if(covisibility_overlap_ratio<covisibility_overlap_threshold) continue;
                    pose_graph_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(i,id,z_ij,noise(0.1,0.5)));
                    E_covis.insert({i,id});
                    recordEdge(i,id,"covisibility",covisibility_overlap_ratio); ++added;
                }
            }
            auto revisit_candidate=static_scan_context.detectLoopClosureID();
            if(revisit_candidate.first>=0 && size_t(revisit_candidate.first)<id) {
                const size_t p=revisit_candidate.first;
                const size_t q=id; // Paper revisit pair (p,q).
                if(!E_rev.count({p,id})) {
                    pcl::GeneralizedIterativeClosestPoint<Point,Point> revisit_gicp;
                    revisit_gicp.setInputSource(scan); revisit_gicp.setInputTarget(keyframes[p].static_scan);
                    revisit_gicp.setMaximumIterations(60);
                    revisit_gicp.setMaxCorrespondenceDistance(5.0);
                    revisit_gicp.setTransformationEpsilon(1e-6);
                    // Odom initializes translation; Scan Context supplies yaw.
                    Eigen::Matrix4f T_pq_initial=matrix(keyframes[p].T_k.between(pose));
                    Eigen::Matrix3f R=T_pq_initial.block<3,3>(0,0);
                    float roll=std::atan2(R(2,1),R(2,2));
                    float pitch=std::atan2(-R(2,0),std::hypot(R(2,1),R(2,2)));
                    // detectLoopClosureID reports the shift to align the recent
                    // descriptor to the historical descriptor (positive yaw).
                    T_pq_initial.block<3,3>(0,0)=(Eigen::AngleAxisf(revisit_candidate.second,Eigen::Vector3f::UnitZ()) *
                        Eigen::AngleAxisf(pitch,Eigen::Vector3f::UnitY()) *
                        Eigen::AngleAxisf(roll,Eigen::Vector3f::UnitX())).toRotationMatrix();
                    Cloud aligned; revisit_gicp.align(aligned,T_pq_initial);
                    // PCL aligns current q into historical p: this matrix is T_p^{-1} T_q,
                    // the relative pose used by BetweenFactor(p,q), not its inverse.
                    const auto T_pq=revisit_gicp.getFinalTransformation();
                    double revisit_registration_fitness=revisit_gicp.getFitnessScore(overlap_correspondence_distance);
                    double revisit_overlap_ratio=overlap(scan,keyframes[p].static_scan,T_pq);
                    if(revisit_gicp.hasConverged() && T_pq.allFinite() &&
                       std::isfinite(revisit_registration_fitness) && revisit_registration_fitness<=revisit_gicp_fitness_threshold && revisit_overlap_ratio>=revisit_overlap_threshold) {
                        // Eq. (13): z_pq from GICP; Eq. (16): Cauchy kernel on revisit factors.
                        const auto z_pq=fromMatrix(T_pq);
                        auto revisit_cauchy_noise_model=gtsam::noiseModel::Robust::Create(
                            gtsam::noiseModel::mEstimator::Cauchy::Create(1.0),noise(0.03,0.15));
                        pose_graph_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(p,q,z_pq,revisit_cauchy_noise_model));
                        E_rev.insert({p,q}); recordEdge(p,q,"revisit",revisit_overlap_ratio);
                        ROS_INFO("Static GICP revisit %zu -> %zu, overlap %.3f",p,q,revisit_overlap_ratio);
                    }
                }
            }
        }
        isam.update(pose_graph_factors,initial_keyframe_poses); isam.update(); optimized_keyframe_poses=isam.calculateEstimate();
        publishPath();
        saveTrajectory();
        if(!save_directory.empty()) pcl::io::savePCDFileBinary(save_directory+"/Scans/"+std::to_string(id)+".pcd",*scan);
    }
    void publishPath() {
        nav_msgs::Path path; path.header.frame_id="camera_init"; path.header.stamp=keyframes.back().time;
        for(size_t i=0;i<keyframes.size();++i) {
            const auto& p=optimized_keyframe_poses.at<gtsam::Pose3>(i);
            geometry_msgs::PoseStamped item; item.header=path.header; item.header.stamp=keyframes[i].time;
            item.pose.position.x=p.x(); item.pose.position.y=p.y(); item.pose.position.z=p.z();
            Eigen::Quaterniond q(p.rotation().matrix());
            item.pose.orientation.x=q.x(); item.pose.orientation.y=q.y(); item.pose.orientation.z=q.z(); item.pose.orientation.w=q.w();
            path.poses.push_back(item);
        }
        path_pub.publish(path);
        nav_msgs::Odometry odom; odom.header=path.header; odom.child_frame_id="lidar";
        odom.pose.pose=path.poses.back().pose; odom_pub.publish(odom);
    }
    void publishMap(const ros::TimerEvent&) {
        if(keyframes.empty() || !map_pub.getNumSubscribers()) return;
        Cloud::Ptr merged(new Cloud); bool truncated=false;
        for(size_t i=0;i<keyframes.size();++i) {
            if(merged->size()+keyframes[i].static_scan->size()>size_t(map_max_points)) { truncated=true; break; }
            Cloud moved; pcl::transformPointCloud(*keyframes[i].static_scan,moved,matrix(optimized_keyframe_poses.at<gtsam::Pose3>(i))); *merged+=moved;
        }
        if(truncated) ROS_WARN("Visualization map capped at %d points; saved scans and poses retain the full map",map_max_points);
        auto filtered=downsample(merged);
        sensor_msgs::PointCloud2 msg; pcl::toROSMsg(*filtered,msg);
        msg.header.frame_id="camera_init"; msg.header.stamp=keyframes.back().time; map_pub.publish(msg);
    }
    void saveTrajectory() const {
        if(save_directory.empty()) return;
        std::ofstream stream(save_directory+"/optimized_tum.txt"); stream.precision(17);
        for(size_t i=0;i<keyframes.size();++i) {
            const auto& p=optimized_keyframe_poses.at<gtsam::Pose3>(i); Eigen::Quaterniond q(p.rotation().matrix());
            stream << keyframes[i].time.toSec() << ' ' << p.x() << ' ' << p.y() << ' ' << p.z() << ' '
                   << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
        }
    }
public:
    Backend():nh("~"),clouds(nh,"/fusion/static_lidar",30),odometry(nh,"/fusion/lidar_odometry",30),sync(clouds,odometry,30) {
        nh.param("keyframe_distance",keyframe_distance,2.0);
        nh.param("keyframe_angle_deg",keyframe_angle,10.0); keyframe_angle*=3.141592653589793/180.0;
        nh.param("voxel_size",voxel_size,0.3);
        nh.param("overlap_distance",overlap_correspondence_distance,0.5);
        nh.param("covisibility_overlap",covisibility_overlap_threshold,0.6);
        nh.param("covisibility_enabled",covisibility_constraints_enabled,true);
        nh.param("covisibility_count",max_covisibility_constraints,2);
        nh.param("revisit_overlap",revisit_overlap_threshold,0.3);
        nh.param("gicp_max_fitness",revisit_gicp_fitness_threshold,0.2);
        nh.param("map_period",map_period,10.0);
        nh.param("map_max_points",map_max_points,5000000);
        nh.param("max_keyframes",max_keyframes,5000);
        nh.param("max_stored_scan_points",max_stored_scan_points,20000000);
        double revisit_descriptor_distance_threshold,scan_context_max_radius;
        nh.param("sc_distance_threshold",revisit_descriptor_distance_threshold,0.2);
        nh.param("sc_max_radius",scan_context_max_radius,80.0);
        const auto positive=[](double x){return std::isfinite(x) && x>0;};
        const auto ratio=[](double x){return std::isfinite(x) && x>=0 && x<=1;};
        if(!positive(voxel_size) || !positive(overlap_correspondence_distance) ||
           !positive(keyframe_distance) || !positive(keyframe_angle) || !positive(map_period) ||
           !positive(scan_context_max_radius) || !ratio(covisibility_overlap_threshold) ||
           !ratio(revisit_overlap_threshold) || !ratio(revisit_descriptor_distance_threshold) ||
           !std::isfinite(revisit_gicp_fitness_threshold) || revisit_gicp_fitness_threshold<0 ||
           map_max_points<1 || max_covisibility_constraints<0 || max_keyframes<1 || max_stored_scan_points<1)
            throw std::invalid_argument("Invalid backend parameters");
        static_scan_context.setSCdistThres(revisit_descriptor_distance_threshold); static_scan_context.setMaximumRadius(scan_context_max_radius);
        nh.param<std::string>("save_directory",save_directory,"");
        if(!save_directory.empty()) {
            // Always create a unique run directory; never delete existing scans.
            save_directory+="/run_"+std::to_string(ros::WallTime::now().toNSec());
            std::filesystem::create_directories(save_directory+"/Scans");
            edge_log.open(save_directory+"/edges.csv"); edge_log<<"from,to,type,overlap\n";
            ROS_INFO("Saving keyframes to %s",save_directory.c_str());
        }
        path_pub=nh.advertise<nav_msgs::Path>("/aft_pgo_path",1,true);
        map_pub=nh.advertise<sensor_msgs::PointCloud2>("/aft_pgo_map",1,true);
        odom_pub=nh.advertise<nav_msgs::Odometry>("/aft_pgo_odom",10);
        sync.registerCallback(boost::bind(&Backend::process,this,boost::placeholders::_1,boost::placeholders::_2));
        map_timer=nh.createTimer(ros::Duration(map_period),&Backend::publishMap,this);
    }
};

int main(int argc,char** argv) {
    ros::init(argc,argv,"cc_mls_backend");
    try { Backend backend; ros::spin(); }
    catch(const std::exception& ex) { ROS_FATAL("Backend stopped: %s",ex.what()); return 1; }
    return 0;
}
