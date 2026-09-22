#pragma once
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cluster_predict/voxel_cluster.h>
#include <cmath>
#include <fstream>
#include <vector>
#include <unordered_set>

// Converts a motion-seed mask into an object-complete dynamic mask.
// Both input clouds are in LiDAR coordinates. Mask values: -1 invalid, 0 static, 1 dynamic.
class MovingObjectCompletion {
public:
    using Cloud=pcl::PointCloud<PointType>;
    struct Parameters {
        float voxel_resolution=0.3f;
        int neighbor_radius_voxels=2;
        int min_cluster_voxels=4;
        float min_seed_fraction=0.3f;
    } parameters;
    struct Statistics {
        size_t frame_count=0,candidate_count=0,accepted_count=0;
        double total_seconds=0, seed_seconds=0, ground_seconds=0, completion_seconds=0;
    } stats;
    bool debug_en=false;
    std::string out_file;

    void initialize();
    void refineDynamicMask(std::vector<int>& mask,const Cloud& motion_seeds,const Cloud& scan_lidar,
        const std_msgs::Header& header,const Eigen::Matrix3d& R_world_lidar,const Eigen::Vector3d& t_world_lidar);

private:
    struct AxisAlignedBounds {
        Eigen::Vector3f min=Eigen::Vector3f::Zero(),max=Eigen::Vector3f::Zero();
        Eigen::Vector3f size() const { return max-min; }
        Eigen::Vector3f center() const { return (max+min)*0.5f; }
    };
    struct ClusterCandidate {
        AxisAlignedBounds bounds;
        Cloud points,ground_samples,ground_inliers;
        std::vector<int> point_indices;
        std::unordered_set<int> growth_voxels;
        std::vector<int> surrounding_voxel_order;
        int seed_point_count=0;
    };
    // These maps are frame-local working storage. Cleared on entry, exit and exceptions.
    SparseVoxelMap seed_voxels_,surrounding_voxels_,interior_voxels_;
    Eigen::Vector3f grid_origin_=Eigen::Vector3f::Zero();
    int grid_width_=0,grid_height_=0,grid_size_=0;
    std::ofstream timing_log_;
    bool initialized_=false;

    void clearFrame();
    std::vector<std::vector<int>> clusterMotionSeeds(const Cloud& seeds);
    std::vector<ClusterCandidate> buildCandidates(const std::vector<std::vector<int>>& clusters);
    void assignCandidateRegions(std::vector<ClusterCandidate>& candidates);
    void assignScanPoints(std::vector<int>& mask,const Cloud& scan,std::vector<ClusterCandidate>& candidates);
    void refineCandidates(std::vector<int>& mask,std::vector<ClusterCandidate>& candidates,const Eigen::Vector3f& world_vertical_lidar);
    bool estimateGround(const Cloud& samples,const Eigen::Vector3f& world_vertical_lidar,Eigen::Vector3f& normal,
        Eigen::Vector4f& plane,Cloud& inliers,std::unordered_set<int>& growth_voxels);
    void removeGround(const Eigen::Vector4f& plane,Cloud& points,std::vector<int>& indices,std::vector<int>& mask,Cloud& ground);
    void completeObject(const Eigen::Vector3f& normal,ClusterCandidate& candidate,std::vector<int>& mask,int candidate_id);
    void keepLargestComponent(Cloud& points,std::vector<int>& indices,std::vector<int>& mask);
    bool fitLegacyPlane(Eigen::Vector4f& coefficients_and_error,const Cloud& points);
    int voxelIndex(const PointType& point) const;
    void voxelOrigin(const int& index,Eigen::Vector3f& origin) const;
};
