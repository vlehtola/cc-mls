#include <m-detector/DynObjCluster.h>
#include <algorithm>
#include <stdexcept>

void MovingObjectCompletion::initialize() {
    if(!std::isfinite(parameters.voxel_resolution) || parameters.voxel_resolution<0.1f)
        throw std::invalid_argument("Cluster voxel resolution must be >= 0.1 m");
    if(parameters.neighbor_radius_voxels<0 || parameters.min_cluster_voxels<1 ||
       !std::isfinite(parameters.min_seed_fraction) || parameters.min_seed_fraction<0 || parameters.min_seed_fraction>1)
        throw std::invalid_argument("Invalid cluster neighborhood, size or seed fraction");
    grid_origin_ << -100.f,-100.f,-20.f;
    grid_width_=std::ceil(200.f/parameters.voxel_resolution);
    grid_height_=std::ceil(40.f/parameters.voxel_resolution);
    grid_size_=grid_width_*grid_width_*grid_height_;
    clearFrame();
    initialized_=true;
    if(!out_file.empty()) timing_log_.open(out_file,std::ios::out|std::ios::binary);
}

void MovingObjectCompletion::refineDynamicMask(std::vector<int>& dyn_tag,
    const Cloud& motion_seeds, const Cloud& scan_lidar,
    const std_msgs::Header& header, const Eigen::Matrix3d& R_world_lidar,
    const Eigen::Vector3d& t_world_lidar) {
    (void)header; (void)t_world_lidar; // Geometry is in the LiDAR frame; translation is not needed.
    if(!initialized_) throw std::logic_error("Initialize clustering before processing a scan");
    if(dyn_tag.size()!=scan_lidar.size()) throw std::invalid_argument("Cluster mask must match input scan size");
    clearFrame();
    const auto begin=ros::WallTime::now();
    try {
        auto voxel_clusters=clusterMotionSeeds(motion_seeds);
        stats.seed_seconds=(ros::WallTime::now()-begin).toSec();
        auto candidates=buildCandidates(voxel_clusters);
        assignCandidateRegions(candidates);
        assignScanPoints(dyn_tag,scan_lidar,candidates);
        const Eigen::Vector3f world_vertical_lidar=R_world_lidar.transpose().cast<float>()*Eigen::Vector3f::UnitZ();
        refineCandidates(dyn_tag,candidates,world_vertical_lidar);
        stats.total_seconds=(ros::WallTime::now()-begin).toSec();
        stats.candidate_count=candidates.size();
        ++stats.frame_count;
        // Preserve the legacy four-column order, using wall time for profiling.
        if(timing_log_) timing_log_ << stats.seed_seconds << ' ' << stats.ground_seconds << ' '
            << stats.completion_seconds << ' ' << stats.total_seconds-stats.completion_seconds << '\n';
        clearFrame();
    } catch(...) { clearFrame(); throw; }
}

void MovingObjectCompletion::clearFrame() {
    seed_voxels_.clear(); surrounding_voxels_.clear(); interior_voxels_.clear();
}

std::vector<std::vector<int>> MovingObjectCompletion::clusterMotionSeeds(const Cloud& seeds) {
    std::vector<std::vector<int>> clusters;
    // Preserve the legacy logging-enabled early-return condition.
    if(!out_file.empty() && seeds.size()<2) return clusters;
    VOXEL_CLUSTER cluster;
    cluster.setInputCloud(seeds);
    cluster.setVoxelResolution(parameters.voxel_resolution,grid_width_,grid_height_,grid_origin_);
    cluster.setExtendRange(parameters.neighbor_radius_voxels);
    cluster.setMinClusterSize(parameters.min_cluster_voxels);
    std::unordered_set<int> occupied;
    cluster.createVoxelMap(seed_voxels_,occupied);
    cluster.extract(clusters);
    return clusters;
}

std::vector<MovingObjectCompletion::ClusterCandidate> MovingObjectCompletion::buildCandidates(const std::vector<std::vector<int>>& clusters) {
    std::vector<ClusterCandidate> candidates;
    for(const auto& voxels:clusters) {
        if(voxels.empty()) continue;
        const int candidate_id=static_cast<int>(candidates.size());
        ClusterCandidate candidate;
        voxelOrigin(voxels.front(),candidate.bounds.min);
        candidate.bounds.max=candidate.bounds.min;
        for(int voxel:voxels) {
            auto& cell=seed_voxels_[voxel]; cell.bbox_index=candidate_id;
            candidate.seed_point_count+=cell.points_num;
            Eigen::Vector3f origin; voxelOrigin(voxel,origin);
            candidate.bounds.min=candidate.bounds.min.cwiseMin(origin);
            candidate.bounds.max=candidate.bounds.max.cwiseMax((origin.array()+parameters.voxel_resolution).matrix());
        }
        const Eigen::Vector3f size=candidate.bounds.size();
        const float minimum=parameters.voxel_resolution+0.001f;
        const bool spans_two_axes=(size.x()>minimum && size.y()>minimum) ||
            (size.x()>minimum && size.z()>minimum) || (size.y()>minimum && size.z()>minimum);
        if(parameters.min_cluster_voxels==1 || spans_two_axes) candidates.push_back(std::move(candidate));
        else for(int voxel:voxels) seed_voxels_[voxel].reset();
    }
    return candidates;
}

void MovingObjectCompletion::assignCandidateRegions(std::vector<ClusterCandidate>& candidates) {
    // Keep sequential writes: overlapping regions retain the legacy last-candidate ownership.
    for(size_t bbox_i=0;bbox_i<candidates.size();++bbox_i) {
        auto& candidate=candidates[bbox_i];
        const auto size=candidate.bounds.size();
        const auto midpoint=candidate.bounds.center();
        const float x_size=size.x(),y_size=size.y(),z_size=size.z();
        PointType center{}, min{}, max{};
        center.x=midpoint.x(); center.y=midpoint.y(); center.z=midpoint.z();
        min.x=candidate.bounds.min.x(); min.y=candidate.bounds.min.y(); min.z=candidate.bounds.min.z();
        max.x=candidate.bounds.max.x(); max.y=candidate.bounds.max.y(); max.z=candidate.bounds.max.z();
        int n_x = std::max(1.0f, 1.0f * x_size) / parameters.voxel_resolution;
        int n_y = std::max(1.0f, 1.0f * y_size) / parameters.voxel_resolution;
        int n_z = std::max(1.0f, 1.0f * z_size) / parameters.voxel_resolution;
        int voxel_center = floor((center.x - grid_origin_(0))/parameters.voxel_resolution) * grid_width_ * grid_height_ + floor((center.y - grid_origin_(1))/parameters.voxel_resolution) * grid_height_ + floor((center.z - grid_origin_(2))/parameters.voxel_resolution);
        int ii = 0;
        Eigen::Vector3f xyz;
        for (int i = 0; i <= 2 * n_x +1; i++)
        {   
            ii += (i%2 ? 1:-1) * i;
            int jj = 0;
            for (int j = 0; j <= 2 * n_y +1; j++)
            {   
                jj += (j%2 ? 1:-1) * j;
                int kk = 0;
                for (int k = 0; k <= 2 * n_z +1; k++)
                {    
                    kk += (k%2 ? 1:-1) * k;
                    int voxel = voxel_center + ii * grid_width_ * grid_height_ + jj * grid_height_ + kk;
                    if(voxel < 0 || voxel >= grid_size_) continue;
                    voxelOrigin(voxel_center, xyz);
                    int cx = (voxel_center / (grid_width_ * grid_height_)) + ii;
                    int cy = ((voxel_center / grid_height_) % grid_width_) + jj;
                    int cz = (voxel_center % grid_height_) + kk;
                    if(cx<0 || cy<0 || cz<0 || cx>=grid_width_ || cy>=grid_width_ || cz>=grid_height_) continue;
                    voxelOrigin(voxel, xyz);
                    Eigen::Vector3f voxel_loc(xyz(0) + 0.5f * parameters.voxel_resolution, xyz(1) + 0.5f * parameters.voxel_resolution, xyz(2) + 0.5f * parameters.voxel_resolution);
                    if (seed_voxels_[voxel].points_num == 0 && !((voxel_loc(0) > min.x && voxel_loc(0) < max.x) && (voxel_loc(1) > min.y && voxel_loc(1) < max.y) && (voxel_loc(2) > min.z && voxel_loc(2) < max.z)))
                    {
                        surrounding_voxels_[voxel].bbox_index = bbox_i;
                        
                        candidate.growth_voxels.emplace(voxel);
                        candidate.surrounding_voxel_order.push_back(voxel);
                    }
                    else if (seed_voxels_[voxel].points_num == 0 && (voxel_loc(0) > min.x && voxel_loc(0) < max.x) && (voxel_loc(1) > min.y && voxel_loc(1) < max.y) && (voxel_loc(2) > min.z && voxel_loc(2) < max.z))
                    {
                        interior_voxels_[voxel].bbox_index = bbox_i;
                        
                    }
                }
            }
        }
    }
}

void MovingObjectCompletion::assignScanPoints(std::vector<int>& dyn_tag,const Cloud& raw_point,std::vector<ClusterCandidate>& candidates) {
    for (int ite = 0; ite < raw_point.size(); ite++)
    {
        if (dyn_tag[ite] == -1)
            continue;
        int voxel = voxelIndex(raw_point[ite]);
        if (voxel < 0 || voxel >= grid_size_)
        {
            continue;
        }
        if (surrounding_voxels_[voxel].bbox_index > -1)
        {
            candidates[surrounding_voxels_[voxel].bbox_index].ground_samples.push_back(raw_point[ite]);
            if (surrounding_voxels_[voxel].points_num == 0)
            {
                surrounding_voxels_[voxel].cloud.reset(new pcl::PointCloud<PointType>());
                surrounding_voxels_[voxel].cloud->reserve(5);
                surrounding_voxels_[voxel].cloud_index.reset(new std::vector<int>());
                surrounding_voxels_[voxel].cloud_index->reserve(5);
                surrounding_voxels_[voxel].cloud->push_back(raw_point[ite]);
                surrounding_voxels_[voxel].cloud_index->push_back(ite);
            }
            else
            {
                surrounding_voxels_[voxel].cloud->push_back(raw_point[ite]);
                surrounding_voxels_[voxel].cloud_index->push_back(ite);
            }
            surrounding_voxels_[voxel].points_num++;
            dyn_tag[ite] = 0;
        }
        else if (seed_voxels_[voxel].points_num > 0 && seed_voxels_[voxel].bbox_index > -1)
        {
            auto tmp = raw_point[ite];
            tmp.curvature = ite;
            candidates[seed_voxels_[voxel].bbox_index].points.push_back(tmp);
            candidates[seed_voxels_[voxel].bbox_index].point_indices.push_back(ite);
            seed_voxels_[voxel].cloud->push_back(tmp);
            dyn_tag[ite] = 1;
        }
        else if (interior_voxels_[voxel].bbox_index > -1)
        {
            auto tmp = raw_point[ite];
            tmp.curvature = ite;
            candidates[interior_voxels_[voxel].bbox_index].points.push_back(tmp);
            candidates[interior_voxels_[voxel].bbox_index].point_indices.push_back(ite);
            if (interior_voxels_[voxel].points_num == 0)
            {
                interior_voxels_[voxel].cloud.reset(new pcl::PointCloud<PointType>());
                interior_voxels_[voxel].cloud->reserve(5);
                interior_voxels_[voxel].cloud->push_back(tmp);
            }
            else
            {
                interior_voxels_[voxel].cloud->push_back(tmp);
            }
            interior_voxels_[voxel].points_num++;
            dyn_tag[ite] = 1;
        }
        else
        {
            dyn_tag[ite] = 0;
        }
    }
}

void MovingObjectCompletion::refineCandidates(std::vector<int>& mask,std::vector<ClusterCandidate>& candidates,
                                    const Eigen::Vector3f& world_vertical_lidar) {
    stats.accepted_count=0;
    stats.ground_seconds=0;
    stats.completion_seconds=0;
    for(size_t index=0;index<candidates.size();++index) {
        auto& candidate=candidates[index];
        Eigen::Vector3f ground_normal=Eigen::Vector3f::Zero();
        Eigen::Vector4f ground_plane=Eigen::Vector4f::Zero();
        const auto ground_begin=ros::WallTime::now();
        const bool has_ground=estimateGround(candidate.ground_samples,world_vertical_lidar,
            ground_normal,ground_plane,candidate.ground_inliers,candidate.growth_voxels);
        stats.ground_seconds+=(ros::WallTime::now()-ground_begin).toSec();
        if(has_ground) {
            const auto completion_begin=ros::WallTime::now();
            completeObject(ground_normal,candidate,mask,static_cast<int>(index));
            stats.completion_seconds+=(ros::WallTime::now()-completion_begin).toSec();
            removeGround(ground_plane,candidate.points,candidate.point_indices,mask,candidate.ground_inliers);
        }
        keepLargestComponent(candidate.points,candidate.point_indices,mask);
        // Preserve original seed-count/final-point-count rule (not a probability or a recount of surviving seeds).
        if(candidate.points.empty()) continue; // No output; avoid division by zero.
        const float seed_fraction=float(candidate.seed_point_count)/float(candidate.points.size());
        if(seed_fraction<parameters.min_seed_fraction) {
            for(int point_index:candidate.point_indices) mask[point_index]=0;
        } else ++stats.accepted_count;
    }
}

int MovingObjectCompletion::voxelIndex(const PointType& point) const {
    if(!std::isfinite(point.x)||!std::isfinite(point.y)||!std::isfinite(point.z)) return -1;
    const double x=(double(point.x)-grid_origin_.x())/parameters.voxel_resolution;
    const double y=(double(point.y)-grid_origin_.y())/parameters.voxel_resolution;
    const double z=(double(point.z)-grid_origin_.z())/parameters.voxel_resolution;
    if(x<0||y<0||z<0||x>=grid_width_||y>=grid_width_||z>=grid_height_) return -1;
    return (int(std::floor(x))*grid_width_+int(std::floor(y)))*grid_height_+int(std::floor(z));
}

void MovingObjectCompletion::voxelOrigin(const int& index,Eigen::Vector3f& xyz) const {
    const int x=index/(grid_width_*grid_height_);
    const int y=(index/grid_height_)%grid_width_;
    const int z=index%grid_height_;
    xyz=grid_origin_+parameters.voxel_resolution*Eigen::Vector3f(x,y,z);
}
