#include <m-detector/DynObjCluster.h>
#include <algorithm>

void MovingObjectCompletion::completeObject(const Eigen::Vector3f &ground_normal,
    ClusterCandidate &candidate, std::vector<int> &mask, int candidate_id)
{
    // One ordered sweep, not recursive flood fill. Preserve the legacy visitation order.
    // Called only after ground estimation succeeds.
    constexpr int dx[6] = {1, -1, 0, 0, 0, 0};
    constexpr int dy[6] = {0, 0, 1, -1, 0, 0};
    constexpr int dz[6] = {0, 0, 0, 0, 1, -1};
    for (int voxel : candidate.surrounding_voxel_order) {
        auto &surrounding = surrounding_voxels_[voxel];
        if (!candidate.growth_voxels.count(voxel) || surrounding.points_num <= 2) continue;

        for (int direction = 0; direction < 6; ++direction) {
            const int x = voxel / (grid_width_ * grid_height_) + dx[direction];
            const int y = (voxel / grid_height_) % grid_width_ + dy[direction];
            const int z = voxel % grid_height_ + dz[direction];
            if (x < 0 || y < 0 || z < 0 || x >= grid_width_ || y >= grid_width_ || z >= grid_height_) continue;
            const int neighbor = (x * grid_width_ + y) * grid_height_ + z;
            const auto &interior = interior_voxels_[neighbor];
            const auto &seed = seed_voxels_[neighbor];
            const bool touches_interior = interior.bbox_index == candidate_id && interior.points_num > 0;
            const bool touches_seed = seed.bbox_index == candidate_id && seed.points_num > 0 && !seed.cloud->empty();
            if (!touches_interior && !touches_seed) continue;

            Eigen::Vector4f plane;
            const Cloud &points = *surrounding.cloud;
            if (!fitLegacyPlane(plane, points) || !(std::abs(ground_normal.dot(plane.head(3).normalized())) < 0.8f)) continue;
            auto &completed = seed_voxels_[voxel];
            completed.bbox_index = candidate_id;
            completed.cloud = surrounding.cloud;
            // Retain legacy points_num semantics: completion does not increment seed occupancy.
            candidate.points += points;
            for (int point_index : *surrounding.cloud_index) {
                mask[point_index] = 1;
                candidate.point_indices.push_back(point_index);
            }
            break;
        }
    }
}

void MovingObjectCompletion::keepLargestComponent(pcl::PointCloud<PointType> &cluster_pcl, std::vector<int> &cluster_pcl_ind, std::vector<int> &dyn_tag)
{
    // Largest means most occupied voxels, not most points.
    if (cluster_pcl.size() < 2)
    {
        return;
    }
    pcl::PointCloud<PointType> new_cluster_pcl;
    std::vector<int> new_cluster_pcl_ind;
    VOXEL_CLUSTER cluster;
    std::unordered_map<int, Point_Cloud::Ptr> umap_cluster;
    std::vector<std::vector<int>> voxel_cluster;
    cluster.setInputCloud(cluster_pcl);
    cluster.setVoxelResolution(parameters.voxel_resolution, grid_width_, grid_height_, grid_origin_);
    cluster.setExtendRange(parameters.neighbor_radius_voxels);
    cluster.setMinClusterSize(parameters.min_cluster_voxels);
    cluster.createVoxelMap(umap_cluster);
    cluster.extract(voxel_cluster);
    int max_cluster_ind = 0;
    int max_voxel_num = 0;
    for (int i = 0; i < voxel_cluster.size(); i++)
    {
        if (voxel_cluster[i].size() > max_voxel_num)
        {
            max_cluster_ind = i;
            max_voxel_num = voxel_cluster[i].size();
        }
    }
    std::unordered_set<int> dyn_index;
    for (int i = 0; i < max_voxel_num; i++)
    {
        int voxel = voxel_cluster[max_cluster_ind][i];
        for (int j = 0; j < umap_cluster[voxel]->cloud->size(); j++)
        {
            new_cluster_pcl.push_back(umap_cluster[voxel]->cloud->points[j]);
            new_cluster_pcl_ind.push_back(cluster_pcl_ind[umap_cluster[voxel]->cloud_index->at(j)]);
            dyn_index.insert(cluster_pcl_ind[umap_cluster[voxel]->cloud_index->at(j)]);
        }
    }
    for (int i = 0; i < cluster_pcl_ind.size(); i++)
    {
        if (!dyn_index.count(cluster_pcl_ind[i]))
        {
            dyn_tag[cluster_pcl_ind[i]] = 0;
        }
    }
    std::unordered_map<int, Point_Cloud::Ptr>().swap(umap_cluster);
    cluster_pcl = new_cluster_pcl;
    cluster_pcl_ind = new_cluster_pcl_ind;
}

