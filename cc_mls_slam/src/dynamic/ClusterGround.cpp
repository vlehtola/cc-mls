#include <m-detector/DynObjCluster.h>
#include <Eigen/QR>
#include <Eigen/Geometry>
#include <algorithm>

bool MovingObjectCompletion::estimateGround(const pcl::PointCloud<PointType> &ground_pcl, const Eigen::Vector3f &world_z, Eigen::Vector3f &ground_norm, Eigen::Vector4f &ground_plane, pcl::PointCloud<PointType> &true_ground, std::unordered_set<int> &extend_pixels)
{
    if (ground_pcl.empty())
        return false;
    int batch_size = std::max(4, (int)ground_pcl.size() / 100);
    const float distance_threshold = 0.10f;
    const float max_ground_tilt_radians = 30.0f / 57.3f;
    pcl::PointCloud<PointType> split_pcl;
    int max_count = 0;
    Eigen::Vector3f max_normvec(0, 0, 0);
    pcl::PointCloud<PointType> max_points;
    for (int i = 0; i < ground_pcl.size(); i++)
    {
        split_pcl.push_back(ground_pcl[i]);
        if (split_pcl.size() == batch_size)
        {
            Eigen::Vector4f plane;
            if (fitLegacyPlane(plane, split_pcl) && plane[3] < distance_threshold)
            {
                Eigen::Vector3f normvec = plane.head(3).normalized();
                if (normvec.cross(world_z).norm() < sin(max_ground_tilt_radians))
                {
                    int count = 0;
                    pcl::PointCloud<PointType> tmp_points;
                    for (int j = 0; j < ground_pcl.size(); j++)
                    {
                        Eigen::Vector3f point;
                        point[0] = ground_pcl[j].x;
                        point[1] = ground_pcl[j].y;
                        point[2] = ground_pcl[j].z;
                        float dis = fabs(point.dot(plane.head(3)) + 1.0f) / plane.head(3).norm();
                        if (dis < distance_threshold)
                        {
                            tmp_points.push_back(ground_pcl[j]);
                            count++;
                        }
                    }
                    if (count > max_count)
                    {
                        max_count = count;
                        max_normvec = normvec;
                        ground_plane = plane;
                        max_points = tmp_points;
                        if (max_count > 0.6f * ground_pcl.size())
                            break;
                    }
                }
            }
            split_pcl.clear();
        }
    }
    if (ground_pcl.size() > 0 && (max_count > 0.2f * ground_pcl.size() || max_count > 500))
    {
        Eigen::Vector4f plane;
        if (fitLegacyPlane(plane, max_points) && plane[3] < distance_threshold)
        {
            Eigen::Vector3f normvec = plane.head(3).normalized();
            if (normvec.cross(world_z).norm() < sin(max_ground_tilt_radians))
            {
                max_normvec = normvec;
                ground_plane = plane;
            }
        }
        for (int j = 0; j < ground_pcl.size(); j++)
        {
            Eigen::Vector3f point;
            point[0] = ground_pcl[j].x;
            point[1] = ground_pcl[j].y;
            point[2] = ground_pcl[j].z;
            float dis = fabs(point.dot(ground_plane.head(3)) + 1.0f) / ground_plane.head(3).norm();
            if (dis < distance_threshold)
            {
                true_ground.push_back(ground_pcl[j]);
                int voxel = voxelIndex(ground_pcl[j]);
                extend_pixels.erase(voxel);
            }
        }
        if (max_normvec[2] < 0)
            max_normvec *= -1;
        ground_norm = max_normvec;
    }
    if (abs(ground_norm.norm() - 1.0f) < 0.1f)
        return true;
    else
        return false;
}

void MovingObjectCompletion::removeGround(const Eigen::Vector4f &ground_plane, pcl::PointCloud<PointType> &cluster_pcl, std::vector<int> &cluster_pcl_ind, std::vector<int> &dyn_tag, pcl::PointCloud<PointType> &true_ground)
{
    const float distance_threshold = 0.10f;
    pcl::PointCloud<PointType> non_ground_points;
    std::vector<int> new_cluster_pcl_ind;
    for (int i = 0; i < cluster_pcl.size(); i++)
    {
        Eigen::Vector3f point;
        point[0] = cluster_pcl[i].x;
        point[1] = cluster_pcl[i].y;
        point[2] = cluster_pcl[i].z;
        float dis = fabs(point.dot(ground_plane.head(3)) + 1.0f) / ground_plane.head(3).norm();
        if (dis > distance_threshold)
        {
            non_ground_points.push_back(cluster_pcl[i]);
            new_cluster_pcl_ind.push_back(cluster_pcl_ind[i]);
        }
        else
        {
            dyn_tag[cluster_pcl_ind[i]] = 0;
            true_ground.push_back(cluster_pcl[i]);
        }
    }
    cluster_pcl = non_ground_points;
    cluster_pcl_ind = new_cluster_pcl_ind;
}

// Legacy representation: xyz = plane coefficients n (n.dot(x)+1=0); w = mean fit error, NOT plane offset.
bool MovingObjectCompletion::fitLegacyPlane(Eigen::Vector4f &pca_result, const pcl::PointCloud<PointType> &point)
{
    const float threshold = 0.1;
    int point_size = point.size();
    Eigen::Matrix<float, Eigen::Dynamic, 3> A;
    Eigen::Matrix<float, Eigen::Dynamic, 1> b;
    A.resize(point_size, 3);
    b.resize(point_size, 1);
    b.setOnes();
    b *= -1.0f;
    for (int j = 0; j < point_size; j++)
    {
        A(j, 0) = point[j].x;
        A(j, 1) = point[j].y;
        A(j, 2) = point[j].z;
    }
    Eigen::Vector3f normvec = A.colPivHouseholderQr().solve(b);
    float norm = normvec.norm();
    float average_dis = 0.0;
    for (int j = 0; j < point_size; j++)
    {
        float tmp = fabs(normvec.dot(A.row(j)) + 1.0);
        average_dis += tmp;
        if (tmp > threshold)
        {
            return false;
        }
    }
    average_dis = std::max(0.01f, average_dis / point_size / norm);
    pca_result(0) = normvec(0);
    pca_result(1) = normvec(1);
    pca_result(2) = normvec(2);
    pca_result(3) = average_dis;
    return true;
}

