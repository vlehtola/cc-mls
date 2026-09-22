#ifndef DYN_OBJ_FLT_H
#define DYN_OBJ_FLT_H

#include <omp.h>
#include <mutex>
#include <math.h>
#include <ros/ros.h>
// #include <so3_math.h>
#include <Eigen/Core>
#include <types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CompressedImage.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/LU>
#include <m-detector/DynObjCluster.h>
#include <parallel_q.h>
#include <algorithm>
#include <memory>
#include <stdexcept>
#include <limits>
#include <cmath>
#include <chrono>
#include <execution>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <tinycolormap.hpp>


using namespace std;
using namespace Eigen;
using namespace cv;

/*** For dynamic object filtering ***/
#define PI_MATH (3.141593f)
#define HASH_P 116101
#define MAX_N 100000
#define MAX_2D_N       (md_grid_width * md_grid_height) //1317755(1317755) //(50086) 39489912 // MAX_1D * MAX_1D_HALF
#define MAX_1D         (md_grid_width) //(2095) //(317)  // 2*pi/ hor_resolution
#define MAX_1D_HALF    (md_grid_height) //3142() //158 31416 pi / ver_resolution
#define DEPTH_WIDTH    (80 * 10)
#define COE_SMALL   1000
#define MAP_NUM     17 //30
#define HASH_PRIM   19 //37

inline int md_grid_width = 1257;
inline int md_grid_height = 629;

inline int md_neighbor_position(int u, int v, int du, int dv) {
    int row=v+dv;
    if(row<0 || row>=md_grid_height) return -1;
    int col=((u+du)%md_grid_width+md_grid_width)%md_grid_width;
    return col*md_grid_height+row;
}

// Keep existing non-motion numeric values; LEAVING is the added fourth motion label.
enum MotionLabel {STATIC=0, ENTERING=1, RECEDING=2, APPROACHING=3, SELF=4, UNCERTAIN=5, INVALID=6, LEAVING=7};
inline bool IsDynamicMotion(MotionLabel motion) {
    return motion==ENTERING || motion==LEAVING || motion==APPROACHING || motion==RECEDING;
}

struct MotionPoint
{
    int          hor_ind;
    V3F          vec;
    int          ver_ind;
    int          position;
    int          occu_times;
    int          is_occu_times;
    Vector3i     occu_index;
    Vector3i     is_occu_index;
    double       time;
    V3F          occ_vec;
    V3F          is_occ_vec;
    M3D          rot;
    V3D          transl;
    MotionLabel  dyn = STATIC;
    V3D          glob;
    V3D          local;
    V3F          cur_vec;
    float        intensity = 0.0f;
    bool         is_distort;
    V3D          last_closest;
    array<float, MAP_NUM> last_depth_interps = {};
    array<V3F, HASH_PRIM> last_vecs = {};
    array<Vector3i, HASH_PRIM> last_positions = {};   
    typedef boost::shared_ptr<MotionPoint> Ptr;
    MotionPoint(V3D & point, float & hor_resolution_max, float & ver_resolution_max)
    {
        vec(2)     = float(point.norm());
        vec(0)     = atan2f(float(point(1)), float(point(0)));
        vec(1)     = atan2f(float(point(2)), sqrt(pow(float(point(0)), 2) + pow(float(point(1)), 2)));
        hor_ind    = floor((vec(0) + PI_MATH) / hor_resolution_max);
        ver_ind    = floor((vec(1) + 0.5 * PI_MATH) / ver_resolution_max);
        position   = hor_ind * MAX_1D_HALF + ver_ind;
        time       = -1;
        occu_times = is_occu_times = 0;
        occu_index = -1*Vector3i::Ones();
        is_occu_index = -1*Vector3i::Ones();
        occ_vec.setZero();
        is_occ_vec.setZero();
        transl.setZero();
        glob.setZero();
        rot.setOnes();
        last_depth_interps.fill(0.0);
        last_vecs.fill(V3F::Zero());
        last_positions.fill(Vector3i::Zero());
        is_distort = false;
        cur_vec.setZero();
        local.setZero();
        last_closest.setZero();
    };
    MotionPoint()
    {
        vec.setZero();
        hor_ind  = ver_ind = position = occu_times = is_occu_times = 0;
        time       = -1;
        occu_index = -1*Vector3i::Ones();
        is_occu_index = -1*Vector3i::Ones();
        occ_vec.setZero();
        is_occ_vec.setZero();
        transl.setZero();
        glob.setZero();
        rot.setOnes();
        last_depth_interps.fill(0.0);
        last_vecs.fill(V3F::Zero());
        last_positions.fill(Vector3i::Zero());
        is_distort = false;
        cur_vec.setZero();
        local.setZero();
        last_closest.setZero();
    };
    MotionPoint(V3F s, int ind1, int ind2, int pos)
    {
        vec = s;
        hor_ind = ind1;
        ver_ind = ind2;
        position = pos;
        occu_times = is_occu_times = 0;
        time = -1;
        occu_index = -1*Vector3i::Ones();
        is_occu_index = -1*Vector3i::Ones();
        occ_vec.setZero();
        is_occ_vec.setZero();
        transl.setZero();
        glob.setZero();
        rot.setOnes();
        last_depth_interps.fill(0.0);
        last_vecs.fill(V3F::Zero());
        last_positions.fill(Vector3i::Zero());
        is_distort = false;
        cur_vec.setZero();
        local.setZero();
        last_closest.setZero();
    };
    MotionPoint(const MotionPoint & cur)
    {   
        vec = cur.vec;
        hor_ind  = cur.hor_ind;
        ver_ind  = cur.ver_ind;
        position  = cur.position;
        time = cur.time;
        occu_times = cur.occu_times;
        is_occu_times = cur.is_occu_times;
        occu_index = cur.occu_index;
        is_occu_index = cur.is_occu_index;
        occ_vec = cur.occ_vec;
        is_occ_vec = cur.is_occ_vec;
        transl = cur.transl;
        glob = cur.glob;
        rot = cur.rot;
        dyn = cur.dyn;
        intensity = cur.intensity;
        last_depth_interps = cur.last_depth_interps;
        last_vecs = cur.last_vecs;
        last_positions = cur.last_positions;
        local = cur.local;
        is_distort = cur.is_distort;
        cur_vec = cur.cur_vec;
        last_closest = cur.last_closest;
    };

    ~MotionPoint(){
    };

    void GetVec(V3D & point, float & hor_resolution_max, float & ver_resolution_max)
    {
        vec(2)    = float(point.norm());
        vec(0)    = atan2f(float(point(1)), float(point(0)));
        vec(1)    = atan2f(float(point(2)), sqrt(pow(float(point(0)), 2) + pow(float(point(1)), 2)));
        if (!std::isfinite(vec(2)) || !std::isfinite(vec(0)) || !std::isfinite(vec(1))) {
            hor_ind = ver_ind = position = -1;
            return;
        }
        hor_ind   = floor((vec(0) + PI_MATH) / hor_resolution_max);
        ver_ind   = floor((vec(1) + 0.5 * PI_MATH) / ver_resolution_max);
        hor_ind = std::max(0, std::min(hor_ind, MAX_1D - 1));
        position = (ver_ind < 0 || ver_ind >= MAX_1D_HALF) ? -1 : hor_ind * MAX_1D_HALF + ver_ind;
    };

    void reset()
    {
        occu_times = is_occu_times = 0;
        occu_index = -1*Vector3i::Ones();
        is_occu_index = -1*Vector3i::Ones();
        occ_vec.setZero();
        is_occ_vec.setZero();
        last_closest.setZero();
        last_depth_interps.fill(0.0);
        last_vecs.fill(V3F::Zero());
        last_positions.fill(Vector3i::Zero());
        is_distort = false;
    };
};

typedef std::vector<std::vector<MotionPoint*>>  PixelPointIndex;

// Paper Eq. (6): M_k, four depth channels in separate arrays (same storage layout).
// Memory remains owned and released by DepthStructure.
struct DepthTensor {
    float* static_min_depth = nullptr;
    float* static_max_depth = nullptr;
    float* all_min_depth = nullptr;
    float* all_max_depth = nullptr;
};

// Paper Eq. (3): D_k = (T_k, t_k, M_k); point indices are implementation auxiliaries.
class DepthStructure
{
public:
    DepthTensor depth_tensor;
    PixelPointIndex       pixel_point_index;
    double           time;
    int              map_index;
    M3D              R_world_lidar;
    V3D              t_world_lidar;
    std::vector<MotionPoint::Ptr>      point_storage;
    int              point_storage_count = 0;
    int*             all_max_point_index = nullptr;
    int*             all_min_point_index = nullptr;
    std::vector<int> index_vector;
    typedef boost::shared_ptr<DepthStructure> Ptr;

    DepthStructure()
    {   
        printf("build depth map2\n");
        pixel_point_index.assign(MAX_2D_N, std::vector<MotionPoint*>());
        
        time = 0.;
        R_world_lidar.setIdentity(3,3);
        t_world_lidar.setZero(3, 1);

        depth_tensor.static_min_depth = new float[MAX_2D_N];
        depth_tensor.all_min_depth = new float[MAX_2D_N];
        depth_tensor.all_max_depth = new float[MAX_2D_N];
        depth_tensor.static_max_depth = new float[MAX_2D_N];
        fill_n(depth_tensor.static_min_depth, MAX_2D_N, 0.0);
        fill_n(depth_tensor.all_min_depth, MAX_2D_N, 0.0);
        fill_n(depth_tensor.all_max_depth, MAX_2D_N, 0.0);
        fill_n(depth_tensor.static_max_depth, MAX_2D_N, 0.0);
        all_max_point_index = new int[MAX_2D_N];
        all_min_point_index = new int[MAX_2D_N];
        fill_n(all_min_point_index, MAX_2D_N, -1);
        fill_n(all_max_point_index, MAX_2D_N, -1);
        map_index = -1;
        index_vector.assign(MAX_2D_N, 0);
        for (int i = 0; i < MAX_2D_N; i++) {
            index_vector[i] = i;
        }
    }

    DepthStructure(M3D rot, V3D transl, double cur_time, int frame)
    {   
        pixel_point_index.assign(MAX_2D_N, std::vector<MotionPoint*>());       
        time = cur_time;
        R_world_lidar = rot;
        t_world_lidar = transl;
        depth_tensor.static_min_depth = new float[MAX_2D_N];
        depth_tensor.all_min_depth = new float[MAX_2D_N];
        depth_tensor.all_max_depth = new float[MAX_2D_N];
        depth_tensor.static_max_depth = new float[MAX_2D_N];
        fill_n(depth_tensor.static_min_depth, MAX_2D_N, 0.0);
        fill_n(depth_tensor.all_min_depth, MAX_2D_N, 0.0);
        fill_n(depth_tensor.all_max_depth, MAX_2D_N, 0.0);
        fill_n(depth_tensor.static_max_depth, MAX_2D_N, 0.0);
        all_max_point_index = new int[MAX_2D_N];
        all_min_point_index = new int[MAX_2D_N];
        fill_n(all_min_point_index, MAX_2D_N, -1);
        fill_n(all_max_point_index, MAX_2D_N, -1);
        map_index = frame;
        index_vector.assign(MAX_2D_N, 0);
        for (int i = 0; i < MAX_2D_N; i++) {
            index_vector[i] = i;
        }
    }

    DepthStructure(const DepthStructure & cur)
    {   
        pixel_point_index = cur.pixel_point_index;       
        time = cur.time;
        R_world_lidar = cur.R_world_lidar;
        t_world_lidar = cur.t_world_lidar;
        point_storage = cur.point_storage;
        depth_tensor.static_min_depth = new float[MAX_2D_N];
        depth_tensor.all_min_depth = new float[MAX_2D_N];
        depth_tensor.all_max_depth = new float[MAX_2D_N];
        depth_tensor.static_max_depth = new float[MAX_2D_N];   
        fill_n(depth_tensor.static_min_depth, MAX_2D_N, 0.0);
        fill_n(depth_tensor.all_min_depth, MAX_2D_N, 0.0);
        fill_n(depth_tensor.all_max_depth, MAX_2D_N, 0.0);
        fill_n(depth_tensor.static_max_depth, MAX_2D_N, 0.0);
        all_max_point_index = new int[MAX_2D_N];       
        all_min_point_index = new int[MAX_2D_N];
        map_index = cur.map_index;      
        for(int i = 0; i < MAX_2D_N; i++)
        {
            depth_tensor.static_min_depth[i] = cur.depth_tensor.static_min_depth[i];
            depth_tensor.static_max_depth[i] = cur.depth_tensor.static_max_depth[i];
            depth_tensor.all_min_depth[i] = cur.depth_tensor.all_min_depth[i];
            depth_tensor.all_max_depth[i] = cur.depth_tensor.all_max_depth[i];
            all_max_point_index[i] = cur.all_max_point_index[i];
            all_min_point_index[i] = cur.all_min_point_index[i];
        }
        index_vector.assign(MAX_2D_N, 0);
        for (int i = 0; i < MAX_2D_N; i++) {
            index_vector[i] = i;
        }
    }
    ~DepthStructure()
    {
        if(depth_tensor.static_min_depth != nullptr) delete [] depth_tensor.static_min_depth;
        if(depth_tensor.all_min_depth  != nullptr) delete [] depth_tensor.all_min_depth;
        if(depth_tensor.all_max_depth  != nullptr) delete [] depth_tensor.all_max_depth;
        if(depth_tensor.static_max_depth  != nullptr) delete [] depth_tensor.static_max_depth;
        if(all_max_point_index  != nullptr) delete [] all_max_point_index;
        if(all_min_point_index  != nullptr) delete [] all_min_point_index;
    }

    void Reset(M3D rot, V3D transl, double cur_time, int frame)
    {   
        time = cur_time;
        R_world_lidar = rot;
        t_world_lidar = transl;
        map_index = frame;
        double t = omp_get_wtime();
        std::for_each(std::execution::par, index_vector.begin(), index_vector.end(), [&](const int &i)
        {
            pixel_point_index[i].clear();
        });
        fill_n(depth_tensor.static_min_depth, MAX_2D_N, 0.0);
        fill_n(depth_tensor.all_min_depth, MAX_2D_N, 0.0);
        fill_n(depth_tensor.all_max_depth, MAX_2D_N, 0.0);
        fill_n(depth_tensor.static_max_depth, MAX_2D_N, 0.0);
        fill_n(all_max_point_index, MAX_2D_N, -1);
        fill_n(all_min_point_index, MAX_2D_N, -1);
    }

};


class MovingObjectRemoval 
{
public:
    std::deque<DepthStructure::Ptr> depth_structure_window;
    PARALLEL_Q<MotionPoint*> buffer;
    std::vector<MotionPoint*> motion_point_pointers;
    int points_num_perframe = 200000;
    int cur_point_soph_pointers = 0;
    int max_pointers_num = 0;
    int frame_num_for_rec = 0;
    std::deque<PointCloudXYZI::Ptr> pcl_his_list;;
    PointCloudXYZI::Ptr laserCloudSteadObj;
    PointCloudXYZI::Ptr laserCloudSteadObj_hist;
    PointCloudXYZI::Ptr laserCloudDynObj;
    PointCloudXYZI::Ptr laserCloudDynObj_world;
    PointCloudXYZI::Ptr laserCloudDynObj_clus;
    PointCloudXYZI::Ptr laserCloudSteadObj_clus;
    std::deque<PointCloudXYZI::Ptr> laserCloudSteadObj_accu;
    int laserCloudSteadObj_accu_times = 0;
    int laserCloudSteadObj_accu_limit = 5;
    float voxel_filter_size = 0.1;
    
    MovingObjectCompletion Cluster;
    bool cluster_coupled = false, cluster_future = false;
    std::vector<int> dyn_tag_cluster;
    std::vector<int> dyn_tag_origin;
    bool draw_depth = false;

    float  depth_thr1 = 0.15f, map_cons_depth_thr1 = 0.5f, map_cons_hor_thr1 = 0.02f, map_cons_ver_thr1 = 0.01f;
    float  enter_min_thr1 = 2.0, enter_max_thr1 = 0.5;
    float  map_cons_hor_dis1 = 0.2, map_cons_ver_dis1 = 0.2;
    int    map_cons_hor_num1 = 0, map_cons_ver_num1 = 0, occluded_map_thr1 = 3;
    bool   entering_leaving_interp_en = false, receding_interp_en = false, approaching_interp_en = false;
    float  interp_hor_thr = 0.01f, interp_ver_thr = 0.01f, interp_thr1 = 1.0f, interp_thr2 = 1.0f,  interp_thr3 = 1.0f;
    float  interp_static_max = 10.0, interp_start_depth1 = 30, interp_kp1 = 0.1, interp_kd1 = 1.0;
    float  interp_all_max = 100.0, interp_start_depth2 = 30, interp_kp2 = 0.1, interp_kd2 = 1.0;
    int    interp_hor_num = 0, interp_ver_num = 0;
    float  v_min_thr2 = 0.5, acc_thr2 = 1.0, v_min_thr3 = 0.5, acc_thr3 = 1.0;
    float  map_cons_depth_thr2 = 0.15f, map_cons_hor_thr2 = 0.02f, map_cons_ver_thr2 = 0.01f;
    float  occ_depth_thr2 = 0.15f, occ_hor_thr2 = 0.02f, occ_ver_thr2 = 0.01f;
    float  depth_cons_depth_thr2 = 0.15f, depth_cons_depth_max_thr2 = 0.15f, depth_cons_hor_thr2 = 0.02f, depth_cons_ver_thr2 = 0.01f;
    float  depth_cons_depth_thr1 = 0.15f, depth_cons_depth_max_thr1 = 1.0f, depth_cons_hor_thr1 = 0.02f, depth_cons_ver_thr1 = 0.01f;
    int    map_cons_hor_num2 = 0, map_cons_ver_num2 = 0, occ_hor_num2 = 0, occ_ver_num2 = 0;
    int    depth_cons_hor_num1 = 0, depth_cons_ver_num1 = 0, depth_cons_hor_num2 = 0, depth_cons_ver_num2 = 0;
    int    occluded_times_thr2 = 3, occluding_times_thr3 = 3;
    float  occ_depth_thr3 = 0.15f, occ_hor_thr3 = 0.02f, occ_ver_thr3 = 0.01f;
    float  map_cons_depth_thr3 = 0.15f, map_cons_hor_thr3 = 0.02f, map_cons_ver_thr3 = 0.01f;
    float  depth_cons_depth_thr3 = 0.15f, depth_cons_depth_max_thr3 = 0.15f, depth_cons_hor_thr3 = 0.02f, depth_cons_ver_thr3 = 0.01f, k_depth2 = 0.005, k_depth3 = 0.005;
    int    map_cons_hor_num3 = 0, map_cons_ver_num3 = 0, occ_hor_num3 = 0, occ_ver_num3 = 0, depth_cons_hor_num3 = 0, depth_cons_ver_num3 = 0;
    float  enlarge_z_thr1 = 0.05, enlarge_angle = 2, enlarge_depth = 3;
    int    enlarge_distort = 4;
    int    checkneighbor_range = 1;

    float  k_depth_min_thr1 = 0.0, d_depth_min_thr1 = 50, cutoff_value = 0;
    float  k_depth_max_thr1 = 0.0, d_depth_max_thr1 = 50, k_depth_max_thr2 = 0.0, d_depth_max_thr2 = 50, k_depth_max_thr3 = 0.0, d_depth_max_thr3 = 50;

    double frame_dur = 0.1, buffer_delay = 0.1, depth_map_dur = 0.2f;
    int    buffer_size = 300000, temporal_window_size = 5;
    int    hor_num = MAX_1D, ver_num = MAX_1D_HALF;
    float  hor_resolution_max = 0.02f, ver_resolution_max = 0.02f;


    int    occu_time_th = 3, is_occu_time_th = 3, map_index = 0;
    int    entering_leaving_num = 0, receding_num = 0, approaching_num = 0;
    
    double time_interp1 = 0.0, time_interp2 = 0.0;
    double time_search = 0.0, time_search_0 = 0.0, time_research = 0.0, time_build = 0.0, time_other0 = 0.0, time_total = 0.0, time_total_avr = 0.0;
    float  buffer_time = 0.0f, buffer_dur = 0.1f;
    int    point_index = 0, time_ind = 0, max_ind = 1257, occlude_windows = 3;
    bool   debug_en = false;
    int    roll_num = 700, pitch_num = 350;
    int    dataset = 0;
    float  self_x_f = 2.5, self_x_b = -1.5, self_y_l = 1.6, self_y_r = -1.6;
    float  fov_up = 2.0, fov_down = -23, fov_cut = -20, fov_left = 180, fov_right = -180;
    float  blind_dis = 0.3;
    int    pixel_fov_up, pixel_fov_down, pixel_fov_cut, pixel_fov_left, pixel_fov_right;
    int    max_pixel_points = 50;
    bool   stop_object_detect = false;
    MotionPoint::Ptr last_point_pointer = nullptr;
    string frame_id = "camera_init";
    double invalid_total = 0.0;
    double entering_leaving_total = 0.0;
    double receding_total = 0.0;
    double approaching_total = 0.0;
    bool dyn_filter_en = true;
    mutex mtx_receding, mtx_approaching; 
    std::vector<int> pos_offset;
    ros::Publisher demo_pcl_display;
    string time_file;
    ofstream time_out;

    std::vector<double> time_test1, time_test2, time_test3, time_occ_check, time_map_cons, time_proj;
    string time_breakdown_file;
    ofstream time_breakdown_out;

    MovingObjectRemoval(){};
    MovingObjectRemoval(float windows_dur, float hor_resolution, float ver_resolution)
    : depth_map_dur(windows_dur), hor_resolution_max(hor_resolution), ver_resolution_max(ver_resolution)
    {};
    ~MovingObjectRemoval() = default;
    MovingObjectRemoval(const MovingObjectRemoval&) = delete;
    MovingObjectRemoval& operator=(const MovingObjectRemoval&) = delete;
    // Classification does not publish or mutate the history until commit_frame.
    bool depth_tensor_detection_enabled = true;
    double depth_interval_margin = 0.15, minimum_radial_velocity = 0.5;
    MotionLabel detectMotion(MotionPoint& p);
    MotionLabel classifyEnteringLeavingDirection(const MotionPoint& p);
    bool defer_history = false;
    std::vector<MotionPoint*> pending_points;
    std::deque<std::pair<double, std::unique_ptr<MotionPoint[]>>> owned_frames;
    std::deque<size_t> owned_frame_bytes;
    size_t retained_point_bytes=0;
    size_t max_history_point_bytes=size_t(2048)*1024*1024;
    double last_frame_time = -std::numeric_limits<double>::infinity();
    void commit_frame(const M3D& rot, const V3D& pos, double time);
    void reset_history();

    void init(ros::NodeHandle& nh);
    void filter(PointCloudXYZI::Ptr feats_undistort, const M3D & rot_end, const V3D & pos_end, const double & scan_end_time);
    void publish_dyn(const ros::Publisher & pub_point_out, const ros::Publisher & pub_frame_out, const ros::Publisher & pub_steady_points, const double & scan_end_time);
    void set_path(string file_path, string file_path_origin);

    void  Points2Buffer(vector<MotionPoint*> &points, std::vector<int> &index_vector);
    void  updateDepthStructureWindow(double cur_time);
    void  SphericalProjection(MotionPoint &p, int depth_index, const M3D &rot, const V3D &transl, MotionPoint &p_spherical);
    bool  detectEnteringLeavingLegacy(MotionPoint & p);
    bool  EnteringLeavingEnter(const MotionPoint & p, const DepthStructure &map_info);
    bool  EnteringLeavingFalseRejection(MotionPoint & p, const DepthStructure &map_info);
    bool  EnteringLeavingMapConsistencyCheck(MotionPoint & p, const DepthStructure &map_info, bool interp);
    float DepthInterpolationStatic(MotionPoint & p, int map_index, const PixelPointIndex &pixel_point_index);
    bool  detectRecedingLegacy(MotionPoint & p);
    bool  RecedingEnter(MotionPoint & p, const DepthStructure &map_info);
    bool  RecedingMapConsistencyCheck(MotionPoint & p, const DepthStructure &map_info, bool interp);
    float DepthInterpolationAll(MotionPoint & p, int map_index, const PixelPointIndex &pixel_point_index);
    bool  RecedingDepthConsistencyCheck(const MotionPoint & p, const DepthStructure &map_info);
    bool  RecedingSearchPointOccludingP(MotionPoint & p, const DepthStructure &map_info);
    bool  RecedingIsOccluded(const MotionPoint & p, const MotionPoint & p_occ);
    bool  RecedingVelCheck(float v1, float v2, double delta_t);
    bool  InvalidPointCheck(const V3D &body, const int intensity);
    bool  SelfPointCheck(const V3D &body, const MotionLabel dyn);
    bool  detectApproachingLegacy(MotionPoint & p);
    bool  ApproachingEnter(MotionPoint & p, const DepthStructure &map_info);
    bool  ApproachingMapConsistencyCheck(MotionPoint & p, const DepthStructure &map_info, bool interp);
    bool  ApproachingSearchPointOccludedbyP(MotionPoint & p, const DepthStructure &map_info);
    bool  ApproachingIsOccluding(const MotionPoint & p, const MotionPoint & p_occ);
    bool  ApproachingVelCheck(float v1, float v2, double delta_t);
    bool  ApproachingDepthConsistencyCheck(const MotionPoint & p, const DepthStructure &map_info);
    bool  CheckVerFoV(const MotionPoint & p, const DepthStructure &map_info);
    void  CheckNeighbor(const MotionPoint & p, const DepthStructure &map_info, float &max_depth, float &min_depth);

private:
    bool is_set_path = false;
    string out_file;
    string out_file_origin;
};

#endif
