#include <iostream>
#include <vector>
#include <random>
#include <numeric>
#include <m-detector/DynObjFilter.h>
// #include <algorithm>
// #include <chrono>
// #include <execution>

#define PI_MATH  (3.14159f)


void  MovingObjectRemoval::init(ros::NodeHandle& nh)
{
    nh.param<bool>("dyn_obj/paper_depth", depth_tensor_detection_enabled, true);
    nh.param<double>("dyn_obj/paper_depth_margin", depth_interval_margin, 0.15);
    nh.param<double>("dyn_obj/paper_min_velocity", minimum_radial_velocity, 0.5);
    if (depth_interval_margin < 0 || minimum_radial_velocity < 0 || !std::isfinite(depth_interval_margin) || !std::isfinite(minimum_radial_velocity))
        throw std::invalid_argument("Invalid paper depth thresholds");
    nh.param<double>("dyn_obj/buffer_delay", buffer_delay, 0.1);
    nh.param<int>("dyn_obj/buffer_size", buffer_size, 300000);
    nh.param<int>("dyn_obj/points_num_perframe", points_num_perframe, 150000);
    int history_point_memory_mb=2048;
    nh.param<int>("dyn_obj/history_point_memory_mb",history_point_memory_mb,2048);
    if(history_point_memory_mb<1) throw std::invalid_argument("History point memory budget must be positive");
    max_history_point_bytes=size_t(history_point_memory_mb)*1024*1024;
    nh.param<double>("dyn_obj/depth_map_dur", depth_map_dur, 0.2);
    nh.param<int>("dyn_obj/max_depth_map_num", temporal_window_size, 5);
    nh.param<int>("dyn_obj/max_pixel_points", max_pixel_points, 50);
    nh.param<double>("dyn_obj/frame_dur", frame_dur, 0.1);
    nh.param<int>("dyn_obj/dataset", dataset, 0);
    nh.param<float>("dyn_obj/self_x_f", self_x_f, 0.15f);
    nh.param<float>("dyn_obj/self_x_b", self_x_b, 0.15f);
    nh.param<float>("dyn_obj/self_y_l", self_y_l, 0.15f);
    nh.param<float>("dyn_obj/self_y_r", self_y_r, 0.5f);
    nh.param<float>("dyn_obj/blind_dis", blind_dis, 0.15f);
    nh.param<float>("dyn_obj/fov_up", fov_up, 0.15f);
    nh.param<float>("dyn_obj/fov_down", fov_down, 0.15f);
    nh.param<float>("dyn_obj/fov_cut", fov_cut, 0.15f);
    nh.param<float>("dyn_obj/fov_left", fov_left, 180.0f);
    nh.param<float>("dyn_obj/fov_right", fov_right, -180.0f);
    nh.param<int>("dyn_obj/checkneighbor_range", checkneighbor_range, 1);
    nh.param<bool>("dyn_obj/stop_object_detect", stop_object_detect, false);
    nh.param<float>("dyn_obj/depth_thr1", depth_thr1, 0.15f);
    nh.param<float>("dyn_obj/enter_min_thr1", enter_min_thr1, 0.15f);
    nh.param<float>("dyn_obj/enter_max_thr1", enter_max_thr1, 0.15f);
    nh.param<float>("dyn_obj/map_cons_depth_thr1", map_cons_depth_thr1, 0.5f);
    nh.param<float>("dyn_obj/map_cons_hor_thr1", map_cons_hor_thr1, 0.01f);
    nh.param<float>("dyn_obj/map_cons_ver_thr1", map_cons_ver_thr1, 0.01f);
    nh.param<float>("dyn_obj/map_cons_hor_dis1", map_cons_hor_dis1, 0.2f);
    nh.param<float>("dyn_obj/map_cons_ver_dis1", map_cons_ver_dis1, 0.1f);
    nh.param<float>("dyn_obj/depth_cons_depth_thr1", depth_cons_depth_thr1, 0.5f);
    nh.param<float>("dyn_obj/depth_cons_depth_max_thr1", depth_cons_depth_max_thr1, 0.5f);
    nh.param<float>("dyn_obj/depth_cons_hor_thr1", depth_cons_hor_thr1, 0.02f);
    nh.param<float>("dyn_obj/depth_cons_ver_thr1", depth_cons_ver_thr1, 0.01f);
    nh.param<float>("dyn_obj/enlarge_z_thr1", enlarge_z_thr1, 0.05f);
    nh.param<float>("dyn_obj/enlarge_angle", enlarge_angle, 2.0f);
    nh.param<float>("dyn_obj/enlarge_depth", enlarge_depth, 3.0f);
    nh.param<int>("dyn_obj/occluded_map_thr1", occluded_map_thr1, 3);
    nh.param<bool>("dyn_obj/case1_interp_en", entering_leaving_interp_en, false);
    nh.param<float>("dyn_obj/k_depth_min_thr1", k_depth_min_thr1, 0.0f);
    nh.param<float>("dyn_obj/d_depth_min_thr1", d_depth_min_thr1, 0.15f);
    nh.param<float>("dyn_obj/k_depth_max_thr1", k_depth_max_thr1, 0.0f);
    nh.param<float>("dyn_obj/d_depth_max_thr1", d_depth_max_thr1, 0.15f);
    nh.param<float>("dyn_obj/v_min_thr2", v_min_thr2, 0.5f);
    nh.param<float>("dyn_obj/acc_thr2", acc_thr2, 1.0f);
    nh.param<float>("dyn_obj/map_cons_depth_thr2", map_cons_depth_thr2, 0.15f);
    nh.param<float>("dyn_obj/map_cons_hor_thr2", map_cons_hor_thr2, 0.02f);
    nh.param<float>("dyn_obj/map_cons_ver_thr2", map_cons_ver_thr2, 0.01f);
    nh.param<float>("dyn_obj/occ_depth_thr2", occ_depth_thr2, 0.15f);
    nh.param<float>("dyn_obj/occ_hor_thr2", occ_hor_thr2, 0.02f);
    nh.param<float>("dyn_obj/occ_ver_thr2", occ_ver_thr2, 0.01f);
    nh.param<float>("dyn_obj/depth_cons_depth_thr2", depth_cons_depth_thr2, 0.5f);
    nh.param<float>("dyn_obj/depth_cons_depth_max_thr2", depth_cons_depth_max_thr2, 0.5f);
    nh.param<float>("dyn_obj/depth_cons_hor_thr2", depth_cons_hor_thr2, 0.02f);
    nh.param<float>("dyn_obj/depth_cons_ver_thr2", depth_cons_ver_thr2, 0.01f);
    nh.param<float>("dyn_obj/k_depth2", k_depth2, 0.005f);
    nh.param<int>("dyn_obj/occluded_times_thr2", occluded_times_thr2, 3);
    nh.param<bool>("dyn_obj/case2_interp_en", receding_interp_en, false);
    nh.param<float>("dyn_obj/k_depth_max_thr2", k_depth_max_thr2, 0.0f);
    nh.param<float>("dyn_obj/d_depth_max_thr2", d_depth_max_thr2, 0.15f);
    nh.param<float>("dyn_obj/v_min_thr3", v_min_thr3, 0.5f);
    nh.param<float>("dyn_obj/acc_thr3", acc_thr3, 1.0f);
    nh.param<float>("dyn_obj/map_cons_depth_thr3", map_cons_depth_thr3, 0.15f);
    nh.param<float>("dyn_obj/map_cons_hor_thr3", map_cons_hor_thr3, 0.02f);
    nh.param<float>("dyn_obj/map_cons_ver_thr3", map_cons_ver_thr3, 0.01f);
    nh.param<float>("dyn_obj/occ_depth_thr3", occ_depth_thr3, 0.15f);
    nh.param<float>("dyn_obj/occ_hor_thr3", occ_hor_thr3, 0.02f);
    nh.param<float>("dyn_obj/occ_ver_thr3", occ_ver_thr3, 0.01f);
    nh.param<float>("dyn_obj/depth_cons_depth_thr3", depth_cons_depth_thr3, 0.5f);
    nh.param<float>("dyn_obj/depth_cons_depth_max_thr3", depth_cons_depth_max_thr3, 0.5f);
    nh.param<float>("dyn_obj/depth_cons_hor_thr3", depth_cons_hor_thr3, 0.02f);
    nh.param<float>("dyn_obj/depth_cons_ver_thr3", depth_cons_ver_thr3, 0.01f);
    nh.param<float>("dyn_obj/k_depth3", k_depth3, 0.005f);
    nh.param<int>("dyn_obj/occluding_times_thr3", occluding_times_thr3, 3);
    nh.param<bool>("dyn_obj/case3_interp_en", approaching_interp_en, false);
    nh.param<float>("dyn_obj/k_depth_max_thr3", k_depth_max_thr3, 0.0f);
    nh.param<float>("dyn_obj/d_depth_max_thr3", d_depth_max_thr3, 0.15f);
    nh.param<float>("dyn_obj/interp_hor_thr", interp_hor_thr, 0.01f);
    nh.param<float>("dyn_obj/interp_ver_thr", interp_ver_thr, 0.01f);
    nh.param<float>("dyn_obj/interp_thr1", interp_thr1, 1.0f);
    nh.param<float>("dyn_obj/interp_static_max", interp_static_max, 10.0f);
    nh.param<float>("dyn_obj/interp_start_depth1", interp_start_depth1, 20.0f);
    nh.param<float>("dyn_obj/interp_kp1", interp_kp1, 0.1f);
    nh.param<float>("dyn_obj/interp_kd1", interp_kd1, 1.0f);
    nh.param<float>("dyn_obj/interp_thr2", interp_thr2, 0.15f);
    nh.param<float>("dyn_obj/interp_thr3", interp_thr3, 0.15f);
    nh.param<bool>("dyn_obj/dyn_filter_en", dyn_filter_en, true);
    nh.param<bool>("dyn_obj/debug_publish", debug_en, true);
    nh.param<int>("dyn_obj/laserCloudSteadObj_accu_limit", laserCloudSteadObj_accu_limit, 5);
    nh.param<float>("dyn_obj/voxel_filter_size", voxel_filter_size, 0.1f);
    nh.param<bool>("dyn_obj/cluster_coupled", cluster_coupled, false);
    nh.param<bool>("dyn_obj/cluster_future", cluster_future, false);
    nh.param<int>("dyn_obj/cluster_extend_pixel", Cluster.parameters.neighbor_radius_voxels, 2);
    nh.param<int>("dyn_obj/cluster_min_pixel_number", Cluster.parameters.min_cluster_voxels, 4);
    nh.param<float>("dyn_obj/cluster_thrustable_thresold", Cluster.parameters.min_seed_fraction, 0.3f);
    nh.param<float>("dyn_obj/cluster_Voxel_revolusion", Cluster.parameters.voxel_resolution, 0.3f);
    nh.param<bool>("dyn_obj/cluster_debug_en", Cluster.debug_en, false);
    nh.param<string>("dyn_obj/cluster_out_file", Cluster.out_file, "");
    nh.param<float>("dyn_obj/ver_resolution_max", ver_resolution_max, 0.0025f);
    nh.param<float>("dyn_obj/hor_resolution_max", hor_resolution_max, 0.0025f);
    nh.param<float>("dyn_obj/buffer_dur", buffer_dur, 0.1f);
    nh.param<int>("dyn_obj/point_index", point_index, 0);
    nh.param<string>("dyn_obj/frame_id", frame_id, "camera_init");
    nh.param<string>("dyn_obj/time_file", time_file, "");
    nh.param<string>("dyn_obj/time_breakdown_file",time_breakdown_file, "");
    if (hor_resolution_max < 0.003f || ver_resolution_max < 0.003f ||
        !std::isfinite(hor_resolution_max) || !std::isfinite(ver_resolution_max) ||
        !std::isfinite(frame_dur) || !std::isfinite(depth_map_dur) || !std::isfinite(buffer_delay) ||
        frame_dur <= 0 || depth_map_dur < frame_dur || buffer_delay < 0 ||
        buffer_size <= 0 || points_num_perframe <= 0 || temporal_window_size < 1 ||
        temporal_window_size > MAP_NUM || max_pixel_points < 1)
        throw std::invalid_argument("Invalid M-detector capacity, resolution or timing parameters");
    md_grid_width = static_cast<int>(std::ceil(2.0 * 3.141592653589793 / hor_resolution_max));
    md_grid_height = static_cast<int>(std::ceil(3.141592653589793 / ver_resolution_max));
    pixel_fov_up = pixel_fov_down = 0;
    max_ind   = floor(3.1415926 * 2 / hor_resolution_max);
    if (pcl_his_list.size() == 0)
    {   
        PointCloudXYZI::Ptr first_frame(new PointCloudXYZI());
        first_frame->reserve(400000);
        pcl_his_list.push_back(first_frame);
        laserCloudSteadObj_hist = PointCloudXYZI::Ptr(new PointCloudXYZI());
        laserCloudSteadObj = PointCloudXYZI::Ptr(new PointCloudXYZI());
        laserCloudDynObj = PointCloudXYZI::Ptr(new PointCloudXYZI());
        laserCloudDynObj_world = PointCloudXYZI::Ptr(new PointCloudXYZI());

    }
    map_cons_hor_num1 = ceil(map_cons_hor_thr1/hor_resolution_max);
    map_cons_ver_num1 = ceil(map_cons_ver_thr1/ver_resolution_max);
    interp_hor_num = ceil(interp_hor_thr/hor_resolution_max);
    interp_ver_num = ceil(interp_ver_thr/ver_resolution_max);
    map_cons_hor_num2 = ceil(map_cons_hor_thr2/hor_resolution_max);
    map_cons_ver_num2 = ceil(map_cons_ver_thr2/ver_resolution_max);
    occ_hor_num2 = ceil(occ_hor_thr2/hor_resolution_max);
    occ_ver_num2 = ceil(occ_ver_thr2/ver_resolution_max);
    depth_cons_hor_num2 = ceil(depth_cons_hor_thr2/hor_resolution_max);
    depth_cons_ver_num2 = ceil(depth_cons_ver_thr2/ver_resolution_max);
    map_cons_hor_num3 = ceil(map_cons_hor_thr3/hor_resolution_max);
    map_cons_ver_num3 = ceil(map_cons_ver_thr3/ver_resolution_max);
    occ_hor_num3 = ceil(occ_hor_thr3/hor_resolution_max);
    occ_ver_num3 = ceil(occ_ver_thr3/ver_resolution_max);
    depth_cons_hor_num3 = ceil(depth_cons_hor_thr3/hor_resolution_max);
    depth_cons_ver_num3 = ceil(depth_cons_ver_thr3/ver_resolution_max);
    buffer.init(buffer_size);

    pixel_fov_up = floor((fov_up/180.0*PI_MATH + 0.5 * PI_MATH)/ver_resolution_max);
    pixel_fov_down = floor((fov_down/180.0*PI_MATH + 0.5 * PI_MATH)/ver_resolution_max);
    pixel_fov_cut = floor((fov_cut/180.0*PI_MATH +  0.5 * PI_MATH)/ver_resolution_max);
    pixel_fov_left = floor((fov_left/180.0*PI_MATH +  PI_MATH)/hor_resolution_max);
    pixel_fov_right = floor((fov_right/180.0*PI_MATH +  PI_MATH)/hor_resolution_max);
    if(time_file != "")
    {
        time_out.open(time_file, ios::out); 
    }
    if(time_breakdown_file != "")
    {
        time_breakdown_out.open(time_breakdown_file, ios::out); 
    }    
    if (cluster_coupled || cluster_future) Cluster.initialize();
}

void  MovingObjectRemoval::filter(PointCloudXYZI::Ptr feats_undistort, const M3D & rot_end, const V3D & pos_end, const double & scan_end_time)
{
    double t00 = omp_get_wtime();
    time_search = time_research = time_search_0 = time_build = time_total = time_other0 = 0.0;
    time_interp1 = time_interp2 =0;
    int num_build = 0, num_search_0 = 0, num_research = 0;
    if (feats_undistort == NULL || feats_undistort->empty()) return;
    if (!pending_points.empty()) throw std::logic_error("Commit the previous frame before classification");
    if (!std::isfinite(scan_end_time)) throw std::invalid_argument("Invalid scan timestamp");
    if (scan_end_time <= last_frame_time) reset_history();
    int size = feats_undistort->points.size();
    if (size > points_num_perframe || size > buffer_size)
        throw std::length_error("Scan exceeds configured M-detector point capacity");
    if (!rot_end.allFinite() || !pos_end.allFinite()) throw std::invalid_argument("Invalid scan pose");
    const size_t frame_bytes=size_t(size)*sizeof(MotionPoint);
    if(frame_bytes>max_history_point_bytes || retained_point_bytes>max_history_point_bytes-frame_bytes)
        throw std::length_error("Dynamic point history memory budget exceeded; reduce point density or history duration");
    owned_frames.emplace_back(scan_end_time, std::unique_ptr<MotionPoint[]>(new MotionPoint[size]));
    owned_frame_bytes.push_back(frame_bytes);
    retained_point_bytes+=frame_bytes;
    if (debug_en)
    {
        laserCloudSteadObj_hist.reset(new PointCloudXYZI());
        laserCloudSteadObj_hist->reserve(20*size);
    }
    dyn_tag_origin.clear();
    dyn_tag_origin.reserve(size);
    dyn_tag_origin.resize(size);
    dyn_tag_cluster.clear();
    dyn_tag_cluster.reserve(size);
    dyn_tag_cluster.resize(size);
    laserCloudDynObj.reset(new PointCloudXYZI());
    laserCloudDynObj->reserve(size);
    laserCloudDynObj_world.reset(new PointCloudXYZI());
    laserCloudDynObj_world->reserve(size);
    laserCloudSteadObj.reset(new PointCloudXYZI());
    laserCloudSteadObj->reserve(size);
    laserCloudDynObj_clus.reset(new PointCloudXYZI());
    laserCloudDynObj_clus->reserve(size);
    laserCloudSteadObj_clus.reset(new PointCloudXYZI());
    laserCloudSteadObj_clus->reserve(size); 
    ofstream out;
    ofstream out_origin;
    bool is_rec = false;
    bool is_rec_origin = false;
    if(is_set_path)
    {
        out.open(out_file, ios::out  | ios::binary);
        out_origin.open(out_file_origin, ios::out | ios::binary);
        if (out.is_open()) 
        {
            is_rec = true;
        }
        if (out_origin.is_open()) 
        {
            is_rec_origin = true;
        }
    }
    time_test1.reserve(size);
    time_test1.resize(size);
    time_test2.reserve(size);
    time_test2.resize(size);
    time_test3.reserve(size);
    time_test3.resize(size);
    time_occ_check.reserve(size);
    time_occ_check.resize(size);
    time_map_cons.reserve(size);
    time_map_cons.resize(size);
    time_proj.reserve(size);
    time_proj.resize(size);
    for(int i = 0; i < size; i++)
    {
        time_test1[i] = 0.0;
        time_test2[i] = 0.0;
        time_test3[i] = 0.0;
        time_occ_check[i] = 0.0;
        time_map_cons[i] = 0.0;
        time_proj[i] = 0.0;
    }
    int receding_num = 0;
    double t0 = omp_get_wtime();
    double time_entering_leaving = 0, time_receding = 0, time_approaching = 0;  
    pcl::PointCloud<PointType> scan_lidar;
    scan_lidar.reserve(size);
    scan_lidar.resize(size);
    std::vector<int> index(size);
    for (int i = 0; i < size; i++) {
        index[i] = i;
    }
    vector<MotionPoint*> points;
    points.reserve(size);
    points.resize(size);
    MotionPoint* p = owned_frames.back().second.get();
    if(time_file != "") time_out << size << " "; //rec computation time
    std::for_each(std::execution::seq, index.begin(), index.end(), [&](const int &i)
    // std::for_each(std::execution::seq, index.begin(), index.end(), [&](const int &i)
    {   
        p[i].reset();     
        V3D p_body(feats_undistort->points[i].x, feats_undistort->points[i].y, feats_undistort->points[i].z);
        int intensity = feats_undistort->points[i].curvature;
        V3D p_glob(rot_end * (p_body) + pos_end);
        p[i].glob = p_glob;        
        p[i].dyn = STATIC;
        p[i].rot = rot_end.transpose();
        p[i].transl = pos_end;
        p[i].time = scan_end_time;
        p[i].local = p_body;
        p[i].intensity = feats_undistort->points[i].intensity;
        if(dataset == 0 && fabs(intensity-666) < 10E-4)
        {
            p[i].is_distort = true;
        }
        if (InvalidPointCheck(p_body, intensity))
        {   
            p[i].dyn = INVALID;
            dyn_tag_origin[i] = 0;
            dyn_tag_cluster[i] = -1;
        }
        else if(SelfPointCheck(p_body, p[i].dyn))
        {
            p[i].dyn = INVALID;
            dyn_tag_origin[i] = 0;
            dyn_tag_cluster[i] = -1;
        }
        else if (depth_tensor_detection_enabled) {
            p[i].dyn = detectMotion(p[i]);
            dyn_tag_origin[i] = IsDynamicMotion(p[i].dyn) ? 1 : 0;
        }
        else if (detectEnteringLeavingLegacy(p[i]))
        {
            p[i].dyn = classifyEnteringLeavingDirection(p[i]);
            dyn_tag_origin[i] = 1;
        }
        else if (detectRecedingLegacy(p[i]))
        {
            p[i].dyn = RECEDING;
            dyn_tag_origin[i] = 1;
        } 
        else if(detectApproachingLegacy(p[i]))
        {
            p[i].dyn = APPROACHING;
            dyn_tag_origin[i] = 1;
        }
        else
        {
            dyn_tag_origin[i] = 0;
        }
        points[i] = &p[i];
    });   
    if(time_file != "") time_out << omp_get_wtime()-t0 << " "; //rec computation time 
    for(int i = 0; i < size; i++)
    {
        PointType po;
        po.x = points[i]->local[0];
        po.y = points[i]->local[1];
        po.z = points[i]->local[2];
        po.intensity = points[i]->intensity;
        PointType po_w;
        po_w.x = points[i]->glob[0];
        po_w.y = points[i]->glob[1];
        po_w.z = points[i]->glob[2];
        scan_lidar[i] = po;
        switch(points[i]->dyn)
        {
            case LEAVING:
                case ENTERING:
                po.normal_x = 1;
                laserCloudDynObj->push_back(po);
                laserCloudDynObj_world->push_back(po_w);
                break;
            case RECEDING:
                po.normal_y = points[i]->occu_times;
                laserCloudDynObj->push_back(po);
                laserCloudDynObj_world->push_back(po_w);
                break;
            case APPROACHING:
                po.normal_z = points[i]->is_occu_times;
                laserCloudDynObj->push_back(po);
                laserCloudDynObj_world->push_back(po_w);
                break;
            default:
                laserCloudSteadObj->push_back(po_w);
        }
    }
	int num_1 = 0, num_2 = 0, num_3 = 0, num_inval = 0, num_neag = 0; 
    double clus_before = omp_get_wtime(); //rec computation time
    std_msgs::Header header_clus;
    header_clus.stamp = ros::Time().fromSec(scan_end_time);
    header_clus.frame_id = frame_id;
    if (cluster_coupled || cluster_future)
    {
        Cluster.refineDynamicMask(dyn_tag_cluster, *laserCloudDynObj, scan_lidar, header_clus, rot_end, pos_end);
        for(int i = 0; i < size; i++)
        {   
            PointType po;
            po.x = points[i]->glob(0);
            po.y = points[i]->glob(1);
            po.z = points[i]->glob(2);
            po.curvature = i;
            switch (points[i]->dyn)
            {   
                case LEAVING:
                case ENTERING:               
                    if (dyn_tag_cluster[i] == 0) 
                    {
                        points[i]->dyn = STATIC;
                        points[i]->occu_times = -1;
                        points[i]->is_occu_times = -1;
                        po.normal_x = 0;
                        po.normal_y = points[i]->is_occu_times;
                        po.normal_z = points[i]->occu_times;
                        po.intensity = (int) (points[i]->local.norm() * 10) + 10;
                        laserCloudSteadObj_clus->push_back(po);
                        num_neag += 1;
                    }
                    else // case1
                    {
                        po.normal_x = 1;
                        po.normal_y = points[i]->is_occu_times;
                        po.normal_z = points[i]->occu_times;
                        laserCloudDynObj_clus->push_back(po);
                        if(!dyn_filter_en)
                        {
                            po.intensity = (int) (points[i]->local.norm() * 10) + 10;
                        }
                        num_1 += 1;
                    }
                    break;               
                case RECEDING:
                    if(dyn_tag_cluster[i] == 0)
                    {
                        points[i]->dyn = STATIC;
                        points[i]->occu_times = -1;
                        points[i]->is_occu_times = -1;
                        po.normal_x = 0;
                        po.normal_y = points[i]->is_occu_times;
                        po.normal_z = points[i]->occu_times;
                        po.intensity = (int) (points[i]->local.norm() * 10) + 10;
                        laserCloudSteadObj_clus->push_back(po);
                        num_neag += 1;
                    }
                    else
                    {
                        po.normal_x = 0;
                        po.normal_y = points[i]->is_occu_times;
                        po.normal_z = points[i]->occu_times;
                        laserCloudDynObj_clus->push_back(po);
                        if(!dyn_filter_en)
                        {
                            po.intensity = (int) (points[i]->local.norm() * 10) + 10;
                        }
                        num_2 += 1;
                    }
                    break;
                case APPROACHING:
                    if(dyn_tag_cluster[i] == 0)
                    {
                        points[i]->dyn = STATIC;
                        points[i]->occu_times = -1;
                        points[i]->is_occu_times = -1;
                        po.normal_x = 0;
                        po.normal_y = points[i]->is_occu_times;
                        po.normal_z = points[i]->occu_times;
                        po.intensity = (int) (points[i]->local.norm() * 10) + 10;
                        laserCloudSteadObj_clus->push_back(po);
                        num_neag += 1;
                    }
                    else
                    {
                        po.normal_x = 0;
                        po.normal_y = points[i]->is_occu_times;
                        po.normal_z = points[i]->occu_times;
                        laserCloudDynObj_clus->push_back(po);
                        if(!dyn_filter_en)
                        {
                            po.intensity = (int) (points[i]->local.norm() * 10) + 10;
                        }
                        num_3 += 1;
                    }
                    break;       
                case STATIC:
                    if(dyn_tag_cluster[i] == 1)
                    {
                        points[i]->dyn = classifyEnteringLeavingDirection(*points[i]);
                        points[i]->occu_times = -1;
                        points[i]->is_occu_times = -1;
                        po.normal_x = 0;
                        po.normal_y = points[i]->is_occu_times;
                        po.normal_z = points[i]->occu_times;
                        laserCloudDynObj_clus->push_back(po);
                        if(!dyn_filter_en)
                        {
                            po.intensity = (int) (points[i]->local.norm() * 10) + 10;
                        }
                        num_1 += 1;
                    }
                    else
                    {
                        po.normal_x = 0;
                        po.normal_y = points[i]->is_occu_times;
                        po.normal_z = points[i]->occu_times;
                        po.intensity = (int) (points[i]->local.norm() * 10) + 10;
                        laserCloudSteadObj_clus->push_back(po);
                        num_neag += 1;
                    }
                    break;               
                default: //invalid
                    num_inval += 1;
                    break;
            }
        }
    }
    if(time_file != "") time_out << omp_get_wtime()-clus_before << " "; //rec computation time  
    double t3 = omp_get_wtime();
    pending_points = points;
    if (!defer_history) commit_frame(rot_end, pos_end, scan_end_time);
    double t4 = omp_get_wtime();
    if(time_file != "") time_out << omp_get_wtime()-t3 << " "; //rec computation time
    // History is committed once, using the final pose.
    if(time_file != "") time_out << omp_get_wtime()-t3 << endl; //rec computation time
    if (cluster_coupled)
    {   
        for(int i = 0; i < size; i++)
        {
            if (dyn_tag_cluster[i] == 1)
            {
                if(is_rec) 
                {
                    int tmp = 251;
                    out.write((char*)&tmp, sizeof(int));
                }
            } 
            else
            {
                if(is_rec)
                {
                    int tmp = 9;
                    out.write((char*)&tmp, sizeof(int));
                } 
            } 

            if (dyn_tag_origin[i] == 1 || dyn_tag_origin[i] == 2)
            {
                if(is_rec_origin) 
                {
                    int tmp = 251;
                    out_origin.write((char*)&tmp, sizeof(int));
                }
            } 
            else
            {
                if(is_rec_origin)
                {
                    int tmp = 9;
                    out_origin.write((char*)&tmp, sizeof(int));
                }
            } 
        }
    }
    else
    {
        for(int i = 0; i < size; i++)
        {
            if (is_rec_origin) {
                int label = dyn_tag_origin[i] == 1 ? 251 : 9;
                out_origin.write(reinterpret_cast<const char*>(&label), sizeof(label));
            }
            if (dyn_tag_origin[i] == 1 || dyn_tag_origin[i] == 2)
            {
                if(is_rec) 
                {
                    int tmp = 251;
                    out.write((char*)&tmp, sizeof(int));
                }
            } 
            else
            {
                if(is_rec)
                {
                    int tmp = 9;
                    out.write((char*)&tmp, sizeof(int));
                }
            } 
        }
    }
    double total_test1 = 0, total_test2 = 0, total_test3 = 0, total_proj = 0, total_occ = 0, total_map = 0;
    for (int i = 0; i < size; i++)
    {
        total_test1 += time_test1[i];
        total_test2 += time_test2[i];
        total_test3 += time_test3[i];
        total_proj += time_proj[i];
        total_occ += time_occ_check[i];
        total_map += time_map_cons[i];
    }
    if(time_breakdown_file != "") 
    {
        time_breakdown_out << total_test1 << " " << total_test2 << " "<< total_test3 << " " << total_proj << " " << total_occ << " " << total_map << endl;
    }
    frame_num_for_rec ++;

    if(is_rec) out.close();
    time_total = omp_get_wtime() - t00;
    time_ind ++;
    time_total_avr = time_total_avr * (time_ind - 1) / time_ind + time_total / time_ind;
}

void  MovingObjectRemoval::Points2Buffer(vector<MotionPoint*> &points, std::vector<int> &index_vector)
{
    int cur_tail = buffer.tail;
    buffer.push_parallel_prepare(points.size());
    std::for_each(std::execution::seq, index_vector.begin(), index_vector.end(), [&](const int &i)
    {   
        buffer.push_parallel(points[i], cur_tail+i);
    });
}

void  MovingObjectRemoval::updateDepthStructureWindow(double cur_time)
{
    int len = buffer.size();
    double total_0 = 0.0;
    double total_1 = 0.0;
    double total_2 = 0.0;
    double total_3 = 0.0;
    double t = 0.0;
    int max_point = 0;
    for (int k = 0; k < len; k++)
    {   
        MotionPoint* point = buffer.front();
        if ((cur_time - point->time) >= buffer_delay - frame_dur/2.0)
        {   
            if(depth_structure_window.size() == 0)
            {
                if(depth_structure_window.size() < temporal_window_size)
                {
                    map_index ++;
                    DepthStructure::Ptr new_map_pointer(new DepthStructure(point->rot, point->transl, point->time, map_index));
                    depth_structure_window.push_back(new_map_pointer);
                }
                else
                {
                    buffer.pop();
                    continue;
                }          
            }
            else if((point->time - depth_structure_window.back()->time) >= depth_map_dur - frame_dur/2.0)
            {
                map_index ++;
                if (depth_structure_window.size() == temporal_window_size)
                {   
                    depth_structure_window.front()->Reset(point->rot, point->transl, point->time, map_index);
                    DepthStructure::Ptr new_map_pointer = depth_structure_window.front();
                    depth_structure_window.pop_front();
                    depth_structure_window.push_back(new_map_pointer);
                }
                else if (depth_structure_window.size() < temporal_window_size)
                {   
                    DepthStructure::Ptr new_map_pointer(new DepthStructure(point->rot, point->transl, point->time, map_index));
                    depth_structure_window.push_back(new_map_pointer);
                }
            }
            switch (point->dyn)
            {

                case STATIC:
                    SphericalProjection(*point, depth_structure_window.back()->map_index, depth_structure_window.back()->R_world_lidar, depth_structure_window.back()->t_world_lidar, *point);
                    if (point->position < 0 || point->position >= MAX_2D_N || point->ver_ind < 0 || point->ver_ind >= MAX_1D_HALF) break;                  
                    if(depth_tensor_detection_enabled || depth_structure_window.back()->pixel_point_index[point->position].size() < max_pixel_points)
                    {
                        depth_structure_window.back()->pixel_point_index[point->position].push_back(point);
                        if (point->vec(2) > depth_structure_window.back()->depth_tensor.all_max_depth[point->position])  
                        {
                            depth_structure_window.back()->depth_tensor.all_max_depth[point->position] = point->vec(2);
                            depth_structure_window.back()->all_max_point_index[point->position] = depth_structure_window.back()->pixel_point_index[point->position].size()-1;
                        }
                        if (point->vec(2) < depth_structure_window.back()->depth_tensor.all_min_depth[point->position] ||\
                            depth_structure_window.back()->depth_tensor.all_min_depth[point->position] < 10E-5)  
                        {
                            depth_structure_window.back()->depth_tensor.all_min_depth[point->position] = point->vec(2);
                            depth_structure_window.back()->all_min_point_index[point->position] = depth_structure_window.back()->pixel_point_index[point->position].size()-1;
                        }
                        if (point->vec(2) < depth_structure_window.back()->depth_tensor.static_min_depth[point->position] ||\
                            depth_structure_window.back()->depth_tensor.static_min_depth[point->position] < 10E-5)  
                        {
                            depth_structure_window.back()->depth_tensor.static_min_depth[point->position] = point->vec(2);
                        }
                        if (point->vec(2) > depth_structure_window.back()->depth_tensor.static_max_depth[point->position])  
                        {
                            depth_structure_window.back()->depth_tensor.static_max_depth[point->position] = point->vec(2);
                        }                        
                    }
                    break;   
                case LEAVING:
                case ENTERING:   

                case RECEDING:

                case APPROACHING:
                    SphericalProjection(*point, depth_structure_window.back()->map_index, depth_structure_window.back()->R_world_lidar, depth_structure_window.back()->t_world_lidar, *point);
                    if (point->position < 0 || point->position >= MAX_2D_N || point->ver_ind < 0 || point->ver_ind >= MAX_1D_HALF) break;
                    if(depth_tensor_detection_enabled || depth_structure_window.back()->pixel_point_index[point->position].size() < max_pixel_points)
                    {
                        depth_structure_window.back()->pixel_point_index[point->position].push_back(point);
                        if (point->vec(2) > depth_structure_window.back()->depth_tensor.all_max_depth[point->position])  
                        {
                            depth_structure_window.back()->depth_tensor.all_max_depth[point->position] = point->vec(2);
                            depth_structure_window.back()->all_max_point_index[point->position] = depth_structure_window.back()->pixel_point_index[point->position].size()-1;
                        }
                        if (point->vec(2) < depth_structure_window.back()->depth_tensor.all_min_depth[point->position] ||\
                            depth_structure_window.back()->depth_tensor.all_min_depth[point->position] < 10E-5)  
                        {
                            depth_structure_window.back()->depth_tensor.all_min_depth[point->position] = point->vec(2);
                            depth_structure_window.back()->all_min_point_index[point->position] = depth_structure_window.back()->pixel_point_index[point->position].size()-1;
                        }
                    }
                    break;
                default:    
                    break;
            }
            buffer.pop();
        }
        else
        {
            break;
        }
    }
    if (debug_en)
    {   
        for (int i = 0; i < depth_structure_window.size(); i++)
        {
            for (int j = 0; j < depth_structure_window[i]->pixel_point_index.size(); j++)
            {
                for (int k = 0; k < depth_structure_window[i]->pixel_point_index[j].size(); k++)
                {
                    PointType po;
                    MotionPoint* point = depth_structure_window[i]->pixel_point_index[j][k];
                    po.x = point->glob(0);
                    po.y = point->glob(1);
                    po.z = point->glob(2);
                    po.intensity = point->local(2);
                    po.curvature = point->local(1);
                    po.normal_x = point->hor_ind;
                    po.normal_y = point->ver_ind;
                    po.normal_z = point->dyn;
                    if(point->dyn == STATIC) laserCloudSteadObj_hist->push_back(po);
                }
            }
        }
    }
}

void  MovingObjectRemoval::SphericalProjection(MotionPoint &p, int depth_index, const M3D &rot, const V3D &transl, MotionPoint &p_spherical)
{
    if(fabs(p.last_vecs.at(depth_index%HASH_PRIM)[2]) > 10E-5)
    {       
        p_spherical.vec = p.last_vecs.at(depth_index%HASH_PRIM);
        p_spherical.hor_ind = p.last_positions.at(depth_index%HASH_PRIM)[0];
        p_spherical.ver_ind = p.last_positions.at(depth_index%HASH_PRIM)[1];
        p_spherical.position = p.last_positions.at(depth_index%HASH_PRIM)[2];     
    }
    else
    {
        V3D p_proj(rot*(p.glob - transl));
        p_spherical.GetVec(p_proj, hor_resolution_max, ver_resolution_max);
        p.last_vecs.at(depth_index%HASH_PRIM) = p_spherical.vec;
        p.last_positions.at(depth_index%HASH_PRIM)[0] = p_spherical.hor_ind;
        p.last_positions.at(depth_index%HASH_PRIM)[1] = p_spherical.ver_ind;
        p.last_positions.at(depth_index%HASH_PRIM)[2] = p_spherical.position;
    }
}

bool  MovingObjectRemoval::InvalidPointCheck(const V3D &body, const int intensity)
{
    if (!body.allFinite()) return true;
    if ((pow(body(0), 2) + pow(body(1), 2) + pow(body(2), 2)) < blind_dis*blind_dis || (dataset == 1 && fabs(body(0)) < 0.1 && fabs(body(1)) < 1.0) && fabs(body(2)) < 0.1)
    {
        return true;
    } 
    else
    {
        return false;
    }
}

bool  MovingObjectRemoval::SelfPointCheck(const V3D &body, const MotionLabel dyn)
{
    if (dataset == 0)
    {      
        if( (body(0) > -1.2 && body(0) < -0.4 && body(1) > -1.7 && body(1) < -1.0 && body(2) > -0.65 && body(2) < -0.4) || \
            (body(0) > -1.75 && body(0) < -0.85 && body(1) > 1.0 && body(1) < 1.6 && body(2) > -0.75 && body(2) < -0.40) || \
            (body(0) > 1.4 && body(0) < 1.7 && body(1) > -1.3 && body(1) < -0.9 && body(2) > -0.8 && body(2) < -0.6) || \
            (body(0) > 2.45 && body(0) < 2.6 && body(1) > -0.6 && body(1) < -0.45 && body(2) > -1.0 && body(2) < -0.9) || \
            (body(0) > 2.45 && body(0) < 2.6 && body(1) > 0.45 && body(1) < 0.6 && body(2) > -1.0 && body(2) < -0.9) )
        {
            return true;
        }
        else
        {
            return false;
        }
    } 
    return false;
}

bool  MovingObjectRemoval::CheckVerFoV(const MotionPoint & p, const DepthStructure &map_info)
{
    bool ver_up = false, ver_down = false;
    for(int i = p.ver_ind; i >= std::max(0, pixel_fov_down); i--)
    {   
        int cur_pos = p.hor_ind * MAX_1D_HALF + i;
        if(cur_pos >= 0 && cur_pos < MAX_2D_N && map_info.pixel_point_index[cur_pos].size() > 0)
        {
            ver_down = true;
            break;
        }
    } 
    for(int i = p.ver_ind; i <= std::min(MAX_1D_HALF - 1, pixel_fov_up); i++)
    {   
        int cur_pos = p.hor_ind * MAX_1D_HALF + i;
        if(cur_pos >= 0 && cur_pos < MAX_2D_N && map_info.pixel_point_index[cur_pos].size() > 0)
        {
            ver_up = true;
            break;
        }
    }   
    if(ver_up && ver_down)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void MovingObjectRemoval::CheckNeighbor(const MotionPoint & p, const DepthStructure &map_info, float &max_depth, float &min_depth)
{   
    int n = checkneighbor_range;
    for (int i = -n; i <= n; i++)
    {
        for (int j = -n; j <= n; j++)
        {
            int cur_pos = (p.hor_ind + i) * MAX_1D_HALF + p.ver_ind + j;
            if(cur_pos < MAX_2D_N && cur_pos >= 0 && map_info.pixel_point_index[cur_pos].size() > 0)
            {
                float cur_max_depth = map_info.depth_tensor.static_max_depth[cur_pos];
                float cur_min_depth = map_info.depth_tensor.static_min_depth[cur_pos];
                if(min_depth > 10E-5) min_depth = std::min(cur_min_depth, min_depth);
                else min_depth = cur_min_depth;
                if(max_depth > 10E-5) max_depth = std::max(cur_max_depth, max_depth);
                else max_depth = cur_max_depth;
            }
        }
    }
}

bool  MovingObjectRemoval::detectEnteringLeavingLegacy(MotionPoint & p)
{
    int depth_map_num = depth_structure_window.size();
    int occluded_map = depth_map_num;
    for (int i = depth_map_num- 1; i >= 0; i--)
    {   
        SphericalProjection(p, depth_structure_window[i]->map_index, depth_structure_window[i]->R_world_lidar, depth_structure_window[i]->t_world_lidar, p);
        if (fabs(p.hor_ind) > MAX_1D || fabs(p.ver_ind) > MAX_1D_HALF || p.vec(2) < 0.0f \
            || p.position < 0 || p.position >= MAX_2D_N)
        {
            p.dyn = INVALID;
            continue;
        }
        if (EnteringLeavingEnter(p, *depth_structure_window[i]))
        { 
            if (EnteringLeavingFalseRejection(p, *depth_structure_window[i]))
            {
                occluded_map -= 1;
            }
        }
        else
        {
            occluded_map -= 1;
        }
        if (occluded_map < occluded_map_thr1)
        {   
            return false;
        }
        if (occluded_map -(i) >= occluded_map_thr1)
        {
            return true;
        }
    }
    if(occluded_map >= occluded_map_thr1)
    {
        return true;
    }
    return false;  
}

bool  MovingObjectRemoval::EnteringLeavingEnter(const MotionPoint & p, const DepthStructure &map_info)
{
    float max_depth = 0, min_depth = 0;
    float depth_tensor.all_max_depth = 0, depth_tensor.all_min_depth = 0;
    if (map_info.pixel_point_index[p.position].size() > 0)
    {
        max_depth = map_info.depth_tensor.static_max_depth[p.position];
        min_depth = map_info.depth_tensor.static_min_depth[p.position];
    }
    else 
    {
        if(p.ver_ind <= pixel_fov_up && p.ver_ind >pixel_fov_down && \
           p.hor_ind <= pixel_fov_left && p.hor_ind >= pixel_fov_right && \
           CheckVerFoV(p, map_info))
        {
            CheckNeighbor(p, map_info, max_depth, min_depth);
        }
    }        
    float cur_min = max(cutoff_value, k_depth_min_thr1*(p.vec(2) - d_depth_min_thr1)) + enter_min_thr1;
    float cur_max = max(cutoff_value, k_depth_max_thr1*(p.vec(2) - d_depth_max_thr1)) + enter_max_thr1;
    float cur_depth = depth_thr1;
    if(dataset == 0 && p.is_distort) 
    {
        cur_min = enlarge_distort*cur_min;
        cur_max = enlarge_distort*cur_max;
        cur_depth = enlarge_distort*cur_depth;
    }
    if (p.vec(2) < min_depth - cur_max || \
        (min_depth < p.vec(2) - cur_min && max_depth > p.vec(2) + cur_max)||\
        (stop_object_detect && min_depth <10E-5 && max_depth < 10E-5 && map_info.pixel_point_index[p.position].size() > 0 && p.vec(2) < map_info.depth_tensor.all_max_depth[p.position] + 1.0)
        )
    {
        entering_leaving_num ++;
        return true;
    }
    return false;
}

bool  MovingObjectRemoval::EnteringLeavingFalseRejection(MotionPoint & p, const DepthStructure &map_info)
{
    return EnteringLeavingMapConsistencyCheck(p, map_info, entering_leaving_interp_en);
}

bool  MovingObjectRemoval::EnteringLeavingMapConsistencyCheck(MotionPoint & p, const DepthStructure &map_info, bool interp)
{   
    float hor_half = max(map_cons_hor_dis1/(max(p.vec(2), blind_dis)), map_cons_hor_thr1);
    float ver_half = max(map_cons_ver_dis1/(max(p.vec(2), blind_dis)), map_cons_ver_thr1);
    float cur_map_cons_depth_thr1 = max(cutoff_value, k_depth_max_thr1*(p.vec(2) - d_depth_max_thr1)) + map_cons_depth_thr1;
    float cur_map_cons_min_thr1 = max(cutoff_value, k_depth_min_thr1*(p.vec(2) - d_depth_min_thr1)) + enter_min_thr1;
    float cur_map_cons_max_thr1 = max(cutoff_value, k_depth_max_thr1*(p.vec(2) - d_depth_max_thr1)) + enter_max_thr1;
    if(dataset == 0 && p.is_distort) 
    {
        cur_map_cons_depth_thr1 = enlarge_distort*cur_map_cons_depth_thr1;
        cur_map_cons_min_thr1 = enlarge_distort*cur_map_cons_min_thr1;
        cur_map_cons_max_thr1 = enlarge_distort*cur_map_cons_max_thr1;
    }
    if (fabs(p.vec(1)) < enlarge_z_thr1 / 57.3)
    {
        hor_half = enlarge_angle * hor_half;
        ver_half = enlarge_angle * ver_half;
        cur_map_cons_depth_thr1 = enlarge_depth * cur_map_cons_depth_thr1;
    }
    int cur_map_cons_hor_num1 = ceil(hor_half/hor_resolution_max);
    int cur_map_cons_ver_num1 = ceil(ver_half/ver_resolution_max);
    int num = 0;
    MotionPoint closest_point;
    float closest_dis = 100;
    for (int ind_hor = -cur_map_cons_hor_num1; ind_hor <= cur_map_cons_hor_num1; ind_hor ++)
    {
        for (int ind_ver = -cur_map_cons_ver_num1; ind_ver <= cur_map_cons_ver_num1; ind_ver ++)
        {   
            int pos_new = md_neighbor_position(p.hor_ind, p.ver_ind, ind_hor, ind_ver);
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
            const vector<MotionPoint*> & points_in_pixel = map_info.pixel_point_index[pos_new];                        
            if (map_info.depth_tensor.static_max_depth[pos_new] < p.vec(2) - cur_map_cons_min_thr1 || \
                map_info.depth_tensor.static_min_depth[pos_new] > p.vec(2) + cur_map_cons_max_thr1)
            {
                continue;
            }   
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const MotionPoint* point = points_in_pixel[j];
                if (point->dyn == STATIC &&\
                  (fabs(p.vec(2)-point->vec(2)) <  cur_map_cons_depth_thr1 ||\
                   ((p.vec(2)-point->vec(2)) >  cur_map_cons_depth_thr1 && (p.vec(2)-point->vec(2)) <  cur_map_cons_min_thr1)) && \
                    fabs(p.vec(0)-point->vec(0)) < hor_half && \
                  fabs(p.vec(1)-point->vec(1)) < ver_half)
                {
                    return true;
                }    
            }         
        }
    }   
    if(interp && (p.local(0) < self_x_b || p.local(0) > self_x_f || p.local(1) > self_y_l || p.local(1) < self_y_r))
    {
        float depth_static = DepthInterpolationStatic(p, map_info.map_index, map_info.pixel_point_index);
        float cur_interp = interp_thr1;
        if(p.vec(2) > interp_start_depth1)
            cur_interp += ((p.vec(2) - interp_start_depth1)* interp_kp1 + interp_kd1);
        if(dataset == 0 )
        {
            if(fabs(depth_static+1) < 10E-5 || fabs(depth_static+2) < 10E-5)
            {
                return false;
            }
            else 
            {
                if(fabs(depth_static - p.vec(2)) < cur_interp)
                {
                    return true;
                } 
            }
        }
        else
        {
            if(fabs(depth_static+1) < 10E-5 || fabs(depth_static+2) < 10E-5)
            {
                return false;
            }
            else 
            {
                if(fabs(depth_static - p.vec(2)) < cur_interp)
                {
                    return true;
                } 
            }        
        }     
    }
    return false;
}

float MovingObjectRemoval::DepthInterpolationStatic(MotionPoint & p, int map_index, const PixelPointIndex &pixel_point_index)
{
    if(fabs(p.last_depth_interps.at(map_index - depth_structure_window.front()->map_index)) > 10E-4)
    {
        float depth_cal = p.last_depth_interps.at(map_index - depth_structure_window.front()->map_index);
        return depth_cal;
    }
    V3F p_1 = V3F::Zero();
    V3F p_2 = V3F::Zero();
    V3F p_3 = V3F::Zero();
    vector<V3F> p_neighbors;
    int all_num = 0, static_num = 0, no_bg_num = 0;     
    for (int ind_hor = -interp_hor_num; ind_hor <= interp_hor_num; ind_hor ++)
    {
        for (int ind_ver = -interp_ver_num; ind_ver <= interp_ver_num; ind_ver ++)
        {   
            int pos_new = md_neighbor_position(p.hor_ind, p.ver_ind, ind_hor, ind_ver); 
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
            const vector<MotionPoint*> & points_in_pixel = pixel_point_index[pos_new];     
            for (int j = 0; j < points_in_pixel.size(); j ++)
            {   
                const MotionPoint* point = points_in_pixel[j]; 
                if (fabs(point->time - p.time) < frame_dur)
                {
                    continue;
                }
                float hor_minus =  point->vec(0) - p.vec(0);
                float ver_minus =  point->vec(1) - p.vec(1);
                if (fabs(hor_minus) < interp_hor_thr && fabs(ver_minus) < interp_ver_thr)
                {
                    all_num ++;
                    if(point->dyn == STATIC) 
                    {
                        static_num ++;
                    }
                    if((point->vec(2) - p.vec(2)) <= interp_static_max && (p.vec(2) - point->vec(2)) < 5.0)
                    {
                        no_bg_num ++;
                    }
                    if(point->dyn == STATIC)
                    {
                        p_neighbors.push_back(point->vec);
                        if (p_1(2)<0.000001 || fabs(hor_minus) + fabs(ver_minus) < fabs(p_1(0) - p.vec(0)) + fabs(p_1(1) - p.vec(1)))
                        {
                            p_1 = point->vec;
                        }
                    }
                }
            }
        }
    }
    if (p_1(2)<10E-5)
    {
        p.last_depth_interps.at(map_index - depth_structure_window.front()->map_index) = -1;
        return -1;
    }
    int cur_size = p_neighbors.size();
    for(int t_i = 0; t_i < cur_size-2; t_i++)
    {
        p_1 = p_neighbors[t_i];
        p_2 = V3F::Zero();
        p_3 = V3F::Zero();
        float min_fabs = 2*(interp_hor_thr + interp_ver_thr);
        float x = p.vec(0) - p_1(0);
        float y = p.vec(1) - p_1(1);
        float alpha  = 0, beta = 0;
        for(int i = t_i+1; i < cur_size-1; i++)
        {           
            if(fabs(p_neighbors[i](0)-p.vec(0)) + fabs(p_neighbors[i](1)-p.vec(1)) < min_fabs)
            {
                p_2 = p_neighbors[i];
                float single_fabs = fabs(p_neighbors[i](0)-p.vec(0)) + fabs(p_neighbors[i](1)-p.vec(1));
                if (single_fabs >= min_fabs) continue;
                for(int ii = i+1; ii < cur_size; ii++)
                {
                    float cur_fabs = fabs(p_neighbors[i](0)-p.vec(0)) + fabs(p_neighbors[i](1)-p.vec(1)) + \
                                    fabs(p_neighbors[ii](0)-p.vec(0)) + fabs(p_neighbors[ii](1)-p.vec(1));
                    if( cur_fabs < min_fabs)
                    {
                        float x1 = p_neighbors[i](0) - p_1(0);
                        float x2 = p_neighbors[ii](0) - p_1(0);
                        float y1 = p_neighbors[i](1) - p_1(1);
                        float y2 = p_neighbors[ii](1) - p_1(1);
                        float lower = x1*y2-x2*y1;
                        if(fabs(lower) > 10E-5)
                        {
                            alpha = (x*y2-y*x2)/lower;
                            beta = -(x*y1-y*x1)/lower;
                            if(alpha > 0 && alpha < 1 && beta > 0 && beta < 1 && (alpha + beta) > 0 && (alpha + beta) < 1)
                            {
                                p_3 = p_neighbors[ii];
                                min_fabs = cur_fabs; 
                            }
                            
                        }
                    }
                }  
            }
        }
        if (p_2(2)<10E-5 || p_3(2)<10E-5)
        {
            continue;
        }
        float depth_cal = (1-alpha-beta)*p_1(2) + alpha*p_2(2) + beta*p_3(2);
        p.last_depth_interps.at(map_index - depth_structure_window.front()->map_index) = depth_cal;
        return depth_cal;
    }
    if(static_num > 0 && cur_size < all_num/2)
    {
        p.last_depth_interps.at(map_index - depth_structure_window.front()->map_index) = -2;
        return -2;
    }
    else
    {
        p.last_depth_interps.at(map_index - depth_structure_window.front()->map_index) = -2;
        return -2;
    }
}// return -1 denotes no point, -2 denotes no trianguolar but with points

bool  MovingObjectRemoval::detectRecedingLegacy(MotionPoint & p)
{   
    if(dataset == 0 && p.is_distort) return false;
    int first_i = depth_structure_window.size();
    first_i -= 1;
    if(first_i < 0) return false;
    MotionPoint p_spherical = p;   
    SphericalProjection(p, depth_structure_window[first_i]->map_index, depth_structure_window[first_i]->R_world_lidar, depth_structure_window[first_i]->t_world_lidar, p_spherical);  
    if (fabs(p_spherical.hor_ind) >= MAX_1D || fabs(p_spherical.ver_ind) >= MAX_1D_HALF || p_spherical.vec(2) < 0.0f || \
        p_spherical.position < 0 || p_spherical.position >= MAX_2D_N)
    {
        p.dyn = INVALID;
        return false;
    }
    int cur_occ_times = 0;
    if (RecedingEnter(p_spherical, *depth_structure_window[first_i]))
    {
        if (!RecedingMapConsistencyCheck(p_spherical, *depth_structure_window[first_i], receding_interp_en))
        {
            double ti = 0;
            float vi = 0; 
            float min_hor = occ_hor_thr2, min_ver = occ_ver_thr2;
            bool map_cons = true;    
            for (int ind_hor = -occ_hor_num2; ind_hor <= occ_hor_num2; ind_hor ++)
            {
                for (int ind_ver = -occ_ver_num2; ind_ver <= occ_ver_num2; ind_ver ++)
                {   
                    int pos_new = md_neighbor_position(p_spherical.hor_ind, p_spherical.ver_ind, ind_hor, ind_ver);       
                    if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
                    const vector<MotionPoint*> & points_in_pixel = depth_structure_window[first_i]->pixel_point_index[pos_new];                    
                    if (depth_structure_window[first_i]->depth_tensor.all_min_depth[pos_new] > p_spherical.vec(2))
                    {
                        continue;
                    }   
                    for (int k = 0; k < points_in_pixel.size() && map_cons; k++)
                    {
                        const MotionPoint*  p_occ = points_in_pixel[k];                   
                        if(RecedingIsOccluded(p_spherical, *p_occ) && RecedingDepthConsistencyCheck(*p_occ, *depth_structure_window[first_i]))
                        {                        
                            cur_occ_times = 1;
                            if(cur_occ_times >= occluded_times_thr2) break;
                            ti = (p_occ->time + p.time)/2;
                            vi = (p_spherical.vec(2) - p_occ->vec(2))/(p.time - p_occ->time);
                            p.occu_index[0] = depth_structure_window[first_i]->map_index;
                            p.occu_index[1] = pos_new;
                            p.occu_index[2] = k;
                            p.occ_vec = p_spherical.vec;
                            p.occu_times = cur_occ_times;
                            MotionPoint  p0 = p;
                            MotionPoint p1 = *points_in_pixel[k];                          
                            int i = depth_structure_window.size();
                            i = i - 2;
                            V3D t1, t2;
                            t1.setZero();
                            t2.setZero();
                            while(i >= 0)
                            {                              
                                if(p1.occu_index[0] == -1 || p1.occu_index[0] < depth_structure_window.front()->map_index)
                                {
                                    SphericalProjection(p1, depth_structure_window[i]->map_index, depth_structure_window[i]->R_world_lidar, depth_structure_window[i]->t_world_lidar, p1);
                                    if(RecedingSearchPointOccludingP(p1, *depth_structure_window[i]))
                                    {
                                        p1.occ_vec = p1.vec;
                                    }
                                    else
                                    {
                                        break;
                                    }                                   
                                }                                
                                i = p1.occu_index[0]-depth_structure_window.front()->map_index;
                                MotionPoint*  p2 = depth_structure_window[i]->pixel_point_index[p1.occu_index[1]][p1.occu_index[2]];                                   
                                SphericalProjection(p, depth_structure_window[i]->map_index, depth_structure_window[i]->R_world_lidar, depth_structure_window[i]->t_world_lidar, p);
                                if(RecedingMapConsistencyCheck(p, *depth_structure_window[i], receding_interp_en))
                                {
                                    map_cons = false;
                                    break;
                                }
                                float vc = (p1.occ_vec(2) - p2->vec(2))/(p1.time - p2->time);
                                double tc = (p2->time + p1.time)/2;        
                                if (RecedingIsOccluded(p, *p2) &&\
                                    RecedingDepthConsistencyCheck(*p2, *depth_structure_window[i]) && RecedingVelCheck(vi, vc, ti-tc) )
                                {                            
                                    cur_occ_times += 1;
                                    if(cur_occ_times >= occluded_times_thr2)
                                    {
                                        p.occu_times = cur_occ_times;
                                        return true;
                                    }
                                    t2 = p2->glob;
                                    p1 = *p2;
                                    vi = vc;
                                    ti = tc;
                                }
                                else
                                {
                                    break;
                                }
                                i--;
                            }                       
                        } 
                        if(cur_occ_times >= occluded_times_thr2) break;
                    }
                    if(cur_occ_times >= occluded_times_thr2) break;
                }
                if(cur_occ_times >= occluded_times_thr2) break;
            }
        }
    }
    if (cur_occ_times >= occluded_times_thr2) 
    {
        p.occu_times = cur_occ_times;
        return true;
    }
    return false;
}

bool  MovingObjectRemoval::RecedingEnter(MotionPoint & p, const DepthStructure &map_info)
{
    if(p.dyn != STATIC)
    {
        return false;
    }
    float max_depth = 0;
    float depth_thr2_final = max(cutoff_value, k_depth_max_thr2*(p.vec(2) - d_depth_max_thr2)) + occ_depth_thr2;
    if(map_info.pixel_point_index[p.position].size() > 0)
    {
        const MotionPoint* max_point = map_info.pixel_point_index[p.position][map_info.all_max_point_index[p.position]];
        max_depth = max_point->vec(2); 
        float delta_t = (p.time - max_point->time);
        depth_thr2_final = min(depth_thr2_final, v_min_thr2*delta_t);
    }
    if(p.vec(2) > max_depth + depth_thr2_final) 
    {
        receding_num ++;
        return true;
    }
    else
    {
        return false;
    }
}

bool  MovingObjectRemoval::RecedingMapConsistencyCheck(MotionPoint & p, const DepthStructure &map_info, bool interp)
{
    float cur_hor = map_cons_hor_thr2;
    float cur_ver = map_cons_ver_thr2;
    float cur_depth = max(cutoff_value, k_depth_max_thr2*(p.vec(2) - d_depth_max_thr2)) + map_cons_depth_thr2;
    for (int ind_hor = -map_cons_hor_num2; ind_hor <= map_cons_hor_num2; ind_hor ++)
    {
        for (int ind_ver = -map_cons_ver_num2; ind_ver <= map_cons_ver_num2; ind_ver ++)  
        {
            int pos_new = md_neighbor_position(p.hor_ind, p.ver_ind, ind_hor, ind_ver);
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
            const vector<MotionPoint*> & points_in_pixel = map_info.pixel_point_index[pos_new];                         
            if (map_info.depth_tensor.all_max_depth[pos_new] > p.vec(2) + cur_depth && \
                map_info.depth_tensor.all_min_depth[pos_new] < p.vec(2) - cur_depth)
            {
                continue;
            }   
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const MotionPoint* point = points_in_pixel[j];
                if (point->dyn == STATIC && \
                    fabs(p.time-point->time) > frame_dur && \
                    fabs(p.vec(2)-point->vec(2)) <  cur_depth && \
                    fabs(p.vec(0)-point->vec(0)) < map_cons_hor_thr2 && \
                    fabs(p.vec(1)-point->vec(1)) < map_cons_ver_thr2)
                {
                    return true;
                }               
            }         
        }
    }
    if(interp && (p.local(0) < self_x_b || p.local(0) > self_x_f || p.local(1) > self_y_l || p.local(1) < self_y_r) )
    {
        float cur_interp = interp_thr2*(depth_structure_window.back()->map_index - map_info.map_index + 1);
        float depth_all = DepthInterpolationAll(p, map_info.map_index, map_info.pixel_point_index);
        if( fabs(p.vec(2) - depth_all)  < cur_interp) 
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}

bool  MovingObjectRemoval::RecedingSearchPointOccludingP(MotionPoint & p, const DepthStructure &map_info)
{ 
    for (int ind_hor = -occ_hor_num2; ind_hor <= occ_hor_num2; ind_hor ++)
    {
        for (int ind_ver = -occ_ver_num2; ind_ver <= occ_ver_num2; ind_ver ++)
        {   
            int pos_new = md_neighbor_position(p.hor_ind, p.ver_ind, ind_hor, ind_ver);         
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
            const vector<MotionPoint*> & points_in_pixel = map_info.pixel_point_index[pos_new];            
            if (map_info.depth_tensor.all_min_depth[pos_new] > p.vec(2))
            {
                continue;
            }   
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const MotionPoint* p_cond = points_in_pixel[j];
                if (RecedingIsOccluded(p, *p_cond) && RecedingDepthConsistencyCheck(*p_cond, map_info)) 
                {
                    p.occu_index[0] = map_info.map_index;
                    p.occu_index[1] = pos_new;
                    p.occu_index[2] = j;
                    p.occ_vec = p.vec;
                    return true;
                }
            }        
        }
    }
    return false;
}

bool  MovingObjectRemoval::RecedingIsOccluded(const MotionPoint & p, const MotionPoint & p_occ)
{
    if((dataset == 0 && p_occ.is_distort) || (dataset == 0 && p.is_distort) || p_occ.dyn == INVALID) return false;
    if((p.local(0) > self_x_b && p.local(0) < self_x_f && p.local(1) < self_y_l && p.local(1) > self_y_r) || \
        (p_occ.local(0) > self_x_b && p_occ.local(0) < self_x_f && p_occ.local(1) < self_y_l && p_occ.local(1) > self_y_r))
    {
        return false;
    }
    float delta_t = p.time - p_occ.time;
    float cur_occ_hor = occ_hor_thr2; 
    float cur_occ_ver = occ_ver_thr2; 
    if(delta_t > 0)
    {
        float depth_thr2_final = min(max(cutoff_value, k_depth_max_thr2*(p.vec(2) - d_depth_max_thr2)) + occ_depth_thr2, v_min_thr2*delta_t);
        if (p.vec(2) >  p_occ.vec(2) + depth_thr2_final && \
            fabs(p.vec(0)-p_occ.vec(0)) < cur_occ_hor && \
            fabs(p.vec(1)-p_occ.vec(1)) < cur_occ_ver )
        {
            return true;
        }              
    }
    return false;
}

float MovingObjectRemoval::DepthInterpolationAll(MotionPoint & p, int map_index, const PixelPointIndex &pixel_point_index)
{

    V3F p_1 = V3F::Zero();
    V3F p_2 = V3F::Zero();
    V3F p_3 = V3F::Zero();
    vector<V3F> p_neighbors;
    int all_num = 0;
    for (int ind_hor = -interp_hor_num; ind_hor <= interp_hor_num; ind_hor ++)
    {
        for (int ind_ver = -interp_ver_num; ind_ver <= interp_ver_num; ind_ver ++)
        {   
            int pos_new = md_neighbor_position(p.hor_ind, p.ver_ind, ind_hor, ind_ver);          
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
            const vector<MotionPoint*> & points_in_pixel = pixel_point_index[pos_new];           
            for (int j = 0; j < points_in_pixel.size(); j ++)
            {
                const MotionPoint*  point = points_in_pixel[j]; 
                if (fabs(point->time - p.time) < frame_dur)
                {
                    continue;
                }
                float hor_minus =  point->vec(0) - p.vec(0);
                float ver_minus =  point->vec(1) - p.vec(1);
                if (fabs(hor_minus) < interp_hor_thr && fabs(ver_minus) < interp_ver_thr)
                {
                    all_num ++;
                    p_neighbors.push_back(point->vec);
                    if (p_1(2)<0.000001 || fabs(hor_minus) + fabs(ver_minus) < fabs(p_1(0) - p.vec(0)) + fabs(p_1(1) - p.vec(1)))
                    {
                        p_1 = point->vec;
                    }                   
                }
            }
        }
    }
    int cur_size = p_neighbors.size();
    if (p_1(2)<10E-5 || cur_size < 3)
    {
        return -1;
    }    
    for(int t_i = 0; t_i < cur_size-2; t_i++)
    {
        p_1 = p_neighbors[t_i];
        p_2 = V3F::Zero();
        p_3 = V3F::Zero();
        float min_fabs = 2*(interp_hor_thr + interp_ver_thr);
        float x = p.vec(0) - p_1(0);
        float y = p.vec(1) - p_1(1);
        float alpha  = 0, beta = 0;
        for(int i = t_i+1; i < cur_size-1; i++)
        {
            if(fabs(p_neighbors[i](0)-p.vec(0)) + fabs(p_neighbors[i](1)-p.vec(1)) < min_fabs)
            {
                p_2 = p_neighbors[i];
                float single_fabs = fabs(p_neighbors[i](0)-p.vec(0)) + fabs(p_neighbors[i](1)-p.vec(1));
                if (single_fabs >= min_fabs) continue;
                for(int ii = i+1; ii < cur_size; ii++)
                {
                    float cur_fabs = fabs(p_neighbors[i](0)-p.vec(0)) + fabs(p_neighbors[i](1)-p.vec(1)) + \
                                    fabs(p_neighbors[ii](0)-p.vec(0)) + fabs(p_neighbors[ii](1)-p.vec(1));
                    if( cur_fabs < min_fabs)
                    {
                        float x1 = p_neighbors[i](0) - p_1(0);
                        float x2 = p_neighbors[ii](0) - p_1(0);
                        float y1 = p_neighbors[i](1) - p_1(1);
                        float y2 = p_neighbors[ii](1) - p_1(1);
                        float lower = x1*y2-x2*y1;
                        if(fabs(lower) > 10E-5)
                        {
                            alpha = (x*y2-y*x2)/lower;
                            beta = -(x*y1-y*x1)/lower;
                            if(alpha > 0 && alpha < 1 && beta > 0 && beta < 1 && (alpha + beta) > 0 && (alpha + beta) < 1)
                            {
                                p_3 = p_neighbors[ii];
                                min_fabs = cur_fabs; 
                            }                        
                        }
                    }
                }
            } 
        }
        if (p_2(2)<10E-5 || p_3(2)<10E-5)
        {
            continue;
        }
        float depth_cal = (1-alpha-beta)*p_1(2) + alpha*p_2(2) + beta*p_3(2);
        return depth_cal;
    }
    return -2;
} // -1 denotes no points, -2 denotes no triangular > 1000 denotes gauss interpolation 

bool  MovingObjectRemoval::RecedingDepthConsistencyCheck(const MotionPoint & p, const DepthStructure &map_info)
{
    float all_minus = 0;
    int num = 0, smaller_num = 0, all_num = 0, greater_num = 0;
    for (int ind_hor = -depth_cons_hor_num2; ind_hor <= depth_cons_hor_num2; ind_hor ++)
    {
        for (int ind_ver = -depth_cons_ver_num2; ind_ver <= depth_cons_ver_num2; ind_ver ++)
        {   
            int pos_new = md_neighbor_position(p.hor_ind, p.ver_ind, ind_hor, ind_ver); 
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
            const vector<MotionPoint*> & points_in_pixel = map_info.pixel_point_index[pos_new];
            for (int j = 0; j < points_in_pixel.size(); j ++)
            {
                const MotionPoint* point = points_in_pixel[j]; 
                if(fabs(point->time - p.time) < frame_dur && fabs(point->vec(0)-p.vec(0)) < depth_cons_hor_thr2 && \
                  fabs(point->vec(1)-p.vec(1)) < depth_cons_ver_thr2)
                {
                    all_num ++;
                    if (point->dyn == STATIC) 
                    {
                        float cur_minus = p.vec(2)-point->vec(2);
                        if (fabs(cur_minus) < depth_cons_depth_max_thr2)
                        {
                            num ++;
                            all_minus += fabs(point->vec(2)-p.vec(2));
                        }
                        else if (cur_minus > 0)
                        {
                            smaller_num ++;
                        }
                        else
                        {
                            greater_num ++;
                        }
                    }
                }
            }
        }
    }
    if(all_num > 0)
    {
        if(num > 1)
        {           
            float cur_depth_thr = max(depth_cons_depth_thr2, k_depth2*p.vec(2));
            if(all_minus/(num-1) > cur_depth_thr)
            {
                return false;
            }      
        }
        if(greater_num == 0 || smaller_num == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }  
}

bool  MovingObjectRemoval::RecedingVelCheck(float v1, float v2, double delta_t)
{   
    if(fabs(v1 - v2) < delta_t*acc_thr2)
    {
        return true;
    }
    return false;
}

bool  MovingObjectRemoval::ApproachingVelCheck(float v1, float v2, double delta_t)
{
    if(fabs(v1 - v2) < delta_t*acc_thr3)
    {
        return true;
    }
    return false;
}

bool  MovingObjectRemoval::detectApproachingLegacy(MotionPoint & p)
{   
    if(dataset == 0 && p.is_distort) return false;
    int first_i = depth_structure_window.size();
    first_i -= 1;
    if(first_i < 0) return false;
    MotionPoint p_spherical = p;
    SphericalProjection(p, depth_structure_window[first_i]->map_index, depth_structure_window[first_i]->R_world_lidar, depth_structure_window[first_i]->t_world_lidar, p_spherical);  
    if (fabs(p_spherical.hor_ind) >= MAX_1D || fabs(p_spherical.ver_ind) >= MAX_1D_HALF || p_spherical.vec(2) < 0.0f || \
        p_spherical.position < 0 || p_spherical.position >= MAX_2D_N)
    {
        p.dyn = INVALID;
        return false;
    }
    int cur_occ_times = 0;
    if (ApproachingEnter(p_spherical, *depth_structure_window[first_i]))
    {
        if (!ApproachingMapConsistencyCheck(p_spherical, *depth_structure_window[first_i], approaching_interp_en))
        {
            double ti = 0;
            float vi = 0; 
            float min_hor = occ_hor_thr3, min_ver = occ_ver_thr3;
            bool map_cons = true;       
            for (int ind_hor = -occ_hor_num3; ind_hor <= occ_hor_num3; ind_hor ++)
            {
                for (int ind_ver = -occ_ver_num3; ind_ver <= occ_ver_num3; ind_ver ++)
                {   
                    int pos_new = md_neighbor_position(p_spherical.hor_ind, p_spherical.ver_ind, ind_hor, ind_ver);  
                    if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
                    const vector<MotionPoint*> & points_in_pixel = depth_structure_window[first_i]->pixel_point_index[pos_new];                   
                    if (depth_structure_window[first_i]->depth_tensor.all_max_depth[pos_new] < p_spherical.vec(2))
                    {
                        continue;
                    }   
                    for (int k = 0; k < points_in_pixel.size() && map_cons; k++)
                    {
                        const MotionPoint* p_occ = points_in_pixel[k];                          
                        if(ApproachingIsOccluding(p_spherical, *p_occ) && ApproachingDepthConsistencyCheck(*p_occ, *depth_structure_window[first_i]))
                        {
                            
                            cur_occ_times = 1;
                            ti = (p_occ->time + p.time)/2;
                            vi = (p_occ->vec(2) - p_spherical.vec(2))/(p.time - p_occ->time);
                            p.is_occu_index[0] = depth_structure_window[first_i]->map_index;
                            p.is_occu_index[1] = pos_new;
                            p.is_occu_index[2] = k;
                            p.is_occ_vec = p_spherical.vec;
                            p.is_occu_times = cur_occ_times;                        
                            MotionPoint  p0 = p;
                            MotionPoint p1 = *points_in_pixel[k];                        
                            int i = depth_structure_window.size();
                            i = i -2;
                            while(i >= 0)
                            {
                                if(p1.is_occu_index[0] == -1 || p1.is_occu_index[0] < depth_structure_window.front()->map_index)
                                {
                                    SphericalProjection(p1, depth_structure_window[i]->map_index, depth_structure_window[i]->R_world_lidar, depth_structure_window[i]->t_world_lidar, p1);
                                    if(ApproachingSearchPointOccludedbyP(p1, *depth_structure_window[i]))
                                    {
                                        p1.is_occ_vec = p1.vec;
                                    }
                                    else
                                    {
                                        break;
                                    }               
                                }
                                i = p1.is_occu_index[0]-depth_structure_window.front()->map_index;
                                MotionPoint* p2 = depth_structure_window[i]->pixel_point_index[p1.is_occu_index[1]][p1.is_occu_index[2]];                        
                                SphericalProjection(p, depth_structure_window[i]->map_index, depth_structure_window[i]->R_world_lidar, depth_structure_window[i]->t_world_lidar, p);
                                if(ApproachingMapConsistencyCheck(p, *depth_structure_window[i], approaching_interp_en))
                                {
                                    map_cons = false;
                                    break;
                                }
                                float vc = -(p1.is_occ_vec(2) - p2->vec(2))/(p1.time - p2->time);
                                double tc = (p2->time + p1.time)/2;                               
                                if (ApproachingIsOccluding(p, *p2) &&\
                                    ApproachingDepthConsistencyCheck(*p2, *depth_structure_window[i]) && ApproachingVelCheck(vi, vc, ti-tc) )
                                {                                    
                                    cur_occ_times += 1;
                                    if(cur_occ_times >= occluding_times_thr3)
                                    {
                                        p.is_occu_times = cur_occ_times;
                                        return true;
                                    }
                                    p1 = *p2;
                                    vi = vc;
                                    ti = tc;
                                }
                                else
                                {
                                    break;
                                }
                                i--;
                            }
                        } 
                        if(cur_occ_times >= occluding_times_thr3) break;
                    }
                    if(cur_occ_times >= occluding_times_thr3) break;
                }
                if(cur_occ_times >= occluding_times_thr3) break;
            }
        }
    }
    if (cur_occ_times >= occluding_times_thr3) 
    {
        p.is_occu_times = cur_occ_times;
        return true;
    }
    return false;
}

bool  MovingObjectRemoval::ApproachingEnter(MotionPoint & p, const DepthStructure &map_info)
{
    if(p.dyn != STATIC)
    {
        return false;
    }
    float min_depth = 0;
    float depth_thr3_final = max(cutoff_value, k_depth_max_thr3*(p.vec(2) - d_depth_max_thr3)) + occ_depth_thr3;
    if(map_info.pixel_point_index[p.position].size() > 0)
    {
        const MotionPoint* min_point = map_info.pixel_point_index[p.position][map_info.all_min_point_index[p.position]];
        min_depth = min_point->vec(2); 
        float delta_t = (p.time - min_point->time);
        depth_thr3_final = min(depth_thr3_final, v_min_thr3*delta_t);
    }
    if(dataset == 0 && p.is_distort)
    {
        depth_thr3_final = enlarge_distort*depth_thr3_final;
    }
    if(p.vec(2) < min_depth - depth_thr3_final)
    {
        approaching_num ++;
        return true;
    }
    else
    {
        return false;
    }
}

bool  MovingObjectRemoval::ApproachingMapConsistencyCheck(MotionPoint & p, const DepthStructure &map_info, bool interp)
{
    float cur_v_min = v_min_thr3;
    float cur_hor = map_cons_hor_thr3;
    float cur_ver = map_cons_ver_thr3;
    float cur_depth = max(cutoff_value, k_depth_max_thr3*(p.vec(2) - d_depth_max_thr3)) + map_cons_depth_thr3;
    if(dataset == 0 && p.is_distort) cur_v_min = enlarge_distort*cur_v_min;
    for (int ind_hor = -map_cons_hor_num3; ind_hor <= map_cons_hor_num3; ind_hor ++)
    {
        for (int ind_ver = -map_cons_ver_num3; ind_ver <= map_cons_ver_num3; ind_ver ++)      
        {
            int pos_new = md_neighbor_position(p.hor_ind, p.ver_ind, ind_hor, ind_ver);
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
            const vector<MotionPoint*> & points_in_pixel = map_info.pixel_point_index[pos_new];                        
            if (map_info.depth_tensor.all_max_depth[pos_new] > p.vec(2) + cur_depth && \
                map_info.depth_tensor.all_min_depth[pos_new] < p.vec(2) - cur_depth)
            {
                continue;
            }   
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const MotionPoint* point = points_in_pixel[j];
                if (point->dyn == STATIC && \
                    fabs(p.time-point->time) > frame_dur && \
                    (point->vec(2)-p.vec(2)) < cur_depth && \ 
                    fabs(p.vec(0)-point->vec(0)) < cur_hor && \
                    fabs(p.vec(1)-point->vec(1)) < cur_ver)
                {

                    return true;
                }               
            }         
        }
    }
    if(interp && (p.local(0) < self_x_b || p.local(0) > self_x_f || p.local(1) > self_y_l || p.local(1) < self_y_r))
    {
        float cur_interp = interp_thr3*(depth_structure_window.back()->map_index - map_info.map_index + 1);
        float depth_all = DepthInterpolationAll(p, map_info.map_index, map_info.pixel_point_index);
        if( fabs(p.vec(2) - depth_all)  < cur_interp) 
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}

bool  MovingObjectRemoval::ApproachingSearchPointOccludedbyP(MotionPoint & p, const DepthStructure &map_info)
{
    for (int ind_hor = -occ_hor_num3; ind_hor <= occ_hor_num3; ind_hor ++)
    {
        for (int ind_ver = -occ_ver_num3; ind_ver <= occ_ver_num3; ind_ver ++)
        {   
            int pos_new = md_neighbor_position(p.hor_ind, p.ver_ind, ind_hor, ind_ver);                
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
            const vector<MotionPoint*> & points_in_pixel = map_info.pixel_point_index[pos_new];            
            if (map_info.depth_tensor.all_min_depth[pos_new] > p.vec(2))
            {
                continue;
            }   
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const MotionPoint* p_cond = points_in_pixel[j];
                if (ApproachingIsOccluding(p, *p_cond) && ApproachingDepthConsistencyCheck(*p_cond, map_info) ) 
                {
                    p.is_occu_index[0] = map_info.map_index;
                    p.is_occu_index[1] = pos_new;
                    p.is_occu_index[2] = j;
                    p.occ_vec = p.vec;
                    return true;
                }
            }        
        }
    }
    return false;
}

bool  MovingObjectRemoval::ApproachingIsOccluding(const MotionPoint & p, const MotionPoint & p_occ)
{
    if((dataset == 0 && p_occ.is_distort) || (dataset == 0 && p.is_distort) || p_occ.dyn == INVALID) return false;
    if((p.local(0) > self_x_b && p.local(0) < self_x_f && p.local(1) < self_y_l && p.local(1) > self_y_r) || \
        (p_occ.local(0) > self_x_b && p_occ.local(0) < self_x_f && p_occ.local(1) < self_y_l && p_occ.local(1) > self_y_r))
    {
        return false;
    }
    float delta_t = p.time - p_occ.time;
    if(delta_t > 0)
    {
        float depth_thr3_final = min(max(cutoff_value, k_depth_max_thr3*(p.vec(2) - d_depth_max_thr3)) + map_cons_depth_thr3, v_min_thr3*delta_t);
        if(dataset == 0 && p.is_distort) depth_thr3_final = enlarge_distort*depth_thr3_final;
        if (p_occ.vec(2) > p.vec(2)  + depth_thr3_final && \
            fabs(p.vec(0)-p_occ.vec(0)) < occ_hor_thr3 && \
            fabs(p.vec(1)-p_occ.vec(1)) < occ_ver_thr3 )
        {
            return true;
        }            
    }
    return false;
}

bool  MovingObjectRemoval::ApproachingDepthConsistencyCheck(const MotionPoint & p, const DepthStructure &map_info)
{
    float all_minus = 0;
    int num = 0, smaller_num = 0, all_num = 0, greater_num = 0;//
    for (int ind_hor = -depth_cons_hor_num3; ind_hor <= depth_cons_hor_num3; ind_hor ++)
    {
        for (int ind_ver = -depth_cons_ver_num3; ind_ver <= depth_cons_ver_num3; ind_ver ++)
        {   
            int pos_new = md_neighbor_position(p.hor_ind, p.ver_ind, ind_hor, ind_ver);      
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
            const vector<MotionPoint*> & points_in_pixel = map_info.pixel_point_index[pos_new];
            for (int j = 0; j < points_in_pixel.size(); j ++)
            {
                const MotionPoint* point = points_in_pixel[j]; 
                if(fabs(point->time - p.time) < frame_dur && fabs(point->vec(0)-p.vec(0)) < depth_cons_hor_thr3 && \
                  fabs(point->vec(1)-p.vec(1)) < depth_cons_ver_thr3)
                {
                    all_num ++;
                    if (point->dyn == STATIC) 
                    {
                        float cur_minus = p.vec(2)-point->vec(2);
                        if (fabs(cur_minus) < depth_cons_depth_max_thr3)
                        {
                            num ++;
                            all_minus += fabs(point->vec(2)-p.vec(2));
                        }
                        else if (cur_minus > 0)
                        {
                            smaller_num ++;
                        }
                        else
                        {
                            greater_num ++;
                        }
                    }
                }
            }
        }
    }
    if(all_num > 0)
    {
        if(num > 1)
        {      
            float cur_depth_thr = max(depth_cons_depth_thr3, k_depth3*p.vec(2));
            if(all_minus/(num-1) > cur_depth_thr)
            {
                return false;
            }      
        }
        if(greater_num == 0 || smaller_num == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}

void MovingObjectRemoval::publish_dyn(const ros::Publisher & pub_point_out, const ros::Publisher & pub_frame_out, const ros::Publisher & pub_steady_points, const double & scan_end_time)
{
    if(cluster_coupled) // pubLaserCloudEffect pub_pcl_dyn_extend  pubLaserCloudEffect_depth
    {    
        cout<<"Found Dynamic Objects, numbers: " << laserCloudDynObj_clus->points.size() << " Total time: " << time_total << " Average total time: "<< time_total_avr << endl;
    }
    else
    {
        cout<<"Found Dynamic Objects, numbers: " << laserCloudDynObj->points.size() << " Total time: " << time_total << " Average total time: "<< time_total_avr << endl;
    }
    cout<<"case1 num: "<<entering_leaving_num<<" case2 num: "<<receding_num<<" case3 num: "<<approaching_num<<endl;
    entering_leaving_num = 0;
    receding_num = 0;
    approaching_num = 0;
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudDynObj_world, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(scan_end_time);
    laserCloudFullRes3.header.frame_id = frame_id;
    pub_point_out.publish(laserCloudFullRes3);
    if(cluster_coupled || cluster_future)
    {
        sensor_msgs::PointCloud2 laserCloudFullRes4;
        pcl::toROSMsg(*laserCloudDynObj_clus, laserCloudFullRes4);
        laserCloudFullRes4.header.stamp = ros::Time().fromSec(scan_end_time);
        laserCloudFullRes4.header.frame_id = frame_id;
        pub_frame_out.publish(laserCloudFullRes4);
    }
    sensor_msgs::PointCloud2 laserCloudFullRes2;
    PointCloudXYZI::Ptr laserCloudSteadObj_pub(new PointCloudXYZI);
    if(cluster_coupled)
    {
        if(laserCloudSteadObj_accu_times < laserCloudSteadObj_accu_limit)
        {
            laserCloudSteadObj_accu_times ++;
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj_clus);
            for(int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        else
        {   
            laserCloudSteadObj_accu.pop_front();
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj_clus);
            for(int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        pcl::VoxelGrid<PointType> downSizeFiltermap;
        downSizeFiltermap.setLeafSize(voxel_filter_size, voxel_filter_size, voxel_filter_size);
        downSizeFiltermap.setInputCloud(laserCloudSteadObj_pub);
        PointCloudXYZI laserCloudSteadObj_down;
        downSizeFiltermap.filter(laserCloudSteadObj_down);
        pcl::toROSMsg(laserCloudSteadObj_down, laserCloudFullRes2);
    }
    else
    {
        cout<<"Found Steady Objects, numbers: " << laserCloudSteadObj->points.size() << endl;
        if(laserCloudSteadObj_accu_times < laserCloudSteadObj_accu_limit)
        {
            laserCloudSteadObj_accu_times ++;
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj);
            for(int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        else
        {   
            laserCloudSteadObj_accu.pop_front();
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj);
            for(int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        pcl::toROSMsg(*laserCloudSteadObj_pub, laserCloudFullRes2);
    }
    laserCloudFullRes2.header.stamp = ros::Time().fromSec(scan_end_time);
    laserCloudFullRes2.header.frame_id = frame_id;
    pub_steady_points.publish(laserCloudFullRes2);
}

void MovingObjectRemoval::set_path(string file_path, string file_path_origin)
{
    is_set_path = true;
    out_file = file_path;
    out_file_origin = file_path_origin;
}


void MovingObjectRemoval::reset_history() {
    pending_points.clear();
    buffer.clear();
    depth_structure_window.clear();
    owned_frames.clear();
    owned_frame_bytes.clear();
    retained_point_bytes=0;
    map_index = 0;
    last_frame_time = -std::numeric_limits<double>::infinity();
}

void MovingObjectRemoval::commit_frame(const M3D& rot, const V3D& pos, double time) {
    if (pending_points.empty()) return;
    // Keep the classification, but clear projections computed from the first-pass pose.
    for (auto* p : pending_points) {
        p->glob = rot * p->local + pos;
        p->rot = rot.transpose();
        p->transl = pos;
        p->last_vecs.fill(V3F::Zero());
        p->last_positions.fill(Vector3i::Zero());
        p->last_depth_interps.fill(0.0f);
    }
    std::vector<int> indices(pending_points.size());
    std::iota(indices.begin(), indices.end(), 0);
    // Drain expired data before reserving slots for this frame.
    updateDepthStructureWindow(time);
    Points2Buffer(pending_points, indices);
    updateDepthStructureWindow(time);
    pending_points.clear();
    last_frame_time = time;
    double oldest = depth_structure_window.empty() ? time : depth_structure_window.front()->time;
    if (buffer.size()) oldest = std::min(oldest, buffer.front()->time);
    while (!owned_frames.empty() && owned_frames.front().first < oldest) {
        retained_point_bytes-=owned_frame_bytes.front();
        owned_frame_bytes.pop_front();
        owned_frames.pop_front();
    }
}

MotionLabel MovingObjectRemoval::classifyEnteringLeavingDirection(const MotionPoint& p) {
    int entering_votes=0, leaving_votes=0;
    for(const auto& map:depth_structure_window) {
        MotionPoint query=p;
        SphericalProjection(query,map->map_index,map->R_world_lidar,map->t_world_lidar,query);
        if(query.position<0 || query.position>=MAX_2D_N || query.ver_ind<0 || query.ver_ind>=MAX_1D_HALF) continue;
        const int pixel=query.position;
        if(map->depth_tensor.static_min_depth[pixel]<=0) continue;
        if(query.vec(2)<map->depth_tensor.static_min_depth[pixel]-depth_interval_margin) ++entering_votes;
        else if(query.vec(2)>map->depth_tensor.static_max_depth[pixel]+depth_interval_margin) ++leaving_votes;
    }
    // Legacy detection/cluster completion already established dynamic membership.
    // Ties or absent directional evidence retain ENTERING as a compatibility label.
    return leaving_votes>entering_votes ? LEAVING : ENTERING;
}

MotionLabel MovingObjectRemoval::detectMotion(MotionPoint& p) {
    int entering_votes=0, leaving_votes=0, approaching_votes=0, receding_votes=0;
    double approaching_velocity=minimum_radial_velocity, receding_velocity=minimum_radial_velocity;
    const int required=(temporal_window_size+1)/2;
    // A historical frame contributes at most one vote to a query point.
    // Traverse oldest to newest so the velocity bound propagates forward.
    for (const auto& map : depth_structure_window) {
        MotionPoint query=p;
        SphericalProjection(p,map->map_index,map->R_world_lidar,map->t_world_lidar,query);
        if(query.position<0 || query.position>=MAX_2D_N || query.ver_ind<0 || query.ver_ind>=MAX_1D_HALF) continue;
        int id=query.position;
        double range=query.vec(2), dt=p.time-map->time;
        if(map->depth_tensor.all_min_depth[id]<=0 || dt<=1e-6) continue;
        double pooled_static_min_depth=std::numeric_limits<double>::infinity(), pooled_static_max_depth=0;
        double pooled_all_min_depth=std::numeric_limits<double>::infinity(), pooled_all_max_depth=0;
        int u0=(query.hor_ind/2)*2, v0=(query.ver_ind/2)*2;
        for(int u=u0;u<std::min(u0+2,MAX_1D);++u)
            for(int v=v0;v<std::min(v0+2,MAX_1D_HALF);++v) {
                int pos=u*MAX_1D_HALF+v;
                if(map->depth_tensor.static_min_depth[pos]>0) {
                    pooled_static_min_depth=std::min(pooled_static_min_depth,double(map->depth_tensor.static_min_depth[pos]));
                    pooled_static_max_depth=std::max(pooled_static_max_depth,double(map->depth_tensor.static_max_depth[pos]));
                }
                if(map->depth_tensor.all_min_depth[pos]>0) {
                    pooled_all_min_depth=std::min(pooled_all_min_depth,double(map->depth_tensor.all_min_depth[pos]));
                    pooled_all_max_depth=std::max(pooled_all_max_depth,double(map->depth_tensor.all_max_depth[pos]));
                }
            }
        bool outside_pool=std::isfinite(pooled_static_min_depth) && (range<pooled_static_min_depth-depth_interval_margin || range>pooled_static_max_depth+depth_interval_margin);
        bool outside_static=map->depth_tensor.static_min_depth[id]>0 &&
            (range<map->depth_tensor.static_min_depth[id]-depth_interval_margin || range>map->depth_tensor.static_max_depth[id]+depth_interval_margin);
        // The paper jointly detects entering/leaving. Split labels by signed range deviation:
        // nearer than static background -> ENTERING; farther -> LEAVING.
        if(outside_static && outside_pool) {
            if(range<pooled_static_min_depth-depth_interval_margin) ++entering_votes;
            else if(range>pooled_static_max_depth+depth_interval_margin) ++leaving_votes;
        }
        bool approaching=range<map->depth_tensor.all_min_depth[id];
        bool receding=range>map->depth_tensor.all_max_depth[id];
        double boundary=approaching ? map->depth_tensor.all_min_depth[id] : map->depth_tensor.all_max_depth[id];
        double deviation=std::abs(range-boundary);
        bool within_all=std::isfinite(pooled_all_min_depth) && range>=pooled_all_min_depth && range<=pooled_all_max_depth;
        if(approaching && deviation>=approaching_velocity*dt && outside_pool && within_all) {
            ++approaching_votes;
            approaching_velocity=std::max(minimum_radial_velocity,deviation/dt);
        }
        if(receding && deviation>=receding_velocity*dt && outside_pool && within_all) {
            ++receding_votes;
            receding_velocity=std::max(minimum_radial_velocity,deviation/dt);
        }
    }
    // Keep entering/leaving precedence; opposite radial directions do not share votes.
    if(entering_votes>=required) return ENTERING;
    if(leaving_votes>=required) return LEAVING;
    if(approaching_votes>=required) return APPROACHING;
    if(receding_votes>=required) return RECEDING;
    return STATIC;
}
