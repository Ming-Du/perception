#include "common/include/tic_toc.h"
#include "rs_perception.h"
#include <common/proto/localization.pb.h>

#include "mogo_track/gpu_track.h"
#include "mogo_track/mogo_track_logger.h"

namespace robosense {

LidarPerceptionManager::LidarPerceptionManager()
    : msg_ptr_(nullptr)
    , exit_flag_(false)
    , en_display_module_(DisplayModule::INVALID) 
{
    display_th_ = std::make_shared<std::thread>(std::bind(&LidarPerceptionManager::displayThread, this));
}

LidarPerceptionManager::~LidarPerceptionManager() {
    if (display_th_ == nullptr)
        return;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        exit_flag_ = true;
    }
    display_cv_.notify_one();
    display_th_->join();
}

void LidarPerceptionManager::init(const Rs_YAMLReader &configParse) {
    cone_point = configParse.getValue<LocalPointParam>("common.cone_filter");
    uphill_point = configParse.getValue<LocalPointParam>("common.uphill_filter");

    localpoints_collect.push_back(cone_point);
    localpoints_collect.push_back(uphill_point);
    hackarea_manager_ptr.reset(new HackAreaManager(localpoints_collect));

    // init preprocessing module
    ROS_INFO("Start init Preprocessing...");
    preprocessing_ptr_.reset(new RsPreprocessing);
    preprocessing_ptr_->init(configParse);

    // init bev builder module
    ROS_INFO("Start init BevBuilder...");
    bev_builder_ptr.reset(new BevBuilder);
    bev_builder_ptr->init(configParse);

    // init roi filter module
    ROS_INFO("Start init RoiFilter...");
    roi_filter_ptr.reset(new RoiFilter);
    roi_filter_ptr->init(configParse);

    // init denoise module
    ROS_INFO("Start init Denoise...");
    denoise_grid_ptr_.reset(new DenoiseGrid);
    denoise_grid_ptr_->init(configParse);

    // init ai detection module
    ROS_INFO("Start init AiDetection...");
    ai_detection_ptr_.reset(new PointPillarsCenterHeadAiDetection);
    ai_detection_ptr_->init(configParse);

    // init ground filter module
    ROS_INFO("Start init GroundFilter...");
    ground_filter_ptr_.reset(new RsGroundFilter);
    ground_filter_ptr_->init(configParse);

    // init segmentor module
    ROS_INFO("Start init Segmentor...");
    segmentor_ptr_.reset(new RsSegmentor);
    segmentor_ptr_->init(configParse);

    // init refine module
    ROS_INFO("Start init Refine...");
    refine_ptr_.reset(new RefineManager);
    refine_ptr_->init(configParse);

    // init tracking module
    ROS_INFO("Start init Tracker...");
    tracking_ptr_.reset(new Tracking);
    tracking_ptr_->init(configParse);
    
    trigger_manager_ptr_.reset(new TriggerManager);
    // init debug_log_info
    ROS_INFO("Start init RvizDisplay...");
    DebugShow::debug_show_ptr_->init(configParse);

    display_ptr.reset(new RvizDisplay(configParse));
    ROS_INFO("Init finshed.");
}

void LidarPerceptionManager::initMsg(const LidarFrameMsg::Ptr &msg_ptr, const localization::Localization &local_current) {
    msg_ptr->axis_pose_map.at(AxisType::GLOBAL_AXIS)->x = local_current.position().x();
    msg_ptr->axis_pose_map.at(AxisType::GLOBAL_AXIS)->y = local_current.position().y();
    msg_ptr->axis_pose_map.at(AxisType::GLOBAL_AXIS)->z = local_current.position().z();
    msg_ptr->axis_pose_map.at(AxisType::GLOBAL_AXIS)->yaw = local_current.yaw();
    msg_ptr->axis_pose_map.at(AxisType::GLOBAL_AXIS)->pitch = local_current.pitch();
    msg_ptr->axis_pose_map.at(AxisType::GLOBAL_AXIS)->roll = local_current.roll();

    {
        Pose::Ptr tmp_pose(new Pose);
        tmp_pose->x = 0;
        tmp_pose->y = 0;
        tmp_pose->z = 0;
        tmp_pose->roll = 0;
        tmp_pose->pitch = 0;
        tmp_pose->yaw = 0;
        *msg_ptr->axis_pose_map.at(AxisType::LIDAR_AXIS) = *tmp_pose;
    }
    {
        Pose::Ptr tmp_pose(new Pose);
        tmp_pose->x = 0;
        tmp_pose->y = 0;
        tmp_pose->z = 0;
        tmp_pose->roll = 0;
        tmp_pose->pitch = 0;
        tmp_pose->yaw = 0;
        *msg_ptr->axis_pose_map.at(AxisType::VEHICLE_AXIS) = *tmp_pose;
    }
    msg_ptr->status = AxisType::VEHICLE_AXIS;
}

void LidarPerceptionManager::perception(const LidarFrameMsg::Ptr &msg_ptr, const localization::Localization &local_current, bool is_parser_map_info) {
    TicToc stimer("perception");
    msg_ptr_ = msg_ptr;

    if (roi_filter_ptr != nullptr) {
        roi_filter_ptr->updataLocalization(local_current, is_parser_map_info);
    }

    {
        TicToc timer("perception/initMsg");
        initMsg(msg_ptr, local_current);
    }

    if (preprocessing_ptr_ != nullptr) {
        TicToc timer("perception/preprocessing");
        preprocessing_ptr_->perception(msg_ptr);

        en_display_module_ = DisplayModule::PREPROCESS;
        display_cv_.notify_one();
    }

    thread_vec.clear();
    if (ai_detection_ptr_ != nullptr) { // thread_vec[0]
        thread_vec.emplace_back(std::thread([&]() {
            TicToc timer("perception/AI detector");
            ai_detection_ptr_->perception(msg_ptr);

        }));
    }

    if (bev_builder_ptr != nullptr) {
        TicToc timer("perception/bev builder");
        bev_builder_ptr->perception(msg_ptr);
    }

    if (roi_filter_ptr != nullptr) {
        TicToc timer("perception/roi filter");
        roi_filter_ptr->perception(msg_ptr);

        en_display_module_ = DisplayModule::ROI_FILTER;
        display_cv_.notify_one();
    }

    if (ground_filter_ptr_ != nullptr) { // thread_vec[1]
        thread_vec.emplace_back(std::thread([&]() {
            TicToc timer("perception/ground filter");
            ground_filter_ptr_->perception(msg_ptr);
        }));
    }
    if (denoise_grid_ptr_ != nullptr) { // thread_vec[2]
        thread_vec.emplace_back(std::thread([&]() {
            if(denoise_grid_ptr_->denoise_enable_){
                TicToc timer("perception/denoise");
                denoise_grid_ptr_->perception(msg_ptr);
            }
        }));
    }
    thread_vec[1].join(); // ground filter
    en_display_module_ = DisplayModule::GROUND_FILTER;
    display_cv_.notify_one();

    if (segmentor_ptr_ != nullptr) { // thread_vec[3]
        thread_vec.emplace_back(std::thread([&]() {
            TicToc timer("perception/segmentor");
            segmentor_ptr_->perception(msg_ptr, local_current);
        }));
    }
    thread_vec[0].join(); // AI

    thread_vec[3].join(); // Segmentor
    en_display_module_ = DisplayModule::SEGMENTOR;
    display_cv_.notify_one();

    thread_vec[2].join(); // Denoise
    en_display_module_ = DisplayModule::DENOISE;
    display_cv_.notify_one();

    debugCopy2Rviz(msg_ptr);

    // refine
    // TODO: parall
    if (refine_ptr_ != nullptr && tracking_ptr_ != nullptr) {
        tracking_ptr_->predict(msg_ptr);
        TicToc timer("perception/refiner");
        refine_ptr_->perception(msg_ptr);
        
        en_display_module_ = DisplayModule::REFINER;
        display_cv_.notify_one();
    }


   
    if (trigger_manager_ptr_ != nullptr) { // thread_vec[4]
        thread_vec.emplace_back(std::thread([&]() {
            TicToc timer("perception/trigger");
            trigger_manager_ptr_->perception(msg_ptr);
        }));
    }
  

    // tracking
    if (tracking_ptr_ != nullptr) {
        TicToc timer("perception/tracker");
        tracking_ptr_->perception(msg_ptr);
    }
    
    en_display_module_ = DisplayModule::TRACKER;
    display_cv_.notify_one();

    thread_vec[4].join(); // trigger

    std::unique_lock<std::mutex> lock(mutex_);
    processed_cv_.wait_for(lock, std::chrono::milliseconds(3));
}

void LidarPerceptionManager::displayThread() {
    while (ros::ok()) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            display_cv_.wait(lock);
        }
        if (exit_flag_)
            break;

        display_ptr->display(msg_ptr_, en_display_module_);

        if (en_display_module_ == DisplayModule::TRACKER)
            processed_cv_.notify_one();
    }
}

void LidarPerceptionManager::debugCopy2Rviz(const LidarFrameMsg::Ptr &msg_ptr) {
    TRY_CATCH
    {
        auto &obj_ai_debug = msg_ptr->objects_ai_debug;
        for (auto &obj_ai : msg_ptr->objects_ai) {
            Object::Ptr tmp_obj(new Object);
            tmp_obj->clone(*obj_ai);
            obj_ai_debug.push_back(tmp_obj);
        }

        auto &obj_rule_debug = msg_ptr->objects_rule_debug;
        for (auto &obj_rule : msg_ptr->objects_rule) {
            Object::Ptr tmp_obj(new Object);
            tmp_obj->clone(*obj_rule);
            obj_rule_debug.push_back(tmp_obj);
        }
    }
    END_TRY_CATCH
}

void LidarPerceptionManager::getBackgroundCloudpoints(const LidarFrameMsg::Ptr &msg_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr &back_cloud_ptr) {
    const auto &scan_ptr = msg_ptr->scan_ptr;
    const auto &background_indice = msg_ptr->out_map_indices;
    TRY_CATCH
    {
        back_cloud_ptr->header.frame_id = "base_link";
        back_cloud_ptr->points.reserve(background_indice.size());
        for (size_t i = 0; i < background_indice.size(); ++i) {
            back_cloud_ptr->points.emplace_back(scan_ptr->points.at(background_indice.at(i)));
        }
    }
    END_TRY_CATCH
}

void LidarPerceptionManager::BackgroundDownsampling(pcl::PointCloud<pcl::PointXYZI>::Ptr &back_cloud_ptr, float downsampling) {
    pcl::PCLPointCloud2::Ptr cloud2_ptr(new pcl::PCLPointCloud2());
    toPCLPointCloud2(*back_cloud_ptr, *cloud2_ptr);
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud2_ptr);
    sor.setLeafSize(downsampling, downsampling, downsampling);
    sor.filter(*cloud_filtered);
    fromPCLPointCloud2(*cloud_filtered, *back_cloud_ptr);
}

void LidarPerceptionManager::toProtoMsg(rule_segement::MogoPointCloud &proto_msg, const pcl::PointCloud<pcl::PointXYZI>::Ptr &back_cloud_ptr) {
    int x_, y_, z_;
    int new_size_i = back_cloud_ptr->points.size();
    for (int i = 0; i < new_size_i; i++) {
        const auto &point = back_cloud_ptr->points[i];
        if (point.x > 60 || point.x < -30 || abs(point.y) > 30)
            continue;
        x_ = point.x * 100; // unit:cm
        y_ = point.y * 100;
        z_ = point.z * 100;
        proto_msg.add_add_data(x_);
        proto_msg.add_add_data(y_);
        proto_msg.add_add_data(z_);
        proto_msg.add_add_data(point.intensity);
    }
}

void LidarPerceptionManager::toPointCloud(rule_segement::MogoPointCloud &proto_msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pc_ptr->header.frame_id = "base_link";

    int new_size_i = proto_msg.add_data_size();
    // std::cout << "...size... " << new_size_i << std::endl;
    for (int i = 0; i < new_size_i; i += 4) {
        pcl::PointXYZI point;
        point.x = proto_msg.add_data(i) / 100.0;
        point.y = proto_msg.add_data(i + 1) / 100.0;
        point.z = proto_msg.add_data(i + 2) / 100.0;
        point.intensity = proto_msg.add_data(i + 3);
        pc_ptr->points.emplace_back(point);
    }
}

} // namespace robosense
