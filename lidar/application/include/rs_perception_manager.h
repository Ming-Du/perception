#pragma once 

#include "common/include/msg/lidar_frame_msg.h"
#include "modules/ground_filter/include/rs_ground_filter.h"
#include "modules/segmentor/include/rs_segmentor.h"
#include "modules/denoise/include/denoise_grid.h"
#include "modules/preprocessor/include/rs_preprocessing.h"
#include "modules/roi_filter/include/roi_filter.h"
#include "modules/bev_builder/include/bev_builder.h"
#include "modules/tracker/include/tracking.h"
#include "modules/refiner/include/refine_manager.h"
#include "modules/rviz_display/include/rviz_display.h"
#include "modules/ai_detection/include/pointpillars/pointpillars_ai_detection.h"
#include "modules/util/debug_log_info/include/debug_show.h"
#include "modules/util/trigger/include/trigger_info_pub.h"
#include "common/proto/mogo_point_cloud.pb.h"
#include "common/proto/ground_map.pb.h"
#include "common/proto/trigger_info.pb.h"
#include <pcl/filters/voxel_grid.h>

#include <thread>
#include <condition_variable>

namespace robosense{

class HackAreaManager{
public:
    using Ptr = std::shared_ptr<HackAreaManager>;
    HackAreaManager(std::vector<LocalPointParam> &localpoints_input): localpoints(localpoints_input){}
    HackAreaType hackArea(const localization::Localization &local_current){
        for (int index=0 ; index < localpoints.size(); index++){
            double xtmp = local_current.position().x();
            double ytmp = local_current.position().y();
            if (sqrtf(pow(xtmp - localpoints[index].hack_area_x, 2) + pow(ytmp - localpoints[index].hack_area_y, 2)) < localpoints[index].hack_area_distance) {
                return HackAreaType(index+1);
            }
        } 
        return HackAreaType::NORMAL;
    };
private:
    std::vector<LocalPointParam> localpoints;

};

class LidarPerceptionManager {
public:
    using Ptr = std::shared_ptr<LidarPerceptionManager>;

    LidarPerceptionManager();
    ~LidarPerceptionManager();

    void init(const Rs_YAMLReader &configParse);
    void perception(const LidarFrameMsg::Ptr& msg_ptr,const localization::Localization& local_current, bool is_parser_map_info);
    void getBackgroundCloudpoints(const LidarFrameMsg::Ptr &msg_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr &back_cloud_ptr);
    void BackgroundDownsampling(pcl::PointCloud<pcl::PointXYZI>::Ptr &back_cloud_ptr, float downsampling);
    void toProtoMsg(rule_segement::MogoPointCloud &proto_msg, const pcl::PointCloud<pcl::PointXYZI>::Ptr & back_cloud_ptr);
    void toPointCloud(rule_segement::MogoPointCloud &proto_msg);
private:
    void initMsg(const LidarFrameMsg::Ptr &msg_ptr, const localization::Localization &local_current);
    void debugCopy2Rviz(const LidarFrameMsg::Ptr &msg_ptr);
    void displayThread();

private:
    LocalPointParam cone_point;
    LocalPointParam uphill_point;
    std::vector<LocalPointParam> localpoints_collect;

    LidarFrameMsg::Ptr msg_ptr_;

    DisplayModule en_display_module_;
   
    bool exit_flag_;
    std::mutex mutex_;
    std::condition_variable display_cv_;
    std::condition_variable processed_cv_;;

    std::vector<std::thread> thread_vec;
    std::shared_ptr<std::thread> display_th_;

    BevBuilder::Ptr bev_builder_ptr = nullptr;
    RoiFilter::Ptr roi_filter_ptr = nullptr;
    RsPreprocessing::Ptr preprocessing_ptr_ = nullptr;
    DenoiseGrid::Ptr denoise_grid_ptr_ = nullptr;

    PointPillarsCenterHeadAiDetection::Ptr ai_detection_ptr_ = nullptr;

    RsGroundFilter::Ptr ground_filter_ptr_ = nullptr;
    RsSegmentor::Ptr segmentor_ptr_ = nullptr;
    RefineManager::Ptr refine_ptr_ = nullptr;
    Tracking::Ptr tracking_ptr_ = nullptr;
    TriggerManager::Ptr trigger_manager_ptr_ = nullptr;
    HackAreaManager::Ptr hackarea_manager_ptr = nullptr;

    RvizDisplay::Ptr display_ptr = nullptr;
};

}   // namespace robosense
