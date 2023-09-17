#pragma once
#include <common/proto/convexhull.pb.h>
#include <common/proto/message_pad.pb.h>  //Modify-jiangnan
#include "autopilot_msgs/ros_proto.h"
#include "base/point_cloud.h"
#include "base/sensor_data_manager.h"  //Modify(@liuxinyu):
#include "common/visualisation.h"
#include "lib/global_postprocessor/global_post_processor.h"
#include "lib/object_filter/object_filter.h"
#include "obstacle_multi_sensor_fusion.h"
#include "perception/base/proto/fusion_component_config.pb.h"
#include "perception/base/proto/perception_component.pb.h"
#include "perception/base/sensor_manager/sensor_manager.h"
#include "perception/base/timer.h"
#include "rviz_display/rviz_display.h"
#include "common/include/pb_utils.h"
#include "common/proto/object.pb.h"
#include "common/proto/sensor_meta.pb.h"
#include "common/proto/param_set_cmd.pb.h"
#include "base/configmanager.h"
#include "base/v2x_param.h"
#include "lib/event_process/event_processor.h"
#include "common/proto/fm_info.pb.h"
#include "lib/map_server/map_server.h"

#include "common/object_convertor/lidar_convertor.h"
#include "common/object_convertor/radar_convertor.h"
#include "common/object_convertor/visual_convertor.h"
#include "common/object_convertor/output_convertor.h"
#include "common/object_convertor/obu_convertor.h"
#include "common/proto/ground_map.pb.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <algorithm>
#include <thread>
namespace perception {
namespace fusion {

struct GroundCoeff{
    int start_ring;
    int end_ring;
    Eigen::Vector4f ground_coe; //平面方程系数
    Eigen::Vector3f ground_normal; //平面法向量
};

class PerceptionFusionComponent {
 public:
  PerceptionFusionComponent() = default;
  ~PerceptionFusionComponent() {
    if (param_thread_ == nullptr) return;
    param_thread_->join();
  };

  bool Init();

 private:
  void SensorObjectsCallback(const std_msgs::StringConstPtr& msg);
  void Process(const fusion::FrameConstPtr& frame, std::vector<fusion::ObjectPtr>* objects);
  bool InitAlgorithmPlugin();
  void InitSubTopics(FusionComponentInitOptions& options);
  void InitPubTopics(FusionComponentInitOptions& options);
  void LidarObjectsCallback(const std_msgs::StringConstPtr& msg);
  void RadarObjectsCallback(const std_msgs::StringConstPtr& msg);
  void CameraObjectsCallback(const std_msgs::StringConstPtr& msg);
  void LocalizationCallback(const autopilot_msgs::BinaryDataConstPtr& msg);
  void HadmapLaneCallback(const autopilot_msgs::BinaryDataConstPtr& msg);
  void GroundFittingInfoCallback(const autopilot_msgs::BinaryDataConstPtr& msg);
  void PublishFusedObjects(const fusion::FrameConstPtr& frame,
                           const std::vector<fusion::ObjectPtr>& fused_objects);
  bool QueryNearestLocalization(const double& timestamp, localization::Localization& localization);
  void AddEgoCarFromLocalization(const localization::Localization localization,
                                 TrackedObjects& tracked_objects);
  // Modify(@liuxinyu): obu_test
  void ObuObjectsCallback(const std_msgs::StringConstPtr& msg);

  void VidarObjectsCallback(const std_msgs::StringConstPtr& msg);
  // Modify-jiangnan
  void FalconLidarObjectsCallback(const std_msgs::StringConstPtr& msg);

  //PreProcess Radar data
  bool PostProcessRadar(const RadarObject &radar_object, const fusion::ObjectPtr apollo_object);

  //PreProcessLidar
  bool PreProcessLidar(const TrackedObject *lidar_obj, const std::string &sensor_name,
                       const double radius_roi_object_check);

  // Modify-baihaijiang
  void MapConvexhullCallback(const autopilot_msgs::BinaryDataConstPtr& msg);
  bool updateObjToMapConvexhull(const double& timestamp, hadmap::MapConvexhull& map_convexhull);
  // transform from radar to vehicle frame temporarily (later moved to radar)- syf
  void TransformToBaseLink(perception::RadarObject& obj_in,
                           perception::RadarObject& obj_out,
                           const Eigen::Affine3d radar_T_base);
  bool IsPassThrough(ObuObject obu_object);
  void InitV2xParam();
  // @brief: parse v2n switching param
  // @author:liuxinyu
  void ParamSetCmdCallback(const autopilot_msgs::BinaryDataConstPtr &msg);
  bool GetV2XParamThread();
  void FilteredBySemanticMap(std::vector<fusion::ObjectPtr>& fused_objects);
  bool IsInVegPolygons(const fusion::ObjectPtr& obj_ptr);
  bool IsBuildingObj(const fusion::ObjectPtr& obj_ptr);
  void TranformPolygonMap2Baselink(map::V_V_Points& polygons, localization::Localization& localization);
  void Front60CameraDetectionCallback(const sensor_msgs::ImageConstPtr& msg);
  void calculateBoxDepthByLane(const localization::Localization& cur_local, fusion::FramePtr frame);
  void calculateCameraFov();
  void GetV2XParam();

 private:
  static std::mutex s_mutex_;
  static uint32_t s_seq_num_;

  std::string fusion_method_;
  // std::string fusion_main_sensor_;
  std::vector<std::string> fusion_main_sensors_;
  bool object_in_roi_check_ = true;
  double radius_for_roi_object_check_ = 80.0;
  double radius_faclon_for_roi_object_check_ = 160.0;
  double falcon_lidar_filter_distance_ = 40.0;

  std::unique_ptr<fusion::ObstacleMultiSensorFusion> fusion_;
  std::map<std::string,ros::Subscriber> subscribers_map_;
  std::map<std::string,ros::Publisher> publishers_map_;


  ProtoPublisher<TrackedObjects> proto_msg_publisher_;
  NoHeaderProtoPublisher<fault_management::FaultReportMsg> fault_msg_publisher_;
  NoHeaderProtoPublisher<mogo::telematics::pad::TrackedObjects>
      proto_msg_publisher_app;  // Modify-jiangnan

  bool enable_publish_planning_ = true;
  bool enable_publish_app_ = true;
  bool pub_vidar_ = false;
  bool pub_radar_ = false;
  bool pub_obu_ = false;
  ros::Timer pub_to_app_timer_;
  TrackedObjects output_objects_;
  fault_management::FaultReportMsg output_fault_messages_;

  std::list<localization::Localization> global_localizations_;
  GlobalPostProcessor global_postprocessor_;
  Visualization visualization_;

  perception::base::SensorInfo lidar_sensor_info_, camera_sensor_info_, radar_sensor_info_,
      vidar_sensor_info_, obu_sensor_info_, falcon_lidar_info_;

  // Modify(@liuxinyu): obu_test
  // perception::base::SensorInfo obu_sensor_info_;
  std::list<ObuObjects> obu_objects_v2n_;
  std::list<ObuObjects> obu_objects_v2i_;
  bool pub_v2n_to_pnc_ = false;
  bool pub_v2i_to_pnc_app_ = false;
  v2x_param v2x_param_;
  std::mutex pub_v2n_mutex_;
  std::mutex pub_v2i_mutex_;

  // Modify-baihaijiang
  std::list<hadmap::MapConvexhull> map_convexhulls_;
  ObjectFilter object_filter_;
  RvizDisplay::Ptr display_ptr;
  RvizDisplay::Ptr map_display_ptr_;
  RvizDisplay::Ptr lane_display_ptr_;
  RvizDisplay::Ptr fov_display_ptr_;
  EventProcessor event_processor_;

  std::string mid_fusion_obstacle_topic_;
  double radar_filter_yaw_thr_ = 0.025;
  bool release_mode_ = true;
  int vehicle_;
  ros::NodeHandlePtr node_ptr_;
  
  
  hadmap::MapMsg hadmap_msg_;
  ground_map::GroundMap ground_map_msg_;
  cv_bridge::CvImagePtr new_detection_image_;
  Eigen::Matrix4d baselink2camera60_pose_;
  Eigen::Matrix4d camera602baselink_pose_;
  base::BaseCameraDistortionModelPtr c60_model_;
  std::vector<GroundCoeff> ground_map_;
  bool use_ground_map_ = false;

  std::vector<perception::VirtualObject> virtual_objects;
  // obu_test
  std::vector<std::string> fusion_main_sensors_bk_;
  std::shared_ptr<std::thread> param_thread_; 
};
bool IsInCameraFov(PointCloud<PointD> polygon_ego);
}  // namespace fusion
}  // namespace perception
