#ifndef POINTS_PILLAR_ROS_H
#define POINTS_PILLAR_ROS_H

// headers in STL
#include <list>
#include <memory>
#include <vector>
// headers in ROS
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <thread>
// headers in local files
#include "common/config_parser.h"
#include "common/visualisation.h"
#include "perception/base/sensor_manager/sensor_manager.h"
#include "common/proto/object.pb.h"
#include "common/proto/hadmap.pb.h"
#include "autopilot_msgs/ros_proto.h"
#include "common/proto/fm_info.pb.h"
// headers in PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <opencv2/opencv.hpp>

#include "common/cluster_interface.h"
#include "common/component_cluster.h"
#include "common/dev_rotate_iou.h"
#include "common/mid_struct.h"
#include "common/subscriber/camera2d_subscriber.h"
#include "common/subscriber/cloud_subscriber.h"
#include "common/subscriber/lidarobject_subscriber.h"
#include "common/subscriber/radar_subscriber.h"
#include "common/tic_toc.h"
#include "frustum_rcnn/include/frustum_pointNets.h"
#include "common/proto/trigger_info.pb.h"
#include "common/trigger/trigger_info_pub.h"

#include "proto/fusion_mid_component.pb.h"
#include "perception/fusion_mid/tracker/mid_fusion_tracker/mid_fusion_tracker.h"
#include "perception/fusion_mid/common/data_converter.h"

namespace perception {
namespace mid_fusion {
  extern std::string recv_lidar_sensor_name_;
class FpnetProcessingUnit {
 public:
  FpnetProcessingUnit();
  /**
   * @brief Create ROS pub/sub obejct
   * @details Create/Initializing ros pub/sub object
   */
  void CreateROSPubSub();
  void modifyCameraLidarExtrinsicMatrix(int c, bool& isCal);
  void Run();

 private:
  bool Init();
  void LoadVehConfParam();
  bool InitType();
  bool ReadData();
  /*
   *@brief: Determine if the Lidar has data ？
   *@author:liuxinyu
   */
  bool HasData();
  bool ExtractValidData();
  void PointsProcess();
  void Checkdata();
  /*
   *@brief: 判断是否连续：检测最新结果是否等于上一帧结果上+1，若不等于则erase
   *@author:liuxinyu
   */
  void CheckContinuity();
  void FaultManager(std::string fault_id);

 protected:
  void ExtraCamInterExterParam(std::string sensor_name,
                             const perception::base::SensorInfo sensor_info, Eigen::Matrix4d& m);
  /* PointsPrecess */
  bool LidarOnImage(const cv::Mat& img, const perception::VisualObjects cameraObjects,
                    std::vector<FrustumLidarPoint>& lidarPoints,
                    std::multimap<int, pcl::PointXYZ>& camerObj_lidarPoint);
  // Identify the eight corners of the target
  void DebugVisualProcess();
  /* display */
  void DisplayLidarobject(perception::TrackedObjects& lidar_measurement);
  void DisplayImageFrustum(std::unordered_map<std::string, Eigen::MatrixXd>& world_points_map);
  /* FpointnetProcess */
  void FpointnetProcess();
  PreprocessFpntInput ExtractPcdAndBox();
  std::vector<perception::TrackedObject*> GetCandidatesList();
  bool IsNoiseCandidate(const perception::TrackedObject& lidar_object);
  bool IsVegetationCandidate(const perception::TrackedObject& lidar_object);
  void PublishFpntObjects(const std::vector<float>& pred_box3d_temp,
                          const PreprocessFpntInput& pre_res);
  void LocalizationCallback(const autopilot_msgs::BinaryDataConstPtr& msg);
  void HadmapLaneCallback(const autopilot_msgs::BinaryDataConstPtr &msg);
  bool QueryNearestLocalization(const double& timestamp, localization::Localization& localization);
  void Front60CameraDetectionCallback(const sensor_msgs::ImageConstPtr& msg);
  void ParserTriggerFromRes(perception::TrackedObjects& lidar_measurement, std::vector<std::string>& trigger_info);

 private:
  perception::mid_common::DevRotateIouPtr dev_rotate_iou_ptr_;
  std::unique_ptr<frcnn::FrustumRCNN> frcnn_obj_ptr_;
  // subscriber
  std::shared_ptr<perception::mid_fusion::CloudSubscriber> cloud_sub_ptr_;
  std::shared_ptr<perception::mid_fusion::RadarSubscriber> radar_sub_ptr_;
  std::shared_ptr<perception::mid_fusion::LidarobjectSubscriber> lidarobjects_sub_ptr_;
  std::unordered_map<std::string, std::shared_ptr<mid_fusion::Camera2dSubscriber>>
      camera2d_sub_ptr_map_;
  /* sub to sensor driver info */
  ros::Subscriber sub_points_;
  ros::Subscriber sub_camera_front60_;
  ros::Subscriber sub_camera_front_left120_;
  ros::Subscriber sub_camera_front_right120_;
  ros::Subscriber sub_camera_back120_;
  /* sub to sensor perception info */
  ros::Subscriber sub_camera_objects_front60_;
  ros::Subscriber sub_camera_objects_left120_;
  ros::Subscriber sub_camera_objects_right120_;
  ros::Subscriber sub_camera_objects_back120_;
  ros::Subscriber sub_lidar_objects_;
  ros::Subscriber sub_frontRadar_objects_;
  ros::Subscriber localization_subscriber_;
  ros::Subscriber hadmap_lane_subscriber;
  ros::Subscriber sub_camera_detection_front60_;
  /* pub */
  ros::Publisher marker_pub_;
  ros::Publisher marker_3dcamera_pub_;
  ros::Publisher visualization_lidar_obstacle_pub_;
  ros::Publisher visualization_lidar_obstacle_text_pub_;
  ros::Publisher visualization_lidar_obstacle_polygon_pub_;
  ros::Publisher visualization_camera_frustum_;
  ros::Publisher visualization_2D_frustum_points_;
  ros::Publisher pub_lidar_objects_;
  ros::Publisher pub_camera_objects_;
  ros::Publisher pub_rangeImg_;
  ros::Publisher pub_trigger_; // triger 
  ProtoPublisher<trigger_info::TriggerInfo> proto_trigger_publisher_; 
  ros::Publisher fault_manager_report_; // FM manager
  NoHeaderProtoPublisher<fault_management::FaultReportMsg> fault_msg_publisher_;
  image_transport::Publisher pub_cloudpointsImg_;
  std::unordered_map<int, image_transport::Publisher> pub_lidarImageIOUImg_map_;

 private:
  MidFusionTracker::Ptr tracker_;
  std::map<uint32_t, double> multi_frame_infer_obj_count_;
  std::map<uint32_t, double> last_infer_obj_count_;
  fault_management::FaultReportMsg output_fault_messages_;
  double min_lidar_camera_sync_time_thr_ = 0.01;
  TriggerManager::Ptr trigger_manager_ptr_;


 private:
  /* config param */
  perception::mid_common::ConfigParserPtr config_parser_ptr_;

  /* param */
  std::string config_file_str_;
  std::string yaml_file_str_;
  std::string shuffle_index_file_str_;
  bool is_pub_cloudpointsImg_ = 0;
  bool is_radar_lidar_visualization_pub_ = 0;
  bool is_addfpnet_visualization_pub_ = 0;
  bool is_pub_camera_frustum_ = 0;
  bool is_pub_frustum_points_ = false;
  bool is_fpnet_ = 1;
  float max_time_tr_ = 0.1;
  int margin_fpnet_;
  std::string rec_topic_name_front_camera_obstacle_front30_;
  std::string rec_topic_name_front_camera_obstacle_front60_;
  std::string rec_topic_name_front_camera_obstacle_front120_;
  std::string rec_topic_name_front_camera_obstacle_left120_;
  std::string rec_topic_name_front_camera_obstacle_right120_;
  std::string rec_topic_name_back_camera_obstacle_back120_;
  std::string rec_topic_name_lidar_cloud_;
  std::string rec_topic_name_lidar_obstacle_;
  std::string rec_topic_name_radar_obstacle_;
  std::string rec_topic_name_localization_;
  std::string rec_topic_name_front_camera_detection_60_;
  std::string vehicle_type_ = "jinlv_star";
  // vegetation and small object infer flags
  bool is_vegetation_infer_ = false;
  bool is_small_object_infer_ = false;
  // vegetation infer roi params
  float vegetation_roi_x_max_ = 40.0;
  float vegetation_roi_x_min_ = 0.0;
  float vegetation_roi_y_max_ = 1.0;
  float vegetation_roi_y_min_ = -4.0;
  float vegetation_threshold_ = 0.8;
  float max_vegetation_velocity_ = 15.0;  // km/h
  // noise 
  bool is_noise_infer_ = false;
  float fixed_noise_minimum_score_thr_ = 0.8;
  float noise_roi_x_max_ = 40.0;
  float noise_roi_x_min_ = 0.0;
  float noise_roi_y_max_ = 1.0;
  float noise_roi_y_min_ = -4.0;
  bool is_only_pub_cone_ = false;
  bool enable_trigger_ = false;
 private:
  /* data buff */
  std::deque<perception::TrackedObjects> lidarobjects_data_buff_;
  std::deque<perception::mid_fusion::CloudData> cloud_data_buff_;
  std::deque<perception::RadarObjects> radar_data_buff_;
  std::unordered_map<std::string, std::deque<perception::VisualObjects>> camera2d_data_buff_map_;
  std::deque<perception::TrackedObjects> past_frames_fpnet_obstacles_;
  // object detection image
  std::unordered_map<std::string, std::list<cv_bridge::CvImagePtr>> detection_image_buffer_map_;
  /* store valid data */
  perception::mid_fusion::Fusion_RawData fusion_rawdata_;  // pointNet lidarObs RadarObs
  std::unordered_map<std::string, perception::VisualObjects> map_current_camera2d_data_;
  /* PointsProcess related */
  perception::mid_fusion::ClusterInterfacePtr cluster_interface_ptr_;
  std::shared_ptr<std::vector<FrustumLidarPoint>> lidarPoints_ptr_;
  /* calib related */
  std::unordered_map<std::string, perception::base::SensorInfo> sensorInfoMap_;
  std::unordered_map<std::string, Eigen::Matrix4d> sensorMatrixMap_;
  std::unordered_map<std::string, Eigen::Matrix4d> intrinsics_camera_map_;
  std::unordered_map<std::string, Eigen::Matrix4d> extrinsics_camera_map_;
  std::unordered_map<std::string, Eigen::VectorXd> distcoeff_camera_map_;
  std::unordered_map<std::string, std::string> inextritopic_totopic_map_;
  std::unordered_map<std::string, int> id_map_;
  /* camera intr and extr related */
  int raw_camera_num_ = 6;
  std::vector<float> cam_extr_;
  std::vector<float> cam_intr_;
  std::vector<int32_t> cam_hw_;
  std::vector<float> cam_extr_inv_;
  std::vector<float> cam_intr_inv_;
  /* record history velocity infos */
  std::unordered_map<int, int> object_id_to_velocity_cnt_;
  /* display */
  perception::mid_fusion::Visualization mid_visualization_;
  // initializer list
  ros::NodeHandle private_nh_;
  ros::NodeHandle nh_;
  bool baselink_support_;
  image_transport::ImageTransport imageTransport_;
  /* rosparam */
  std::string config_file_;
  std::string models_file_;

  std::list<localization::Localization> global_localizations_;

  private:
  perception::mid_fusion::LaneMarkersDataRecords lanemarks_;
};
}  // namespace mid_fusion
}  // namespace perception
#endif  // POINTS_PILLAR_ROS_H
