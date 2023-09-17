#ifndef POINTS_PILLAR_ROS_H
#define POINTS_PILLAR_ROS_H

#include <list>
#include <memory>
#include <vector>
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
#include <pcl/io/pcd_io.h>
#include <thread>
#include <pcl/PCLPointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <opencv2/opencv.hpp>

#include <visualization_msgs/Marker.h>
#include "common/config_parser.h"
#include "common/visualisation.h"
#include "perception/base/sensor_manager/sensor_manager.h"
#include "common/proto/object.pb.h"

extern std::unordered_map<std::string, std::list<cv_bridge::CvImagePtr>> imgBuffer_map_tmp_;

enum camera_postion {
  FRONT_FOV_60 = 1,
  FRONT_FOV_30,
  FRONT_FOV_120,
};

struct MyRect {
  cv::Rect camera_rect_;
  int id;
  MyRect(int a, int b, int c, int d) {
    cv::Rect temprect(a, b, c, d);
    camera_rect_ = temprect;
    id = -1;
  }
};
struct FrustumLidarPoint {  // single lidar point in space
  unsigned int id;
  camera_postion cp;  // front FOV_30, FOV_60. FOV_120, left FOV_120. right
                      // FOV_120, rear FOV_120
  double point3D_x, point3D_y, point3D_z,
      point3D_r;  // x,y,z in [m], r is point reflectivity
  bool is_on_img = false;
  double point2D_x, point2D_y;  // point 2D on image
  int camera_object_id = -1;
  float frustum_angle = 0.0;
  int only_camera_count = 0;
};

struct MYPoint {
  pcl::PointXYZ my_point;
  int only_camera_count = 0;
  int camera_object_id = -1;
};

class CalibCheckROS {
 public:
  CalibCheckROS();
  void CreateROSPubSub();
  void ModifyCameraLidarExtrinsicMatrix(int c, bool& isCal);

 private:
  bool Init();
  void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& input);
  bool LidarOnImage(const cv::Mat& img, std::vector<FrustumLidarPoint>& lidarPoints,
                  std::multimap<int, pcl::PointXYZ>& camerObj_lidarPoint);
  void MulitplyKandExtrinsic(std::string sensor_name,
                             const perception::base::SensorInfo sensor_info, Eigen::Matrix4d& camera2world_pose);
  void DistortImage(cv::Mat& undistort_img, cv::Mat& distort_img);
  void DistortPoint(std::vector<cv::Point2f>& src, std::vector<cv::Point2f>& dst);

  bool LoadCalibConfig(std::string config_file);
  void LoadCalibrationData();
  void SaveCalibrationMatrix(std::string config_file);
  void UpdateExtrinsicMatrix();
  void CameraObjectsCallback(const std_msgs::StringConstPtr& msg);
  void UndisortBox(const perception::VisualObject& camera_object, Eigen::Matrix4d& Intrinsics, Eigen::VectorXd& Distcoeff, perception::VisualObject* camera_object_undisort);
  Eigen::Vector2d GetUndisoredCoord(double u, double v, Eigen::Matrix4d& Intrinsics, Eigen::VectorXd& Distcoeff);
  bool Camera2dBoxOnImage(const cv::Mat& img, const perception::VisualObjects& ori_camera, const perception::VisualObjects& undisort_camera);


 private:
  friend class TestClass;
  calib_common::ConfigParserPtr config_parser_ptr_;           //配置文件获取
  // 参数
  std::string config_file_;
  std::string config_file_str_;
  std::string calib_camera_name_;     //要进行手动调整外参的相机名称
  int load_multable_calib_flag_ = 0;  //加载手动调整的相机外参
  bool is_pub_frustumImg_ = 0;
  bool is_pub_camera_frustum_ = 0;
  bool is_dedistortion_ = false;
  std::string rec_topic_name_lidar_cloud_;
  std::string rec_topic_name_front_camera_obstacle_front60_;
  float car_boundary_x_min_ = 1;
  float car_boundary_x_max_ = 60;
  float car_boundary_y_min_ = -10;
  float car_boundary_y_max_ = 10;
  double project_point_x_min_= 1.0;
  double project_point_x_max_= 60.0;
  double project_point_y_min_ = -4.0;
  double project_point_y_max_ = 4.0;
  double leaf_size_ = 0.15;
  float max_time_tr_ = 0.1;
  float image_pointcloud_diff_ = 0.1;
  bool baselink_support_;
  std::string check_lidar_;
  // sub&pub
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber sub_points_;
  ros::Subscriber sub_camera2d_;
  ros::Publisher pub_frustumPoints_;
  image_transport::Publisher pub_2dbox_image_;
  image_transport::ImageTransport imageTransport_;
  image_transport::Publisher pub_frustumImg_;
  image_transport::Publisher pub_frustumImg_nvjpeg_;
  tf::TransformListener tf_listener_;
  tf::StampedTransform baselink2lidar_;
  tf::Transform angle_transform_;
  tf::Transform angle_transform_inversed_;
  // 激光&相机内外参
  std::unordered_map<std::string, perception::base::SensorInfo> sensorInfoMap_;
  std::unordered_map<std::string, Eigen::Matrix4d> sensorMatrixMap_;
  std::unordered_map<std::string, Eigen::Matrix4d> intrinsics_camera_map_;
  std::unordered_map<std::string, Eigen::Matrix4d> extrinsics_camera_map_;
  std::unordered_map<std::string, Eigen::VectorXd> distcoeff_camera_map_;
  std::unordered_map<std::string, std::string> sensorname2topic_map_;
  perception::base::SensorInfo lidar_sensor_info_;
  perception::base::SensorInfo camera_sensor_info_;
  Eigen::Matrix4d T_camera2IMU_;
  Eigen::Matrix4d T_IMU2camera_;
  Eigen::Matrix4d T_lidar2IMU_;
  Eigen::Matrix4d T_CameraIntr_;
  //
  double rx_, ry_, rz_, tx_, ty_, tz_;
  //是否要手动调整相机内参，默认不进行手动调整，修改在配置文件“mid_fusion_config.txt”中
 public:
  int isCalib = 0;
  std::deque<perception::VisualObjects> camera2d_data_deque_;
};

#endif  // POINTS_PILLAR_ROS_H
