#pragma once

#include <unordered_map>

#include "common/proto/object.pb.h"
#include "common/visualisation.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/PCLPointCloud2.h>

#include <std_msgs/String.h>
#include <image_transport/subscriber_filter.h>
#include "perception/fusion_mid/common/mid_struct.h"
#include "perception/fusion_mid/common/sensor_data/cloud_data.h"

namespace perception {
namespace mid_fusion {
void PubSingleFeature(visualization_msgs::Marker& marker,
                      uint32_t shape,
                      double postion_x,
                      double postion_y,
                      double postion_z);
void DisplayObject(perception::TrackedObjects& objects,
                   Visualization& mid_visualization,
                   ros::Publisher& box_publish,
                   ros::Publisher& text_publish,
                   ros::Publisher& polygon_publish);
bool ProjectLidarPointsToImage(
    const cv::Mat& img,
    const perception::VisualObjects& camera_objects,
    const std::unordered_map<std::string, Eigen::Matrix4d>& intrinsics_camera_map,
    const std::unordered_map<std::string, Eigen::Matrix4d>& extrinsics_camera_map,
    const std::vector<FrustumLidarPoint>& lidar_points);
void DebugLidarCameraSync(
    const std::unordered_map<std::string, std::list<cv_bridge::CvImagePtr>>&
        detection_image_buffer_map,
    const std::unordered_map<std::string, Eigen::Matrix4d>& intrinsics_camera_map,
    const std::unordered_map<std::string, Eigen::Matrix4d>& extrinsics_camera_map,
    const perception::VisualObjects& camera_objects,
    const CloudData& cloud_data,
    image_transport::Publisher& cloud_points_image_pub);
void GetImageFrustum(const std::vector<perception::VisualObjects>& camera_frame_vec,
                     const std::unordered_map<std::string, Eigen::Matrix4d>& intrinsics_camera_map,
                     const std::unordered_map<std::string, Eigen::Matrix4d>& extrinsics_camera_map,
                     std::unordered_map<std::string, Eigen::MatrixXd>& world_points_map);
void DisplayLidarImageIOU(
    const std::vector<perception::VisualObjects>& camera_frame_vec,
    std::unordered_map<int, std::vector<CameraIDRect>>& camera_rect_map,
    std::unordered_map<int, image_transport::Publisher>& pub_lidar_image_iou_map);
}  // namespace mid_fusion
}  // namespace perception
