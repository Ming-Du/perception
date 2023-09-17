#include "perception/fusion_mid/common/display.h"

#include <chrono>

#include <ros/ros.h>
#include "perception/fusion_mid/common/define.h"


namespace perception {
namespace mid_fusion {

void PubSingleFeature(visualization_msgs::Marker& marker,
                      uint32_t shape,
                      double postion_x,
                      double postion_y,
                      double postion_z) {
  // Set our initial shape type to be a cube
  // Cycle between different shapes
  static int id = 1000000;
  switch (shape) {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::CUBE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
  }

  // Set the frame ID and timestamp.  See the TF tutorials for information on
  // these.
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique
  // ID Any marker sent with the same namespace and id will overwrite the old
  // one
  marker.ns = "basic_shapes";
  marker.id = id;

  // Set the marker type.  Initially this is CUBE, and cycles between that and
  // SPHERE, ARROW, and CYLINDER
  marker.type = shape;
  marker.points.clear();

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3
  // (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the
  // frame/time specified in the header
  marker.pose.position.x = postion_x;
  marker.pose.position.y = postion_y;
  marker.pose.position.z = postion_z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 2;
  marker.scale.y = 2;
  marker.scale.z = 2;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1;
  if (shape == visualization_msgs::Marker::SPHERE) {
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1;
  }
  marker.lifetime = ros::Duration(0.2);

  id++;
}

void DisplayObject(perception::TrackedObjects& objects,
                   perception::mid_fusion::Visualization& mid_visualization,
                   ros::Publisher& box_publish,
                   ros::Publisher& text_publish,
                   ros::Publisher& polygon_publish) {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::MarkerArray marker_array_text;
  visualization_msgs::MarkerArray marker_array_polygon;
  mid_visualization.BboxDisplay(objects, marker_array);
  mid_visualization.TextDisplay(objects, marker_array_text);
  mid_visualization.PolygonDisplay(objects, marker_array_polygon);
  if (!marker_array.markers.empty()) {
    box_publish.publish(marker_array);
    text_publish.publish(marker_array_text);
    polygon_publish.publish(marker_array_polygon);
  }
}

bool ProjectLidarPointsToImage(
    const cv::Mat& img,
    const perception::VisualObjects& camera_objects,
    const std::unordered_map<std::string, Eigen::Matrix4d>& intrinsics_camera_map,
    const std::unordered_map<std::string, Eigen::Matrix4d>& extrinsics_camera_map,
    const std::vector<FrustumLidarPoint>& lidar_points) {
#ifdef TEST_TIME
  std::chrono::time_point<std::chrono::system_clock> start, end;
  start = std::chrono::system_clock::now();
#endif
  if (lidar_points.empty() || camera_objects.objs_size() == 0) {
    ROS_INFO_STREAM("input data image or lidarpoints or cameraObject is error");
    ROS_INFO_STREAM("lidar points size:" << lidar_points.size());
    return false;
  }
  int width = 0;
  int height = 0;
  if (!img.empty()) {
    width = img.cols;
    height = img.rows;
  } else {
    width = camera_objects.width();
    height = camera_objects.height();
  }

  Eigen::Vector4d X;
  Eigen::Vector4d Y;
  X.setIdentity();
  Y.setIdentity();
  ROS_DEBUG_STREAM("lidar points size:" << lidar_points.size());
  float maxVal = 50.0;
  float val = 0.0;
  int red = 0;
  int green = 0;
  int blue = 0;
  if (!extrinsics_camera_map.count(camera_objects.sensor_name()) ||
      !intrinsics_camera_map.count(camera_objects.sensor_name())) {
    ROS_DEBUG_STREAM("no extrins or intrins");
    return false;
  }
  for (auto it = lidar_points.begin(); it != lidar_points.end(); ++it) {
    if (it->point3D_x < 0.0 || std::fabs(it->point3D_y) > 50.0 || std::isnan(it->point3D_x) ||
        std::isnan(it->point3D_y)) {
      continue;
    }

    X(0, 0) = it->point3D_x;
    X(1, 0) = it->point3D_y;
    X(2, 0) = it->point3D_z;
    X(3, 0) = 1;
    // todo 到相机坐标系转换并归一化
 
    X = extrinsics_camera_map.at(camera_objects.sensor_name()) * X;

    // Y = kMutiplyCamera2IMU * X;
    if (X(2) < 0.1) {
      continue;
    }
    Y = intrinsics_camera_map.at(camera_objects.sensor_name()) * X;
    cv::Point pt;
    pt.x = Y(0) / Y(2);
    pt.y = Y(1) / Y(2);

    if (pt.x > width || pt.x < 0.0 || pt.y > height || pt.y < 0.0) {
      continue;
    }
    val = it->point3D_x;
    blue = 153;
    green = 247;
    red = 255;
    cv::circle(img, pt, 3, cv::Scalar(blue, green, red),
               -1);  // origin 5 ->3
  }

#ifdef TEST_TIME
  end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::time_t end_time = std::chrono::system_clock::to_time_t(end);
  ROS_DEBUG_STREAM(__FUNCTION__ << " elapsed time: " << elapsed_seconds.count());
#endif
  return true;
}

void DebugLidarCameraSync(
  const std::unordered_map<std::string, std::list<cv_bridge::CvImagePtr>>& detection_image_buffer_map,
  const std::unordered_map<std::string, Eigen::Matrix4d>& intrinsics_camera_map,
  const std::unordered_map<std::string, Eigen::Matrix4d>& extrinsics_camera_map,
  const perception::VisualObjects& camera_objects,
  const CloudData& cloud_data,
  image_transport::Publisher& cloud_points_image_pub) {
#ifdef TEST_TIME
  std::chrono::time_point<std::chrono::system_clock> start, end;
  start = std::chrono::system_clock::now();
#endif
  // fine the newest camera detection objects
  double lidar_curent_time = cloud_data.time;

  cv::Mat lasteImg;
  double tempTime = 0.0;
  float min_timediff = 1.0;
  cv_bridge::CvImagePtr temp_ptr;
  int find_flag = -1;
  if (!detection_image_buffer_map.count("/perception/camera/object_detection_front60")) {
    return;
  }
  for (auto img_ptr =
           detection_image_buffer_map.at("/perception/camera/object_detection_front60").begin();
       img_ptr != detection_image_buffer_map.at("/perception/camera/object_detection_front60").end();
       img_ptr++) {
    tempTime = (*img_ptr)->header.stamp.sec + (*img_ptr)->header.stamp.nsec * 1e-9;
    ROS_INFO_STREAM("image_pointcloud_diff: " << fabs(tempTime - lidar_curent_time));
    if (fabs(tempTime - lidar_curent_time) < min_timediff) {
      min_timediff = fabs(tempTime - lidar_curent_time);
      temp_ptr = *img_ptr;
      find_flag = 1;
    }
  }
  if (find_flag == 1) {
    lasteImg = temp_ptr->image.clone();
  } else {
    std::cout << "no image !!!!" << std::endl;
    return;
  }
  // pcl::StopWatch filter_timer;
  std::vector<FrustumLidarPoint> frustLidarPoints;

  for (size_t i = 0; i < cloud_data.cloud_ptr->size(); i++) {
    pcl::PointXYZI point = cloud_data.cloud_ptr->at(i);
    if (point.x < 1 && point.x > 60) {
      continue;
    }

    FrustumLidarPoint tempPoint;
    tempPoint.id = i;
    tempPoint.point3D_x = point.x;
    tempPoint.point3D_y = point.y;
    tempPoint.point3D_z = point.z;
    tempPoint.point3D_r = point.intensity;
    frustLidarPoints.push_back(tempPoint);
  }
  // ROS_INFO_STREAM("Filter time=" << filter_timer.getTime() << "ms");
  bool ret = ProjectLidarPointsToImage(lasteImg, camera_objects, intrinsics_camera_map,
                                       extrinsics_camera_map, frustLidarPoints);
  // ROS_INFO_STREAM("LidarOnImage time=" << filter_timer.getTime() << "ms");
  if (!ret)  // 没有点或没有图像检测结果，直接退出
  {
    return;
  }
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", lasteImg).toImageMsg();
  cloud_points_image_pub.publish(msg);
  // ROS_INFO_STREAM("Publish Pub Cloud Filter time=" << filter_timer.getTime() << "ms");
}

void GetImageFrustum(const std::vector<perception::VisualObjects>& camera_frame_vec,
                     const std::unordered_map<std::string, Eigen::Matrix4d>& intrinsics_camera_map,
                     const std::unordered_map<std::string, Eigen::Matrix4d>& extrinsics_camera_map,
                     std::unordered_map<std::string, Eigen::MatrixXd>& world_points_map) {
  for (size_t j = 0; j < camera_frame_vec.size(); j++) {
    std::string camera_sensor_name = camera_frame_vec[j].sensor_name();
    if (!intrinsics_camera_map.count(camera_sensor_name) ||
        !extrinsics_camera_map.count(camera_sensor_name)) {
      continue;
    }
    Eigen::MatrixXd imgpoints(4, 3);
    imgpoints << 0, 0, 1, camera_frame_vec[j].width(), 0, 1, camera_frame_vec[j].width(),
        camera_frame_vec[j].height(), 1, 0, camera_frame_vec[j].height(), 1;
    Eigen::MatrixXd temp_points(4, 4);
    temp_points.setIdentity();
    temp_points.block(0, 0, 3, 4) =
        ((intrinsics_camera_map.at(camera_sensor_name).block(0, 0, 3, 3)).inverse()) *
        (50 * imgpoints.transpose());
    temp_points.block(3, 0, 1, 4) << 1, 1, 1, 1;
    Eigen::MatrixXd world_points(4, 4);
    world_points.setIdentity();
    world_points = extrinsics_camera_map.at(camera_sensor_name).inverse() * temp_points;
    world_points_map[camera_sensor_name] = world_points;
  }
}

void DisplayLidarImageIOU(
    const std::vector<perception::VisualObjects>& camera_frame_vec,
    std::unordered_map<int, std::vector<CameraIDRect>>& camera_rect_map,
    std::unordered_map<int, image_transport::Publisher>& pub_lidar_image_iou_map) {
  for (size_t j = 0; j < camera_frame_vec.size(); j++) {
    cv::Mat lasteImg(camera_frame_vec[j].height(), camera_frame_vec[j].width(), CV_8UC3,
                     cv::Scalar(255, 255, 255));
    for (int i = 0; i < camera_rect_map[j].size(); i++) {
      CameraIDRect rect_temp = camera_rect_map[j][i];
      cv::rectangle(lasteImg, rect_temp.camera_rect_, cv::Scalar(0, 0, 0),
                    2);  // cv::Scalar(0x27, 0xC1, 0x36)
      cv::putText(lasteImg, std::to_string(rect_temp.id),
                  cv::Point(rect_temp.camera_rect_.x, rect_temp.camera_rect_.y - 1),
                  cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 4);
    }
    for (int i = 0; i < camera_frame_vec[j].objs_size(); i++) {
      if (camera_frame_vec[j].objs(i).obj().type() > perception::ObjectType::TYPE_UNKNOWN_SMALL) {
        continue;
      }
      cv::Rect rect_temp2(camera_frame_vec[j].objs(i).x(), camera_frame_vec[j].objs(i).y(),
                          camera_frame_vec[j].objs(i).width(),
                          camera_frame_vec[j].objs(i).height());
      cv::rectangle(lasteImg, rect_temp2, cv::Scalar(0, 255, 0),
                    2);  // cv::Scalar(0x27, 0xC1, 0x36)
      cv::putText(lasteImg, "camera_object", cv::Point(rect_temp2.x, rect_temp2.y - 1),
                  cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 255, 0), 2);
    }
    sensor_msgs::ImagePtr msgNew =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", lasteImg).toImageMsg();
    pub_lidar_image_iou_map[j].publish(msgNew);
  }
}

}  // namespace mid_fusion
}  // namespace perception
