#pragma once

#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include "base/frame.h"
#include "common/gps_proj.h"  // Mo
#include "common/proto/localization.pb.h"
#include "common/proto/object.pb.h"
#include "perception/base/object.h"
#include "perception/base/distortion_model.h"

namespace perception {
namespace fusion {

class Visualization {
 public:
  Visualization() = default;
  ~Visualization() = default;

  bool TextDisplay(localization::Localization localization,
                   TrackedObjects& objects,
                   visualization_msgs::MarkerArray& marker_array);
  bool BboxDisplay(TrackedObjects& tracked_objects, visualization_msgs::MarkerArray& marker_array);
  void RadarObjDisplay(const fusion::FrameConstPtr& radar_frame, ros::Publisher& publisher);
  void ObuObjDisplay(const fusion::FrameConstPtr& obu_frame, ros::Publisher& publisher);
  void GroundPlaneDisplay(const std::vector<std::vector<Point3DD>> ground_planes, ros::Publisher& publisher);
  void VidarObjDisplay(const fusion::FrameConstPtr& vidar_frame,
                       ros::Publisher& publisher);  // ming.du
  // Modify @jiangnan:output polygon to rviz
  bool PolygonDisplay(localization::Localization localization,
                      TrackedObjects& tracked_objects,
                      visualization_msgs::MarkerArray& marker_array);
  void LaneImageDisplay(cv::Mat image,
                        const std::vector<std::vector<cv::Point2d>>& lanes,
                        ros::Publisher& publisher,
                        bool pub = false);
  void LidarImageDisplay(const fusion::FramePtr& frame,
                         const std::vector<fusion::ObjectPtr>& fused_objects,
                         ros::Publisher& publisher);
  void LineImageDisplay(cv::Mat image,
                        const std::vector<std::pair<double, std::vector<cv::Point2d>>>& lines,
                        ros::Publisher& publisher,
                        bool pub = false);
  void PointsImageDisplay(cv::Mat image,
                          const std::vector<std::vector<double>>& points,
                          ros::Publisher& publisher,
                          bool pub = false);
  void Camera3DDisplay(const std::vector<std::vector<double>>& objs,
                       ros::Publisher& publisher);
  // Modify(@liuxinyu): obu_test
  void ObuRTEDisplay(const perception::ObuObjects& obu_objets,
                     localization::Localization localization,
                     ros::Publisher& publisher);
  void ObuTmpObjDisplay(const perception::ObuObjects& obu_objets, ros::Publisher& publisher);

 private:
  Eigen::Quaterniond RPYToQuaterniond(const float& roll = 0.0,
                                      const float& pitch = 0.0,
                                      const float& yaw = 0.0);
  void PointRotate(const double cosYaw,
                   const double sinYaw,
                   const double dcx,
                   const double dcy,
                   geometry_msgs::Point& point);
  void AddTypeText(const perception::Object& obj, visualization_msgs::Marker& marker);
  void AddFusionMark(const perception::Object& obj, visualization_msgs::Marker& marker);
  void AddBboxPoint(const perception::Object& obj, visualization_msgs::Marker& marker);

  template <typename T>
  std::string NumToStr(T num, int precision) {
    std::stringstream ss;
    ss.setf(std::ios::fixed, std::ios::floatfield);
    ss.precision(precision);
    std::string st;
    ss << num;
    ss >> st;

    return st;
  }

  /* l=left  f=front d=down  r=right b=back u=up */
  /* lfd lbd  rfd rbd   lfu lbu  rfu rbu*/
  void SetPointLFD(const perception::Object& object, geometry_msgs::Point& point) {
    point.x = object.center().x() - object.size().x() / 2.;
    point.y = object.center().y() - object.size().y() / 2.;
    point.z = object.center().z() - object.size().z() / 2.;
    point.z = 0.;
  }
  void SetPointLBD(const perception::Object& object, geometry_msgs::Point& point) {
    point.x = object.center().x() - object.size().x() / 2.;
    point.y = object.center().y() + object.size().y() / 2.;
    point.z = object.center().z() - object.size().z() / 2.;
    point.z = 0.;
  }
  void SetPointRFD(const perception::Object& object, geometry_msgs::Point& point) {
    point.x = object.center().x() + object.size().x() / 2.;
    point.y = object.center().y() - object.size().y() / 2.;
    point.z = object.center().z() - object.size().z() / 2.;
    point.z = 0.;
  }
  void SetPointRBD(const perception::Object& object, geometry_msgs::Point& point) {
    point.x = object.center().x() + object.size().x() / 2.;
    point.y = object.center().y() + object.size().y() / 2.;
    point.z = object.center().z() - object.size().z() / 2.;
    point.z = 0.;
  }
  void SetPointLFU(const perception::Object& object, geometry_msgs::Point& point) {
    point.x = object.center().x() - object.size().x() / 2.;
    point.y = object.center().y() - object.size().y() / 2.;
    point.z = object.center().z() + object.size().z() / 2.;
  }
  void SetPointLBU(const perception::Object& object, geometry_msgs::Point& point) {
    point.x = object.center().x() - object.size().x() / 2.;
    point.y = object.center().y() + object.size().y() / 2.;
    point.z = object.center().z() + object.size().z() / 2.;
  }
  void SetPointRFU(const perception::Object& object, geometry_msgs::Point& point) {
    point.x = object.center().x() + object.size().x() / 2.;
    point.y = object.center().y() - object.size().y() / 2.;
    point.z = object.center().z() + object.size().z() / 2.;
  }
  void SetPointRBU(const perception::Object& object, geometry_msgs::Point& point) {
    point.x = object.center().x() + object.size().x() / 2.;
    point.y = object.center().y() + object.size().y() / 2.;
    point.z = object.center().z() + object.size().z() / 2.;
  }
};  // end of class Visualization

}  // end of namespace fusion
}  // end of namespace perception
