#pragma once

#include <tf/tf.h>  // Modify-guoxiaoxiao
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>
#include <unordered_map>

#include "common/proto/localization.pb.h"
#include "common/proto/object.pb.h"
#include "perception/base/object.h"
#include "perception/fusion/base/frame.h"

namespace perception {
namespace mid_fusion {

class Visualization {
 public:
  Visualization() = default;
  ~Visualization() = default;

  bool TextDisplay(TrackedObjects& objects, visualization_msgs::MarkerArray& marker_array);
  bool BboxDisplay(TrackedObjects& tracked_objects, visualization_msgs::MarkerArray& marker_array);
  void RadarObjDisplay(const fusion::FrameConstPtr& radar_frame,
                       localization::Localization localization,
                       ros::Publisher& publisher);  // Debug-guoxiaoxiao
  void CameraObjDisplay(const fusion::FrameConstPtr& camera_frame,
                        ros::Publisher& publisher);  // Debug-guoxiaoxiao
  void lineMarker(std::unordered_map<std::string, Eigen::MatrixXd> world_points_map,
                  visualization_msgs::MarkerArray& markers);
  bool PolygonDisplay(TrackedObjects& tracked_objects,
                      visualization_msgs::MarkerArray& marker_array);
  void PointsDisplay(const std::vector<std::vector<float>> frustum_points,
                     const std::vector<float> box_det2d,
                     const double pub_timestamp,
                     visualization_msgs::MarkerArray& marker_array);

 private:
  void AddPolygonPoint(const perception::Object& obj, visualization_msgs::Marker& marker);
  Eigen::Quaterniond RPYToQuaterniond(const float& roll = 0.0, const float& pitch = 0.0,
                                      const float& yaw = 0.0);
  void PointRotate(const double cosYaw, const double sinYaw, const double dcx, const double dcy,
                   geometry_msgs::Point& point);
  void AddTypeText(const perception::Object& obj, visualization_msgs::Marker& marker);
  void AddNoiseStateText(const perception::Object& obj, visualization_msgs::Marker& marker);
  void AddFusionText(const perception::Object& obj, visualization_msgs::Marker& marker);
  void AddBboxPoint(const perception::Object& obj, visualization_msgs::Marker& marker);
  // Modify-guoxiaoxiao
  void linkColorToSensorType(const perception::Object& obj, visualization_msgs::Marker& marker);
  // Modify-guoxiaoxiao
  void ObjectSetColorByType(const perception::Object& obj, visualization_msgs::Marker& marker);
  void ObjectTypeToColor(const perception::ObjectType type, visualization_msgs::Marker& marker);

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
  }
  void SetPointLBD(const perception::Object& object, geometry_msgs::Point& point) {
    point.x = object.center().x() - object.size().x() / 2.;
    point.y = object.center().y() + object.size().y() / 2.;
    point.z = object.center().z() - object.size().z() / 2.;
  }
  void SetPointRFD(const perception::Object& object, geometry_msgs::Point& point) {
    point.x = object.center().x() + object.size().x() / 2.;
    point.y = object.center().y() - object.size().y() / 2.;
    point.z = object.center().z() - object.size().z() / 2.;
  }
  void SetPointRBD(const perception::Object& object, geometry_msgs::Point& point) {
    point.x = object.center().x() + object.size().x() / 2.;
    point.y = object.center().y() + object.size().y() / 2.;
    point.z = object.center().z() - object.size().z() / 2.;
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

}  // namespace mid_fusion
}  // end of namespace perception
