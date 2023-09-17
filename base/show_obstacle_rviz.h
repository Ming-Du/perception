#ifndef SRC_SHOW_OBSTACLE_RVIZ_H
#define SRC_SHOW_OBSTACLE_RVIZ_H
#include <visualization_msgs/MarkerArray.h>
//#include <std_msgs/Int32.h>
//#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <common/proto/object.pb.h>
#include "common/proto/localization.pb.h"
//#include "perception/camera/trt_yolov5/yololayer.h"



namespace perception {
namespace base{

class RvizDisplay {

public:
  RvizDisplay() =default;
  ~RvizDisplay() = default;

  bool Init();

  void ShowObstacle(perception::TrackedObjects& lidar_objs,
                    visualization_msgs::MarkerArray& marker_array,int type = 0);
  void ShowObstacle(perception::VisualObjects& camera_objs,
                    visualization_msgs::MarkerArray& marker_array,int type = 1);
  void ShowObstacle(std::vector<perception::Object>& objs,
                    visualization_msgs::MarkerArray& marker_array,int type = 0);

    private:
//  perception::TrackedObjects lidar_objects_;
//  perception::VisualObjects camera_objects_;

  void Build_BBox(visualization_msgs::Marker& marker,
                  perception::Object* object);
  geometry_msgs::Quaternion RotationAxistoQuaternion(
                  geometry_msgs::Vector3 axis, double angle);

  void ConvertVisualObjects(perception::VisualObjects& camera_obj,
                            std::vector<perception::Object>& objs);

  void ConvertTrackedObjects(perception::TrackedObjects& lidar_obj,
                            std::vector<perception::Object>& objs);
};



}
}
#endif //SRC_SHOW_OBSTACLE_RVIZ_H
