#include "show_obstacle_rviz.h"
#include "track_object_types.h"
#include "camera_util.h"
namespace perception {
namespace base
{

bool RvizDisplay::Init(){

  return true;
};

void RvizDisplay::ShowObstacle(perception::VisualObjects& camera_objs,
                               visualization_msgs::MarkerArray& marker_array,
                               int type){
  std::vector<perception::Object> objs;
  ConvertVisualObjects(camera_objs,objs);
  ShowObstacle(objs,marker_array,type);
}

void RvizDisplay::ShowObstacle(perception::TrackedObjects& lidar_objs,
                               visualization_msgs::MarkerArray& marker_array,
                               int type){
  std::vector<perception::Object> objs;
  ConvertTrackedObjects(lidar_objs,objs);
  ShowObstacle(objs,marker_array,type);
}
void RvizDisplay::ShowObstacle(std::vector<perception::Object>& objs,
                               visualization_msgs::MarkerArray& marker_array,
                               int type){
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.header.frame_id = "base_link";

  static size_t last_marker_size_ = 0;
  int32_t mk_id = 0;
//for iter in objs:
  perception::Object* object;
  for (int i = 0; i < objs.size(); i++) {
    object = &objs[i];

    if (object->size().x() > 8
        || object->size().y() > 4
        || object->size().z() > 4) continue;
// 文字标记
    marker.id = mk_id++;
    marker.points.clear();
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    marker.text = "x:" + std::to_string(object->center().x())
                  + "\ny:" + std::to_string(object->center().y())
                  + "\nz:" + std::to_string(object->center().z());
//                  + "\nvx:" + std::to_string(object->velocity().x())
//                  + "\nvy:" + std::to_string(object->velocity().y());
//        + "\nself_speed:" + std::to_string(vehicle_velocity);

    marker.scale.x = 0.f;
    marker.scale.y = 0.f;
    marker.scale.z = 0.5f;
    if(type == 1){
      marker.color.a = 1.0f;
      marker.color.r = 1.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
    } else if (type == 2){
      marker.color.a = 1.0f;
      marker.color.r = 0.7f;
      marker.color.g = 0.0f;
      marker.color.b = 0.2f;
    } else{
      marker.color.a = 1.0f;
      marker.color.r = 0.76f;
      marker.color.g = 0.82f;
      marker.color.b = 0.94f;
    }
    marker.pose.position.x = object->center().x();
    marker.pose.position.y = object->center().y();
    marker.pose.position.z = object->center().z() + 1.5;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.points.clear();
    marker_array.markers.emplace_back(marker);
//
{
    //Object 中心处画个小球
//    marker.id = mk_id++;
//    marker.type = visualization_msgs::Marker::SPHERE;
//    marker.action = visualization_msgs::Marker::ADD;
//    marker.scale.x = 0.4f;
//    marker.scale.y = 0.4f;
//    marker.scale.z = 0.4f;
//    marker.color.a = 0.7f;
//    marker.color.r = 0.5f;
//    marker.color.g = 0.f;
//    marker.color.b = 0.5f;
//    marker.pose.position.x = object->center().x();
//    marker.pose.position.y = object->center().y();
//    marker.pose.position.z = object->center().z();
//    marker_array.markers.emplace_back(marker);
//  }

//  {
    //cubic
//    marker.id = mk_id++;
//    marker.type = visualization_msgs::Marker::CUBE;
//    marker.action = visualization_msgs::Marker::ADD;
//    marker.scale.x = object->size().x();
//    marker.scale.y = object->size().y();
//    marker.scale.z = object->size().z();
//    marker.color.a = 0.4f;
//    marker.color.r = 1.0f;
//    marker.color.g = 1.0f;
//    marker.color.b = 0.0f;
//    marker.pose.position.x = object->center().x();
//    marker.pose.position.y = object->center().y();
//    marker.pose.position.z = object->center().z();
//    marker_array.markers.emplace_back(marker);

}

//  {
    //bbox
    double angle = 0.0;
    geometry_msgs::Vector3 axis;
    axis.x = 0.0;
    axis.y = 0.0;
    axis.z = 1.0;
    marker.id = mk_id++;
    marker.header.frame_id = "base_link";
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.text.clear();
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = object->center().x();
    marker.pose.position.y = object->center().y();
    marker.pose.position.z = object->center().z();
    marker.scale.x = 0.05;
    marker.scale.y = 0.0;
    marker.scale.z = 0.0;
    if(type == 1){
      marker.color.a = 1.0f;
      marker.color.r = 1.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
    } else if (type == 2){
      marker.color.a = 1.0f;
      marker.color.r = 0.7f;
      marker.color.g = 0.0f;
      marker.color.b = 0.2f;
    } else{
      marker.color.a = 1.0f;
      marker.color.r = 0.76f;
      marker.color.g = 0.82f;
      marker.color.b = 0.94f;
    }

    Build_BBox(marker, object);
    if(object->velocity().x() > 1.0 ){
      angle = atan2(object->velocity().y(), object->velocity().x());
      marker.pose.orientation = RotationAxistoQuaternion(axis,angle);
    }
    marker_array.markers.emplace_back(marker);

  }
  while (mk_id < last_marker_size_) {
    visualization_msgs::Marker marker;
    marker.id = mk_id++;
    marker.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.push_back(marker);
  }

  last_marker_size_ = marker_array.markers.size();

}

void RvizDisplay::Build_BBox(visualization_msgs::Marker& marker,
                perception::Object* object){
  double x = object->center().x();
  double y = object->center().y();
  double z = object->center().z();
  double a = object->size().x();
  double b = object->size().y();
  double c = object->size().z();
  geometry_msgs::Point p1,p2,p3,p4,p5,p6,p7,p8;
  p1.x =  - a/2;   p1.y =  - b/2;   p1.z =  - c/2;
  p2.x =  - a/2;   p2.y =  + b/2;   p2.z =  - c/2;
  p3.x =  - a/2;   p3.y =  + b/2;   p3.z =  + c/2;
  p4.x =  - a/2;   p4.y =  - b/2;   p4.z =  + c/2;
  p5.x =  + a/2;   p5.y =  - b/2;   p5.z =  - c/2;
  p6.x =  + a/2;   p6.y =  + b/2;   p6.z =  - c/2;
  p7.x =  + a/2;   p7.y =  + b/2;   p7.z =  + c/2;
  p8.x =  + a/2;   p8.y =  - b/2;   p8.z =  + c/2;
  marker.points.push_back(p1);
  marker.points.push_back(p2);
  marker.points.push_back(p2);
  marker.points.push_back(p3);
  marker.points.push_back(p3);
  marker.points.push_back(p4);
  marker.points.push_back(p4);
  marker.points.push_back(p1);

  marker.points.push_back(p5);
  marker.points.push_back(p6);
  marker.points.push_back(p6);
  marker.points.push_back(p7);
  marker.points.push_back(p7);
  marker.points.push_back(p8);
  marker.points.push_back(p8);
  marker.points.push_back(p5);

  marker.points.push_back(p1);
  marker.points.push_back(p5);
  marker.points.push_back(p2);
  marker.points.push_back(p6);
  marker.points.push_back(p3);
  marker.points.push_back(p7);
  marker.points.push_back(p4);
  marker.points.push_back(p8);

}

geometry_msgs::Quaternion RvizDisplay::RotationAxistoQuaternion(
    geometry_msgs::Vector3 axis, double angle) {
  geometry_msgs::Quaternion quaternion;
  quaternion.x = axis.x * sin(angle / 2);
  quaternion.y = axis.y * sin(angle / 2);
  quaternion.z = axis.z * sin(angle / 2);
  quaternion.w = cos(angle / 2);
  return quaternion;
}


void RvizDisplay::ConvertVisualObjects(perception::VisualObjects& camera_obj,
                                       std::vector<perception::Object>& objs){
  perception::Object object;
  perception::VisualObject visualObject;
  Eigen::Vector2d position;
  const std::string sensor_name = camera_obj.sensor_name();

  const float w = 2.0 * track_object_types.at(ObjectType::TYPE_CAR)[0];//car width
  const float l = 2.0 * track_object_types.at(ObjectType::TYPE_CAR)[1];//car length
  for (int i = 0; i < camera_obj.objs_size(); i++){
    visualObject = camera_obj.mutable_objs()->Get(i);

    if(visualObject.obj().type() == perception::TYPE_CAR){
      calculate_car_position(
        sensor_name, visualObject.obj().camera_supplement().box(), position, w, l);

      object.mutable_center()->set_x(position.x());
      object.mutable_center()->set_y(position.y());
      object.mutable_center()->set_z(0.85);
      object.mutable_size()->set_x(l);
      object.mutable_size()->set_y(w);
      object.mutable_size()->set_z(1.5);
      objs.push_back(object);
    }
  }
}

void RvizDisplay::ConvertTrackedObjects(perception::TrackedObjects& lidar_obj,
                                        std::vector<perception::Object>& objs){
  perception::Object object;
  for(int i = 0; i < lidar_obj.objs_size(); i++){
    object = *lidar_obj.mutable_objs(i)->mutable_obj();
    objs.push_back(object);
  }

}

}
}