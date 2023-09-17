#include "common/data_converter.h"

namespace perception {
namespace mid_fusion {

constexpr double kSmallObjMaxExistProb = 0.9;
const std::vector<float> kPersonSize = {0.5, 0.5, 1.7};
const std::vector<float> kBicycleSize = {1.5, 0.5, 1.7};
const std::vector<float> kConeSize = {0.2, 0.2, 0.5};

perception::mid_fusion::GlobalToLocalTransformer g_localTransformer;


// void ModelPred2Mogo(){}
void Mogo2Tracker(const perception::TrackedObject& det_obj, mid_fusion::ObjectPtr& tracker_obj) {
  perception::Object object = det_obj.obj();
  tracker_obj->meas_id = object.id();
  tracker_obj->track_id = object.id();
  tracker_obj->direction << cosf(object.angle()), sinf(object.angle()), 0;
  tracker_obj->theta = object.angle();
  tracker_obj->center << object.center().x(), object.center().y(), object.center().z();
  tracker_obj->velocity << object.velocity().x(), object.velocity().y(), object.velocity().z();
  tracker_obj->size << object.size().x(), object.size().y(), object.size().z();
  tracker_obj->confidence = object.exist_confidence();
  tracker_obj->tracking_time = object.tracking_time();
  // tracker_obj->acceleration << object.acceleration().x(), object.acceleration().y(),
  //     object.acceleration().z();
  tracker_obj->type = mid_fusion::kMogo2TrackerTypeMap.at(object.type());
}

void Tracker2Mogo(const mid_fusion::Frame& detected_frame,
                  const mid_fusion::FramePtr& tracked_frame,
                  perception::TrackedObjects& fpnet_tracked_result,
                  const localization::Localization& localization) {
  static int seq = 0;
  fpnet_tracked_result.mutable_header()->set_seq(seq++);
  ros::Time ts;
  ts.fromSec(detected_frame.timestamp);
  fpnet_tracked_result.mutable_header()->mutable_stamp()->set_sec(ts.sec);
  fpnet_tracked_result.mutable_header()->mutable_stamp()->set_nsec(ts.nsec);
  fpnet_tracked_result.mutable_header()->set_frame_id("base_link");
  fpnet_tracked_result.mutable_header()->set_module_name("perception_mid_fusion");
  for (size_t i = 0; i < tracked_frame->objects.size(); ++i) {
    TrackedObject* obj = fpnet_tracked_result.add_objs();
    TrackerObj2Mogo(tracked_frame->objects[i], obj, localization);
  }
}

void TrackerObj2Mogo(mid_fusion::ObjectPtr& object_ptr,
                     perception::TrackedObject* tracker_obj_ptr,
                     const localization::Localization& local_current) {
  double host_yaw_global = local_current.yaw();
  double cos_host = std::cos(host_yaw_global);
  double sin_host = std::sin(host_yaw_global);
  double host_velocity = std::sqrt(std::pow(local_current.longitudinal_v(), 2) +
                                   std::pow(local_current.lateral_v(), 2));
  perception::Object* obj_ptr = tracker_obj_ptr->mutable_obj();
  // base_link
  {
    obj_ptr->set_id(object_ptr->track_id);
    obj_ptr->set_type(mid_fusion::kTracker2MogoTypeMap.at(object_ptr->type));
    obj_ptr->set_angle(object_ptr->theta);
    obj_ptr->set_exist_confidence(object_ptr->confidence);
    obj_ptr->mutable_size()->set_x(object_ptr->size(0));
    obj_ptr->mutable_size()->set_y(object_ptr->size(1));
    obj_ptr->mutable_size()->set_z(object_ptr->size(2));
    obj_ptr->mutable_center()->set_x(object_ptr->center(0));
    obj_ptr->mutable_center()->set_y(object_ptr->center(1));
    obj_ptr->mutable_center()->set_z(object_ptr->center(2));
    obj_ptr->set_x_distance(object_ptr->center(0));
    obj_ptr->set_y_distance(object_ptr->center(1));
    obj_ptr->mutable_velocity()->set_x(object_ptr->velocity(0));
    obj_ptr->mutable_velocity()->set_y(object_ptr->velocity(1));
    obj_ptr->mutable_velocity()->set_z(object_ptr->velocity(2));
    GenerateContour(obj_ptr);
  }
  // UTM
  {
    // ego coordinate -> world of utm
    double object_yaw_global = host_yaw_global + object_ptr->theta;
    if(object_yaw_global > 2*M_PI) {
      object_yaw_global -= 2*M_PI;
    } else if (object_yaw_global < 0) {
      object_yaw_global += 2*M_PI;
    }
    tracker_obj_ptr->set_yaw(object_yaw_global);
    double vel_x_utm =
        (obj_ptr->velocity().x() + host_velocity) * cos_host - obj_ptr->velocity().y() * sin_host;
    double vel_y_utm =
        (obj_ptr->velocity().x() + host_velocity) * sin_host + obj_ptr->velocity().y() * cos_host;
    if (obj_ptr->type() == perception::ObjectType::TYPE_TRIANGLEROADBLOCK) {
      vel_x_utm = 0;
      vel_y_utm = 0;
    }
    tracker_obj_ptr->mutable_velocity()->set_x(vel_x_utm);
    tracker_obj_ptr->mutable_velocity()->set_y(vel_y_utm);
    tracker_obj_ptr->mutable_velocity()->set_z(obj_ptr->velocity().z());
    int longitude_p = obj_ptr->center().x() * cos_host - obj_ptr->center().y() * sin_host +
                      local_current.position().x();
    int latitude_p = obj_ptr->center().x() * sin_host + obj_ptr->center().y() * cos_host +
                     local_current.position().y();
    tracker_obj_ptr->set_longitude_p(longitude_p);
    tracker_obj_ptr->set_latitude_p(latitude_p);
    tracker_obj_ptr->set_source(perception::ObjectSource::MID_FUSION);
  }
}

void GenerateContour(perception::Object* obj_ptr) {
  if (obj_ptr == nullptr) {
    return;
  }
  float pz = obj_ptr->center().z() - 0.5 * obj_ptr->size().z();
  float p1x = obj_ptr->center().x() - 0.5 * obj_ptr->size().x() * cos(obj_ptr->angle()) -
              0.5 * obj_ptr->size().y() * sin(obj_ptr->angle());
  float p1y = obj_ptr->center().y() - 0.5 * obj_ptr->size().x() * sin(obj_ptr->angle()) +
              0.5 * obj_ptr->size().y() * cos(obj_ptr->angle());
  float p2x = obj_ptr->center().x() - 0.5 * obj_ptr->size().x() * cos(obj_ptr->angle()) +
              0.5 * obj_ptr->size().y() * sin(obj_ptr->angle());
  float p2y = obj_ptr->center().y() - 0.5 * obj_ptr->size().x() * sin(obj_ptr->angle()) -
              0.5 * obj_ptr->size().y() * cos(obj_ptr->angle());
  float p3x = obj_ptr->center().x() + 0.5 * obj_ptr->size().x() * cos(obj_ptr->angle()) +
              0.5 * obj_ptr->size().y() * sin(obj_ptr->angle());
  float p3y = obj_ptr->center().y() + 0.5 * obj_ptr->size().x() * sin(obj_ptr->angle()) -
              0.5 * obj_ptr->size().y() * cos(obj_ptr->angle());
  float p4x = obj_ptr->center().x() + 0.5 * obj_ptr->size().x() * cos(obj_ptr->angle()) -
              0.5 * obj_ptr->size().y() * sin(obj_ptr->angle());
  float p4y = obj_ptr->center().y() + 0.5 * obj_ptr->size().x() * sin(obj_ptr->angle()) +
              0.5 * obj_ptr->size().y() * cos(obj_ptr->angle());
  geometry::Point point_temp;
  point_temp.set_x(p1x);
  point_temp.set_y(p1y);
  point_temp.set_z(pz);
  obj_ptr->add_polygon()->CopyFrom(point_temp);
  obj_ptr->add_contour()->CopyFrom(point_temp);
  point_temp.set_x(p2x);
  point_temp.set_y(p2y);
  point_temp.set_z(pz);
  obj_ptr->add_polygon()->CopyFrom(point_temp);
  obj_ptr->add_contour()->CopyFrom(point_temp);
  point_temp.set_x(p3x);
  point_temp.set_y(p3y);
  point_temp.set_z(pz);
  obj_ptr->add_polygon()->CopyFrom(point_temp);
  obj_ptr->add_contour()->CopyFrom(point_temp);
  point_temp.set_x(p4x);
  point_temp.set_y(p4y);
  point_temp.set_z(pz);
  obj_ptr->add_polygon()->CopyFrom(point_temp);
  obj_ptr->add_contour()->CopyFrom(point_temp);
}

void ConvertFpnetResultToObject(const std::vector<float>& pred_box3d_temp,
                                const PreprocessFpntInput& pre_res,
                                const localization::Localization& local_current,
                                const int index,
                                perception::TrackedObject& lidar_obj_temp) {
  perception::Object* object_ptr = lidar_obj_temp.mutable_obj();
  const int camera_2d_type = int(pre_res.box_det2d[index * 7 + 6]);
  // Center, size and heading from fpnet pred.
  object_ptr->mutable_center()->set_x(
      pred_box3d_temp[index * (int)FpnetPred::dim_size + (int)FpnetPred::center_x]);
  object_ptr->mutable_center()->set_y(
      pred_box3d_temp[index * (int)FpnetPred::dim_size + (int)FpnetPred::center_y]);
  object_ptr->mutable_center()->set_z(
      pred_box3d_temp[index * (int)FpnetPred::dim_size + (int)FpnetPred::center_z]);
  object_ptr->mutable_size()->set_x(
      pred_box3d_temp[index * (int)FpnetPred::dim_size + (int)FpnetPred::size_x]);
  object_ptr->mutable_size()->set_y(
      pred_box3d_temp[index * (int)FpnetPred::dim_size + (int)FpnetPred::size_y]);
  object_ptr->mutable_size()->set_z(
      pred_box3d_temp[index * (int)FpnetPred::dim_size + (int)FpnetPred::size_z]);
  object_ptr->set_angle(
      pred_box3d_temp[index * (int)FpnetPred::dim_size + (int)FpnetPred::heading]);
  const double confidence =
      pred_box3d_temp[index * (int)FpnetPred::dim_size + (int)FpnetPred::confidence];
  if (1 - confidence < PRECISION_NUMERIC) {
    object_ptr->set_exist_confidence(kSmallObjMaxExistProb);
  } else {
    object_ptr->set_exist_confidence(confidence);
  }
  // Add utm coordinate.
  const double host_yaw_global = local_current.yaw();
  const double cos_host = std::cos(host_yaw_global);
  const double sin_host = std::sin(host_yaw_global);
  int longitude_p = object_ptr->center().x() * cos_host - object_ptr->center().y() * sin_host +
                    local_current.position().x();
  int latitude_p = object_ptr->center().x() * sin_host - object_ptr->center().y() * cos_host +
                   local_current.position().y();
  lidar_obj_temp.set_longitude_p(longitude_p);
  lidar_obj_temp.set_latitude_p(latitude_p);
  object_ptr->mutable_velocity()->set_x(0-local_current.longitudinal_v());
  object_ptr->mutable_velocity()->set_y(0-local_current.lateral_v());
  object_ptr->mutable_velocity()->set_z(0);
  if (camera_2d_type == 1) {
    object_ptr->mutable_size()->set_x(kPersonSize[0]);
    object_ptr->mutable_size()->set_y(kPersonSize[1]);
    object_ptr->mutable_size()->set_z(kPersonSize[2]);
  } else if (camera_2d_type == 9) {
    object_ptr->mutable_size()->set_x(kConeSize[0]);
    object_ptr->mutable_size()->set_y(kConeSize[1]);
    object_ptr->mutable_size()->set_z(kConeSize[2]);
  } else if (camera_2d_type == 2 || camera_2d_type == 3 || camera_2d_type == 4) {
    object_ptr->mutable_size()->set_x(kBicycleSize[0]);
    object_ptr->mutable_size()->set_y(kBicycleSize[1]);
    object_ptr->mutable_size()->set_z(kBicycleSize[2]);
  }
  object_ptr->set_id(pre_res.box_ID[index]);
  object_ptr->set_type(FpnetIndexConvertToLidarType(camera_2d_type));
  object_ptr->set_sensor_name(pre_res.index_to_camtopic_map.at(index));
  GenerateContour(object_ptr);
}

void ConvertMapMsg2LaneData(hadmap::MapMsg map,
                            perception::mid_fusion::LaneMarkerDataRecord& lanemark,
                            const localization::Localization& local_current) {
  hadmap::Section* hadmap_section_temp;
  hadmap_section_temp = map.mutable_map()->mutable_roads(0)->mutable_sections(0);
  // section内boundary存放顺序[left2,left_right,left,right,right_left,right2]
  lanemark.time_stamp = common::TimeToSecond(*map.mutable_header()->mutable_stamp());
  HadmapMsgToLaneMarkerData(hadmap_section_temp->mutable_boundaries(0), lanemark.left2, local_current);
  HadmapMsgToLaneMarkerData(hadmap_section_temp->mutable_boundaries(1), lanemark.left_right, local_current);
  HadmapMsgToLaneMarkerData(hadmap_section_temp->mutable_boundaries(2), lanemark.left, local_current);
  HadmapMsgToLaneMarkerData(hadmap_section_temp->mutable_boundaries(3), lanemark.right, local_current);
  HadmapMsgToLaneMarkerData(hadmap_section_temp->mutable_boundaries(4), lanemark.right_left, local_current);
  HadmapMsgToLaneMarkerData(hadmap_section_temp->mutable_boundaries(5), lanemark.right2, local_current);
}

void HadmapMsgToLaneMarkerData(const hadmap::LaneBoundary* hadmap_laneboundary,
                               perception::mid_fusion::LaneMarkerData& marker_data,
                               const localization::Localization& local_current) {
  perception::mid_fusion::Point2d pt_temp;
  perception::mid_fusion::BoundaryPoint boundary_point;

  if (hadmap_laneboundary->point_size() < 3) {
    marker_data.confidence = 0;
  } else {
    marker_data.confidence = 1;
  }
  g_localTransformer.update(local_current.position().x(),
                            local_current.position().y(), local_current.yaw());
  marker_data.type = hadmap_laneboundary->type();
  marker_data.points.clear();
  marker_data.points_with_type.clear();

  marker_data.points.reserve(hadmap_laneboundary->point_size());
  marker_data.points_with_type.reserve(hadmap_laneboundary->point_size());
  // ROS_ERROR_COND(hadmap_laneboundary->point_size() != hadmap_laneboundary->point_size(), \
    //   "Lane Boundary size diff %ld, %ld", hadmap_laneboundary->point_size(), hadmap_laneboundary->point_size());
  for (int i = 0; i < hadmap_laneboundary->point_size(); i++) {
    pt_temp.x = hadmap_laneboundary->point(i).x();
    pt_temp.y = hadmap_laneboundary->point(i).y();
    g_localTransformer.transform(pt_temp.x, pt_temp.y);
    // points_with_type
    boundary_point.type = hadmap_laneboundary->point(i).point_type();
    boundary_point.x = pt_temp.x;
    boundary_point.y = pt_temp.y;
    if (i == 0) {
      ROS_INFO("HadmapMsgToLaneMarkerData: the i  = %d  local point  is %f, %f", i, boundary_point.x, boundary_point.y);
    }

    if (pt_temp.x > -0.5 && pt_temp.x < 300)  // influence lane fake. point num & first_y_left
    {
      marker_data.points.push_back(pt_temp);
      marker_data.points_with_type.push_back(boundary_point);
    }
  }
}

}  // namespace mid_fusion
}  // namespace perception
