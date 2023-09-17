//
// Created by tiger on 23-5-6.
//

#include "object_marker_pub.h"

namespace perception {
namespace fusion {
std::vector<ROS_VISUALIZATION_MARKER>& ObjectMarkerPub::display(
    const TrackedObjects& tracked_objects) {
  std::vector<TrackedObject> obj_vec;
  for (size_t i = 0; i < tracked_objects.objs_size(); i++) {
    obj_vec.push_back(tracked_objects.objs(i));
  }

  if (params_.max_obj_size < obj_vec.size()) {
    params_.marker_list.resize(params_.max_obj_size);
  } else {
    for (size_t i = obj_vec.size(); i < params_.max_obj_size; ++i) {
      auto& tmp_marker = params_.marker_list[i];
      tmp_marker.action = ROS_VISUALIZATION_MARKER::DELETE;
    }
  }
  return params_.marker_list;
}

bool ObjectMarkerPub::FindKey(std::vector<TrackedObject> &obj_vec, int key) {
  for (auto tmp_obj : obj_vec) {
    if (tmp_obj.obj().id() == key) return true;
  }
  return false;
}

void ObjectMarkerPub::UpdateRviz(std::vector<TrackedObject> &obj_vec) {
  for (auto tmp_obj : id_key_) {
    auto obj_id = tmp_obj.first;
    if (FindKey(obj_vec, obj_id)) continue;
      for (auto marker_index : tmp_obj.second) {
         auto &tmp_marker = params_.marker_list[marker_index];
         tmp_marker.action = ROS_VISUALIZATION_MARKER::DELETE;
         del_mkid_index_.emplace_back(marker_index);
      }
      delete_key_nums_++;
  }
}

bool ObjectMarkerPub::FindDelIndex(std::vector<int> del_mkid_index, int mkid) {
  for (auto key : del_mkid_index) {
    if (key == mkid) return true;
  }
  return false;
}

std::vector<ROS_VISUALIZATION_MARKER>& ObjectMarkerPub::display_app(const TrackedObjects &tracked_objects,
                                                                    const localization::Localization localization) {
  localization_ = localization;
  std::vector<TrackedObject> obj_vec;
  int show_elements = 7;
  for (size_t i = 0; i < tracked_objects.objs_size(); i++) {
    obj_vec.push_back(tracked_objects.objs(i));
  }

  UpdateRviz(obj_vec);
  if (id_key_.size() < obj_vec.size()) {
    if (params_.max_obj_size > (obj_vec.size() + delete_key_nums_) * show_elements) {
      for (size_t i = show_elements * (obj_vec.size()); i < params_.max_obj_size; ++i) {
        auto& tmp_marker = params_.marker_list[i];
        tmp_marker.action = ROS_VISUALIZATION_MARKER::DELETE;
      }
    }else {
      params_.marker_list.resize((obj_vec.size() + delete_key_nums_) * show_elements);
      params_.max_obj_size = (obj_vec.size() + delete_key_nums_) * show_elements;
    }
  } else {
    int extend_size = id_key_.size() - delete_key_nums_;
    extend_size = obj_vec.size() - extend_size;
    if (params_.max_obj_size > (id_key_.size() + extend_size) * show_elements) {
      params_.max_obj_size = params_.max_obj_size;
    }else {
      params_.marker_list.resize((id_key_.size() + extend_size) * show_elements);
      params_.max_obj_size = (id_key_.size() + extend_size) * show_elements;
    }
    for (size_t i = show_elements * (obj_vec.size()); i < params_.max_obj_size; ++i) {
      auto& tmp_marker = params_.marker_list[i];
      tmp_marker.action = ROS_VISUALIZATION_MARKER::DELETE;
    }
  }
  mk_id = 0;
  id_key_.clear();


  for (size_t i = 0; i < obj_vec.size();) {
//    ROS_VISUALIZATION_MARKER track_marker;
    const auto& tmp_obj = obj_vec[i];
    auto &obj = tmp_obj.obj();
    auto key = obj.id();
    if (FindDelIndex(del_mkid_index_, mk_id)) {
//      update_key_nums_++;
      mk_id++;
      continue;
    }

    //TrackInfo
    auto& track_marker = params_.marker_list[mk_id];
    id_key_[key].emplace_back(mk_id);
    TrackInfo(tmp_obj, track_marker);
    mk_id++;

    //label
    auto& label_marker = params_.marker_list[mk_id];
    id_key_[key].emplace_back(mk_id);
    LabelInfo(tmp_obj, label_marker);
    mk_id++;

    //Arrow info
    auto& arrow_marker = params_.marker_list[mk_id];
    id_key_[key].emplace_back(mk_id);
    ArrowInfo(tmp_obj, arrow_marker, true);
    mk_id++;

    //Vel direction
    auto& velocity_marker = params_.marker_list[mk_id];
    id_key_[key].emplace_back(mk_id);
    ArrowInfo(tmp_obj, velocity_marker, false, 1.);
    mk_id++;

    //PolygonInfo
    auto& polygon_marker = params_.marker_list[mk_id];
    id_key_[key].emplace_back(mk_id);
    PolygonInfo(tmp_obj, polygon_marker);
    mk_id++;

    //trajectory
    auto &trajectory_marker = params_.marker_list[mk_id];
    id_key_[key].emplace_back(mk_id);
    TrajectoryInfo(tmp_obj, trajectory_marker);
    mk_id++;
    auto &trajectory_cube_marker = params_.marker_list[mk_id];
    id_key_[key].emplace_back(mk_id);
    TrajectoryCubeInfo(tmp_obj, trajectory_cube_marker);
    mk_id++;
    ++i;
  }

  int index = 0;
  for (auto marker : params_.marker_list) {
    index++;
  }
//  delete_map_.clear();
  delete_key_nums_ = 0;
  del_mkid_index_.clear();
  return params_.marker_list;
}


void ObjectMarkerPub::LabelInfo(const TrackedObject &tmp_obj, ROS_VISUALIZATION_MARKER &marker) {
  auto &obj = tmp_obj.obj();
//  marker.color.r = 0.0;
//  marker.color.g = 1.0;
//  marker.color.b = 1.;
  marker.scale = params_.default_scale_type;
  marker.action = ROS_VISUALIZATION_MARKER::ADD;
  marker.id = obj.id() + 1;
  marker.header.frame_id = options_.frame_id;
  marker.color = getClassColor(obj);
  marker.ns = std::to_string(obj.id());

  bool _has_camera = obj.has_camera_supplement();
  bool _has_lidar = obj.has_lidar_supplement();
  bool _has_radar = obj.has_radar_supplement();
  bool _has_falcon_lidar = obj.has_falcon_lidar_supplement();
  bool _has_obu = obj.has_obu_supplement();
  bool _has_vidar = obj.has_vidar_supplement();  // ming.du


  Eigen::Vector3d pos;
  pos << obj.center().x(), obj.center().y(), obj.center().z();
  pos.z() += 0.5;
  std::string text_label = object_type_2_name_map.at(obj.type()) + "-";
  //@lijian,显示匹配来源和id
  if (_has_lidar) {
    text_label += "L";
    text_label += std::to_string(obj.match_ids(0));
  }
  if (_has_falcon_lidar) {
    text_label += "F";
    text_label += std::to_string(obj.match_ids(3));
  }
  if (_has_camera) {
    text_label += "C";
    text_label += std::to_string(obj.match_ids(1));
  }
  if (_has_radar) {
    text_label += "R";
    text_label += std::to_string(obj.match_ids(2));
  }
  if (_has_obu) {
    text_label += "O";
  }
  if (_has_vidar) {
    text_label += "V";
    text_label += std::to_string(obj.match_ids(4));
  }
  //  STATE_DETECT = 0;  STATE_PREDICT = 1;
  if (obj.detect_state() == 0)
    text_label += "-DET";
  else
    text_label += "-PRE";
  drawText(pos, text_label, marker, 1., 1.);
}

void ObjectMarkerPub::TrackInfo(const TrackedObject& tmp_obj, ROS_VISUALIZATION_MARKER &marker) {
  //track info
  auto& obj = tmp_obj.obj();

//    tmp_marker.pose.orientation.w = 1.;
  marker.color.a = 1.f;

  marker.color = getClassColor(obj);
//    tmp_marker.color.r = 0.;
//    tmp_marker.color.g = 1.0;
//    tmp_marker.color.b = 0.;
  marker.scale = params_.default_scale_type;
  marker.ns = std::to_string(obj.id());
  marker.action = ROS_VISUALIZATION_MARKER::ADD;
//    tmp_marker.action = ROS_VISUALIZATION_MARKER::ADD;
  marker.id = obj.id();
  marker.header.frame_id = options_.frame_id;

  Eigen::Vector3d pos;
  pos << obj.center().x(), obj.center().y(), obj.center().z();
  pos.x() -= 0.5;
//  auto& tmp_marker = params_.marker_list[i];

  double speed =
      std::sqrt(std::pow(tmp_obj.velocity().x(), 2) + std::pow(tmp_obj.velocity().y(), 2));

  bool is_static = false;
  if (obj.motion_state() == 0)
    is_static = true;
  std::string text_track = "<" + num2str<int>(obj.id(), 0) + ">" +
                           num2str<double>(speed * 3.6f, 1) + "km/h" + "_s_" +
                           num2str<int>(is_static, 0);

  drawText(pos, text_track, marker, 1., 1.);

}

Eigen::Quaterniond ObjectMarkerPub::RPYToQuaterniond(const float& roll,
                                                    const float& pitch,
                                                    const float& yaw) {
  Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  Eigen::Quaterniond _orientation = yawAngle * pitchAngle * rollAngle;
  return _orientation;
}

void ObjectMarkerPub::TrajectoryInfo(const TrackedObject &tmp_obj, ROS_VISUALIZATION_MARKER &marker) {
  auto& obj = tmp_obj.obj();
  marker.scale = params_.default_scale_type;
  marker.ns = std::to_string(obj.id());
  marker.id = obj.id() + 5;
  marker.header.frame_id = options_.frame_id;
  marker.type = ROS_VISUALIZATION_MARKER::LINE_STRIP;
  marker.action = ROS_VISUALIZATION_MARKER::ADD;
  marker.scale.x = 0.1;
  marker.color = getClassColor(obj);
  marker.color.a = 0.5f;
  size_t p_size = obj.polygon_size();
  marker.points.reserve(p_size);
  marker.points.clear();
  for (size_t i = 0; i < p_size; ++i) {
    geometry_msgs::Point pts;
    pts.x = obj.polygon(i).x();
    pts.y = obj.polygon(i).y();
    pts.z = obj.polygon(i).z();
    marker.points.emplace_back(pts);
//    cube_marker.points.emplace_back(pts);
  }
}

void ObjectMarkerPub::TrajectoryCubeInfo(const TrackedObject &tmp_obj, ROS_VISUALIZATION_MARKER &marker) {
  auto& obj = tmp_obj.obj();
  marker.scale = params_.default_scale_type;
  marker.ns = std::to_string(obj.id());
  marker.id = obj.id() + 6;
  marker.header.frame_id = options_.frame_id;
  marker.type = ROS_VISUALIZATION_MARKER::CUBE_LIST;
  marker.action = ROS_VISUALIZATION_MARKER::ADD;
//  marker.scale.x = 0.1;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color = getClassColor(obj);
  marker.color.a = 0.5f;
  size_t p_size = obj.polygon_size();
  marker.points.reserve(p_size);
  marker.points.clear();
  for (size_t i = 0; i < p_size; ++i) {
    geometry_msgs::Point pts;
    pts.x = obj.polygon(i).x();
    pts.y = obj.polygon(i).y();
    pts.z = obj.polygon(i).z();
    marker.points.emplace_back(pts);
//    cube_marker.points.emplace_back(pts);
  }
}

void ObjectMarkerPub::ArrowInfo(const TrackedObject &tmp_obj, ROS_VISUALIZATION_MARKER &tmp_maker, bool is_arrow, double alpha) {
//  marker.type = ROS_VISUALIZATION_MARKER::ARROW;
  auto& obj = tmp_obj.obj();
//  auto& tmp_maker = ROS_VISUALIZATION_MARKER;
//  tmp_maker.color = getClassColor(tmp_obj);
  tmp_maker.scale = params_.default_scale_type;
  tmp_maker.ns = std::to_string(obj.id());
//  tmp_maker.action = ROS_VISUALIZATION_MARKER::ADD;

  tmp_maker.header.frame_id = options_.frame_id;
  tmp_maker.type = ROS_VISUALIZATION_MARKER::ARROW;
  tmp_maker.action = ROS_VISUALIZATION_MARKER::ADD;

  const auto& center = obj.center();
  const auto& size = obj.size();
  tmp_maker.color.a = alpha;
  double vx_baselink = .0;
  double vy_baselink = .0;
  if (is_arrow) {
    tmp_maker.color.r = 255;
    tmp_maker.color.g = 0;
    tmp_maker.color.b = 0;
    tmp_maker.id = obj.id() + 2;
  } else {
    tmp_maker.color.r = 0;
    tmp_maker.color.g = 255;
    tmp_maker.color.b = 0;
    tmp_maker.id = obj.id() + 3;
    //计算base_link方向下面obj的速度
    double host_yaw_global = localization_.yaw();
    double cos_host = std::cos(host_yaw_global);
    double sin_host = std::sin(host_yaw_global);
    vx_baselink = obj.velocity().x() * cos_host + obj.velocity().y() * sin_host;
    vy_baselink = -obj.velocity().x() * sin_host + obj.velocity().y() * cos_host;
    if (std::fabs(vx_baselink) < 0.001 && std::fabs(vy_baselink) < 0.001) {
      tmp_maker.action = ROS_VISUALIZATION_MARKER::DELETE;
      return;
    }
  }
  tmp_maker.pose.position.x = center.x();
  tmp_maker.pose.position.y = center.y();
  tmp_maker.pose.position.z = center.z();
  tmp_maker.scale.x = 3;
  tmp_maker.scale.y = 0.3;
  tmp_maker.scale.z = 0.1;
  Eigen::Quaterniond quat = RPYToQuaterniond(0.0, 0.0, is_arrow ? static_cast<float>(obj.angle()) : std::atan2(vy_baselink, vx_baselink));
  tmp_maker.pose.orientation.w = quat.w();
  tmp_maker.pose.orientation.x = quat.x();
  tmp_maker.pose.orientation.y = quat.y();
  tmp_maker.pose.orientation.z = quat.z();
}

void ObjectMarkerPub::PolygonInfo(const TrackedObject &tmp_obj, ROS_VISUALIZATION_MARKER &tmp_marker) {
  auto& obj = tmp_obj.obj();
//  auto tmp_marker = ROS_VISUALIZATION_MARKER;
//  tmp_marker.color = getClassColor(tmp_obj);
  tmp_marker.ns = std::to_string(obj.id());
  tmp_marker.id = obj.id() + 4;
  tmp_marker.header.frame_id = options_.frame_id;
  tmp_marker.type = ROS_VISUALIZATION_MARKER::LINE_LIST;
  tmp_marker.action = ROS_VISUALIZATION_MARKER::ADD;
  tmp_marker.scale.x = 0.1;
  tmp_marker.scale.y = 0.1;
  tmp_marker.scale.z = 0.1;
  tmp_marker.color = getClassColor(obj);
  tmp_marker.color.a = 0.5f;
//  if (obj.motion_state() == 0) {  // static-white
//    tmp_marker.color.r = 1.0;
//    tmp_marker.color.g = 1.0;
//    tmp_marker.color.b = 1.0;
//  } else {  // dynamic-green
//    tmp_marker.color.r = 0.0;
//    tmp_marker.color.g = 1.0;
//    tmp_marker.color.b = 0.0;
//  }
  tf::Quaternion quat = tf::createQuaternionFromRPY(0., 0., 0);
  tf::quaternionTFToMsg(quat, tmp_marker.pose.orientation);

  std::vector<geometry_msgs::Point> cub_points;
  size_t p_size = obj.contour_size();
  double p_low = obj.center().z() - obj.size().z() * 0.5f;
  double p_high = obj.center().z() + obj.size().z() * 0.5f;
  for (size_t i = 0; i < p_size; ++i) {
    geometry_msgs::Point pts;
    pts.x = obj.contour(i).x();
    pts.y = obj.contour(i).y();
    pts.z = p_low;
    cub_points.emplace_back(pts);
  }

  for (size_t i = 0; i < p_size; ++i) {
    geometry_msgs::Point pts;
    pts.x = obj.contour(i).x();
    pts.y = obj.contour(i).y();
    pts.z = p_high;
    cub_points.emplace_back(pts);
  }
//  auto& tmp_marker = params_.marker_list[i];
  tmp_marker.points.reserve(p_size * 6);
  tmp_marker.points.clear();

  for (size_t i = 0; i < p_size; ++i) {
    size_t next = i + 1;
    next = next < p_size ? next : 0;

    tmp_marker.points.emplace_back(cub_points[i]);
    tmp_marker.points.emplace_back(cub_points[next]);
  }
  for (size_t i = 0; i < p_size; ++i) {
    size_t next = i + 1;
    next = next < p_size ? next : 0;

    tmp_marker.points.emplace_back(cub_points[i + p_size]);
    tmp_marker.points.emplace_back(cub_points[next + p_size]);
  }

  for (size_t i = 0; i < p_size; ++i) {
    tmp_marker.points.emplace_back(cub_points[i]);
    tmp_marker.points.emplace_back(cub_points[i + p_size]);
  }
}

}
}
