#include "obstacle_radar_fusion.h"
#include <math.h>
#include <nav_msgs/GridCells.h>

namespace perception {
namespace radar {

RadarFusion::RadarFusion(const std::shared_ptr<ros::NodeHandle>& node) : node_(node) {}

bool RadarFusion::Init() {
  ros::NodeHandle pn("~");
  RadarFusionInitOptions options;
  ROS_INFO_STREAM("Init: read radar fusion config from: " << FLAGS_radar_fusion_conf_path);
  if (!::common::GetProtoFromFile(FLAGS_radar_fusion_conf_path, options)) {
    ROS_ERROR("Init: failed to load radar fusion config file.");
    return false;
  }
  roigrid_ptr_.reset(new MgRoigrid);
  ::common::Binary::SetName(Name());
  pn.param("match_min_distance_thresh", match_min_distance_thresh_, 0.3);
  pn.param("x_offset", x_offset_, 4.995);
  pn.param("perception_grid_map_path", perception_grid_map_path_,
           std::string("/perception/radar/config/map"));
  pn.param("radar_config_path", radar_config_path_, std::string("/perception/radar/config"));
  ROS_INFO_STREAM("Init: number of inpute radar sensors: " << options.input_sensor_size());
  ROS_INFO_STREAM("Init: main radar sensor: " << options.fusion_main_sensor());
  ROS_INFO_STREAM(
      "Init: output radar fusion obstacles topic name: " << options.output_obstacles_topic());
  ROS_INFO_STREAM("Init: output radar fusion rviz topic name: " << options.output_viz_topic());
  ROS_INFO_STREAM("Init: localization topic: " << options.localization_topic());
  ROS_INFO_STREAM("Init: perception_grid_map_path: " << perception_grid_map_path_);
  ROS_INFO_STREAM("Init: radar_config_path_: " << radar_config_path_);

  fusion_main_sensor_ = options.fusion_main_sensor();

  YAML::Node radar_config_node;
  std::string yaml_conf = radar_config_path_ + "/bus.yaml";
  bool tmp = LoadFile(yaml_conf, radar_config_node);
  YAML::Node roadmap_node;
  YamlSubNode(radar_config_node, "roadmap", roadmap_node);
  for (auto ri : roadmap_node) {
    YamlRoadMap roadmap_tmp;
    YamlRead(ri, "city", roadmap_tmp.city);
    YamlRead(ri, "utm_zone", roadmap_tmp.utm_zone);
    YamlRead(ri, "mappath", roadmap_tmp.mappath);
    YamlRead(ri, "version", roadmap_tmp.version);
    yaml_roadmap[roadmap_tmp.utm_zone] = roadmap_tmp;
  }

  YAML::Node radar_fusion_conf_node;
  YamlSubNode(radar_config_node, "radarFusion", radar_fusion_conf_node);
  YamlRead(radar_fusion_conf_node, "roadmap_shrink", roadmap_shrink_);
  YamlRead(radar_fusion_conf_node, "is_use_roadmap", is_use_roadmap_);
  YamlRead(radar_fusion_conf_node, "is_show_roadmap", is_show_roadmap_);
  YamlRead(radar_fusion_conf_node, "single_radar_max_id", single_radar_max_id_);
  YamlRead(radar_fusion_conf_node, "lowspeed_threshold", lowspeed_threshold_);
  YamlRead(radar_fusion_conf_node, "lowspeed_speed_threshold", lowspeed_speed_threshold_);
  YamlRead(radar_fusion_conf_node, "highspeed_speed_threshold", highspeed_speed_threshold_);
  YamlRead(radar_fusion_conf_node, "match_min_distance_thresh", match_min_distance_thresh_);
  YamlRead(radar_fusion_conf_node, "lowspeed_distance_threshold", lowspeed_distance_threshold_);
  YamlRead(radar_fusion_conf_node, "highspeed_distance_threshold", highspeed_distance_threshold_);
  YamlRead(radar_fusion_conf_node, "size_lowspeed_threshold", size_lowspeed_threshold_);
  YamlRead(radar_fusion_conf_node, "lowspeed_size_length_threshold", lowspeed_size_length_threshold_);
  YamlRead(radar_fusion_conf_node, "lowspeed_size_wideth_threshold", lowspeed_size_wideth_threshold_);
  YamlRead(radar_fusion_conf_node, "lowspeed_size_height_threshold", lowspeed_size_height_threshold_);
  YamlRead(radar_fusion_conf_node, "highspeed_size_length_threshold", highspeed_size_length_threshold_);
  YamlRead(radar_fusion_conf_node, "highspeed_size_wideth_threshold", highspeed_size_wideth_threshold_);
  YamlRead(radar_fusion_conf_node, "highspeed_size_height_threshold", highspeed_size_height_threshold_);
  ROS_INFO_STREAM("Init: roadmap_shrink: " << roadmap_shrink_);
  ROS_INFO_STREAM("Init: is_use_roadmap: " << is_use_roadmap_);
  ROS_INFO_STREAM("Init: is_show_roadmap: " << is_show_roadmap_);
  ROS_INFO_STREAM("Init: single_radar_max_id: " << single_radar_max_id_);
  ROS_INFO_STREAM("Init: lowspeed_threshold: "<< lowspeed_threshold_);
  ROS_INFO_STREAM("Init: lowspeed_speed_threshold: "<< lowspeed_speed_threshold_);
  ROS_INFO_STREAM("Init: highspeed_speed_threshold: "<< highspeed_speed_threshold_);
  ROS_INFO_STREAM("Init: match_min_distance_thresh: "<< match_min_distance_thresh_);
  ROS_INFO_STREAM("Init: lowspeed_distance_threshold: "<< lowspeed_distance_threshold_);
  ROS_INFO_STREAM("Init: highspeed_distance_threshold: "<< highspeed_distance_threshold_);
  ROS_INFO_STREAM("Init: size_lowspeed_threshold: "<< size_lowspeed_threshold_);
  ROS_INFO_STREAM("Init: lowspeed_size_length_threshold: "<< lowspeed_size_length_threshold_);
  ROS_INFO_STREAM("Init: lowspeed_size_wideth_threshold: "<< lowspeed_size_wideth_threshold_);
  ROS_INFO_STREAM("Init: lowspeed_size_height_threshold: "<< lowspeed_size_height_threshold_);
  ROS_INFO_STREAM("Init: highspeed_size_length_threshold: "<< highspeed_size_length_threshold_);
  ROS_INFO_STREAM("Init: highspeed_size_wideth_threshold: "<< highspeed_size_wideth_threshold_);
  ROS_INFO_STREAM("Init: highspeed_size_height_threshold: "<< highspeed_size_height_threshold_);

  int queueSize = 20;
  radar_objects_pub_ = pn.advertise<std_msgs::String>(options.output_obstacles_topic(), queueSize);
  radar_objects_viz_pub =
      pn.advertise<visualization_msgs::MarkerArray>(options.output_viz_topic(), queueSize);
  localization_subscriber_ = pn.subscribe(options.localization_topic(), queueSize,
                                          &RadarFusion::LocalizationCallback, this);
  pub_roadmap_ = pn.advertise<nav_msgs::GridCells>("viz_roadmap", 1, true);
  // processing for different radar sensor after receiving the message.
  base::SensorManager* sensor_manager = base::SensorManager::Instance();
  InputSensor input_sensor;
  for (size_t indx = 0; indx < options.input_sensor_size(); ++indx) {
    input_sensor = options.input_sensor(indx);
    if (sensor_manager->IsSensorExist(input_sensor.sensor_name())) {
      ROS_DEBUG_STREAM("Init: input_sensor.sensor_name() = " << input_sensor.sensor_name());
      if (!sensor_manager->GetSensorInfo(input_sensor.sensor_name(), &sensor_info_))
        ROS_ERROR_STREAM("Init: sensor: " << input_sensor.sensor_name()
                                          << " get sensor_info failure!");
      ros::Subscriber sub_sensor_radar =
          pn.subscribe(input_sensor.topic(), queueSize, &RadarFusion::RadarObstacleCallback, this);
      ROS_DEBUG_STREAM("Init: subscriber radar topic: " << input_sensor.topic());
      sensor_subscribers_.push_back(sub_sensor_radar);
    }
  }
  return true;
}

void RadarFusion::RadarObstacleCallback(const std_msgs::StringConstPtr& msg) {
  perception::RadarObjects radar_measurement;
  radar_measurement.ParseFromString(msg->data);
  std::string radar_name = radar_measurement.sensor_name();
  // update localization
  double timestamp =
      radar_measurement.header().stamp().sec() + radar_measurement.header().stamp().nsec() * 1e-9;
  localization::Localization localization;
  if (!UpdateRadarToImu(timestamp, localization)) {
    ROS_ERROR_STREAM("RadarObstacleCallback: Fail to get localization for Radar measurement.");
    return;
  }
  ROS_INFO_STREAM("RadarObstacleCallback: receive "
                  << radar_name << ": " << radar_measurement.objs_size() << " obstacles frame.");
  const auto it = sensor_frames_.find(radar_name);
  if (it == sensor_frames_.end()) {
    sensor_frames_.emplace(radar_name, radar_measurement);
  } else {
    it->second = radar_measurement;
  }

  if (radar_name == fusion_main_sensor_) {
    auto start_time = std::chrono::steady_clock::now();
    //change different radar id to different section
    ChangeObjectId(single_radar_max_id_);

    perception::RadarObjects radar_objects_fusion;
    static int seq = 0;
    radar_objects_fusion.mutable_header()->set_seq(seq++);
    radar_objects_fusion.mutable_header()->mutable_stamp()->set_sec(
        radar_measurement.header().stamp().sec());
    radar_objects_fusion.mutable_header()->mutable_stamp()->set_nsec(
        radar_measurement.header().stamp().nsec());
    radar_objects_fusion.mutable_header()->set_frame_id("base_link");
    radar_objects_fusion.mutable_header()->set_module_name("perception_radar_fusion");
    radar_objects_fusion.set_sensor_name(fusion_main_sensor_);

    for (auto frame = sensor_frames_.begin(); frame != sensor_frames_.end(); ++frame) {
      ROS_DEBUG_STREAM("RadarObstacleCallback: Add " << frame->first << ": "
                                                     << frame->second.objs_size()
                                                     << " related radar frame for fusion.");
      if (radar_objects_fusion.objs_size() == 0) {
        is_main_sensor_ = true;
        radarFilter(frame, &radar_objects_fusion, is_main_sensor_, localization);
      } else {
        is_main_sensor_ = false;
        radarFilter(frame, &radar_objects_fusion, is_main_sensor_, localization);
      }
    }

    //add yaw and polygon 
    ModifyShape(radar_objects_fusion, localization);
    // for (size_t i = 0; i < radar_objects_fusion.objs_size(); ++i) {
    //   ROS_INFO_STREAM(" id: "<< radar_objects_fusion.objs(i).obj().id()<< " yaw: " << radar_objects_fusion.objs(i).yaw()
    //               << " vx: " << radar_objects_fusion.objs(i).obj().velocity().x() 
    //               << " vy: " << radar_objects_fusion.objs(i).obj().velocity().y());
    //   for (size_t t = 0; t < radar_objects_fusion.objs(i).obj().contour_size(); ++t)
    //   {
    //     double x = radar_objects_fusion.objs(i).obj().contour(t).x();
    //     double y = radar_objects_fusion.objs(i).obj().contour(t).y();
    //     double z = radar_objects_fusion.objs(i).obj().contour(t).z();
    //     //ROS_INFO_STREAM("polygon: "<< x << "+" << y << "+" << z);
    //   }
    // }

    // RoiGrid Filter
    perception::RadarObjects radar_objects_fusion_filter;
    radar_objects_fusion_filter.mutable_header()->set_seq(seq++);
    radar_objects_fusion_filter.mutable_header()->mutable_stamp()->set_sec(
        radar_objects_fusion.header().stamp().sec());
    radar_objects_fusion_filter.mutable_header()->mutable_stamp()->set_nsec(
        radar_objects_fusion.header().stamp().nsec());
    radar_objects_fusion_filter.mutable_header()->set_frame_id("base_link");
    radar_objects_fusion_filter.mutable_header()->set_module_name("perception_radar_fusion");
    radar_objects_fusion_filter.set_sensor_name(fusion_main_sensor_);
    if (is_use_roadmap_) {
      //  base::MicrosecondTimer sensor_timer("RadarFusionRoiGrid");
      //  sensor_timer.begin();
      const float search_x = 0.5;
      const float search_y = 0.5;  // unit: m
      float unit_size = 0.2;
      float inverse_grid_size = 1.0 / unit_size;
      road_map_size_ = 1.0 / inverse_grid_size;
      roadmap_.clear();
      // update local_current
      localization::Localization local_current;
      double timestamp = radar_objects_fusion.header().stamp().sec() +
                         radar_objects_fusion.header().stamp().nsec() * 1e-9;
      if (!UpdateRadarToImu(timestamp, local_current)) {
        ROS_ERROR_STREAM("Fail to get localization for Radar measurement.");
        return;
      }
      roigrid_ptr_->local_cur_ptr = &local_current;
      roigrid_ptr_->grid_map_data_path = grid_map_data_path_;
      roigrid_ptr_->setShrinkSize(roadmap_shrink_);
      roigrid_ptr_->curfrm_used_paths.clear();
      // show road map
      if (is_show_roadmap_) {
        for (float local_pos_x = range.xmin; local_pos_x < range.xmax; local_pos_x += 0.2) {
          for (float local_pos_y = range.ymin; local_pos_y < range.ymax; local_pos_y += 0.2) {
            int road_type = 0;
            if (roigrid_ptr_->isInRoadMap(local_pos_x, local_pos_y, road_type)) {
              Eigen::Vector2d tmp;
              tmp << local_pos_x, local_pos_y;
              roadmap_.push_back(tmp);
            }
          }
        }
      }
      // roadmap filter
      for (size_t i = 0; i < radar_objects_fusion.objs_size(); i++) {
        int road_type = 0;
        Eigen::Vector2d local_pos(radar_objects_fusion.objs(i).obj().center().x(),
                                  radar_objects_fusion.objs(i).obj().center().y());
        bool is_in_road = roigrid_ptr_->isInRoadMap(local_pos.x(), local_pos.y(), road_type);
        if (!is_in_road) {
          continue;
        }
        perception::RadarObject* radar_obj_ptr = radar_objects_fusion_filter.add_objs();
        *radar_obj_ptr = radar_objects_fusion.objs(i);
      }

      // roigrid_ptr_->showPngsUsedThisFrame();
      roigrid_ptr_->delOldMap();
      //  sensor_timer.end();
    }
    ROS_DEBUG_STREAM(HDEBUG_B << "RadarObstacleCallback: radar_objects_fusion.objs_size():"
                              << radar_objects_fusion.objs_size());
    ROS_DEBUG_STREAM(HDEBUG_B << "RadarObstacleCallback: radar_objects_fusion_filter.objs_size():"
                              << radar_objects_fusion_filter.objs_size());
    std_msgs::String radar_msg;
    if (is_use_roadmap_) {
      RadarFusionDisplay(radar_objects_fusion_filter, radar_objects_viz_pub, param_, localization);
      // show road map
      RadarFusionShowRoadMap(pub_roadmap_);
      radar_objects_fusion_filter.SerializeToString(&radar_msg.data);
      ROS_INFO_STREAM("RadarObstacleCallback: radar filter object number: "
                      << radar_objects_fusion_filter.objs_size());
    } else {
      RadarFusionDisplay(radar_objects_fusion, radar_objects_viz_pub, param_, localization);
      radar_objects_fusion.SerializeToString(&radar_msg.data);
      ROS_INFO_STREAM("RadarObstacleCallback: radar origin object number: "
                      << radar_objects_fusion.objs_size());
    }
    radar_objects_pub_.publish(radar_msg);

    auto end_time = std::chrono::steady_clock::now();
    auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    ROS_INFO_STREAM("RadarObstacleCallback: time_diff: " << time_diff << "ms");
  }
}

double RadarFusion::GetRadarObjDis2D(const perception::RadarObject* radar_object1,
                                     const perception::RadarObject* radar_object2) {
  double dis_result = DBL_MAX;
  Eigen::Vector2d dis_vector(
      abs(radar_object1->obj().center().x() - radar_object2->obj().center().x()),
      abs(radar_object1->obj().center().y() - radar_object2->obj().center().y()));
  dis_result = dis_vector.norm();
  return dis_result;
}

double RadarFusion::GetRadarObjSpeed2D(const perception::RadarObject* radar_object1,
                                     const perception::RadarObject* radar_object2) {
  double dis_result = DBL_MAX;
  Eigen::Vector2d dis_vector(
      abs(radar_object1->obj().velocity().x() - radar_object2->obj().velocity().x()),
      abs(radar_object1->obj().velocity().y() - radar_object2->obj().velocity().y()));
  dis_result = dis_vector.norm();
  return dis_result;
}

bool RadarFusion::filter_front_308(const perception::RadarObject& radar_object) {
  bool is_filter = false;
  auto is_satisfaction = [](double x, double y) {
    if (abs(x) < y)
      return true;
    return false;
  };
  // need to subtract the difference between the translation of Radar to the center of the rear axis
  double x_distance = radar_object.obj().center().x() - x_offset_;
  double y_distance = radar_object.obj().center().y();
  double degree_10 = 60.0;
  double degree_70 = 40.0;
  double degree_150 = 6.0;
  double degree_250 = 4.0;
  double y_dis_10 = tan(degree_10 * M_PI / 180.0) * 10;
  double y_dis_70 = tan(degree_70 * M_PI / 180.0) * 70;
  double y_dis_150 = tan(degree_150 * M_PI / 180.0) * 150;
  double y_dis_250 = tan(degree_250 * M_PI / 180.0) * 250;
  int distance_flag = (x_distance <= 10) ? 1 : 0;
  distance_flag = (x_distance <= 70 && x_distance > 10) ? 2 : distance_flag;
  distance_flag = (x_distance <= 150 && x_distance > 70) ? 3 : distance_flag;
  distance_flag = (x_distance <= 250 && x_distance > 150) ? 4 : distance_flag;
  switch (distance_flag) {
    case 1:
      is_filter = is_satisfaction(y_distance, y_dis_10);
      break;
    case 2:
      is_filter = is_satisfaction(y_distance, y_dis_70);
      break;
    case 3:
      is_filter = is_satisfaction(y_distance, y_dis_150);
      break;
    case 4:
      is_filter = is_satisfaction(y_distance, y_dis_250);
      break;
    default:
      ROS_WARN_STREAM("This Object over threshold!");
      break;
  }
  return is_filter;
}

void RadarFusion::RadarFusionShowRoadMap(ros::Publisher& publisher) {
  float unit_size = road_map_size_;
  auto& roadmap = roadmap_;

  nav_msgs::GridCells cells;
  cells.header.frame_id = "base_link";
  cells.cell_height = unit_size;
  cells.cell_width = unit_size;
  if (roadmap.size() < 1) {
    geometry_msgs::Point obstacle;
    obstacle.x = 0;
    obstacle.y = 0;
    obstacle.z = 0;
    cells.cells.push_back(obstacle);
  }
  for (auto mi : roadmap) {
    geometry_msgs::Point obstacle;
    obstacle.x = mi.x();
    obstacle.y = mi.y();
    obstacle.z = 0;
    cells.cells.push_back(obstacle);
  }
  publisher.publish(cells);
  roadmap.clear();
}

bool RadarFusion::AttributeFilter(const perception::RadarObject* radar_object) {
  // 2 = measured, 3 = predicted
  if (radar_object->meas_state() != 2 || radar_object->probexist() < 0.89 ||
      radar_object->rcs() < -0.5) {
    return true;
  }
  return false;
}

bool RadarFusion::FovFilter(const perception::RadarObject* radar_object) {
  double radar_x = radar_object->obj().radar_supplement().x();
  double radar_y = radar_object->obj().radar_supplement().y();
  double degree = atan(radar_x/radar_y);
  ROS_INFO_STREAM("degree: " << degree/M_PI*180 << " M_PI: " << M_PI);
  if (abs(degree/M_PI*180) < 60) {
    return true;
  }
  return false;
}

void RadarFusion::ChangeObjectId(const size_t maxId) {
  size_t idAddNum = 0;
  for (auto frame = sensor_frames_.begin(); frame != sensor_frames_.end(); ++frame) {
    if (frame->first == "radar_front") {
      idAddNum = 0*maxId;
    }else if (frame->first.find("radar_front_left") != std::string::npos) {
      idAddNum = 1*maxId;
    }else if (frame->first.find("radar_front_right") != std::string::npos) {
      idAddNum = 2*maxId;
    }else if (frame->first.find("radar_rear_left") != std::string::npos) {
      idAddNum = 3*maxId;
    }else if (frame->first.find("radar_rear_right") != std::string::npos) {
      idAddNum = 4*maxId;
    }

    for (size_t m = 0; m < frame->second.objs_size(); ++m) {
      perception::RadarObject* radar_obj_fusion_ptr = frame->second.mutable_objs(m);
      auto old_id = radar_obj_fusion_ptr->obj().id();
      if (old_id < maxId) {
        radar_obj_fusion_ptr->mutable_obj()->set_id(old_id + idAddNum);
      }
    }
  }
}


void RadarFusion::radarFilter(const std::map<std::string, perception::RadarObjects>::iterator frame, 
                              perception::RadarObjects* radar_objects_fusion, bool is_main_sensor, 
                              localization::Localization local_current) {
  for (size_t i = 0; i < frame->second.objs_size(); ++i) {
    perception::RadarObject* radar_obj_ptr = frame->second.mutable_objs(i); 
    if ((!is_main_sensor) && (frame->first.find("anngic") != std::string::npos)) {       
      if (FovFilter(radar_obj_ptr)) { //filter the object less than 60 degrees
        continue;
      }
    }
    if ((frame->first.find("radar_rear_left") != std::string::npos) ||
        (frame->first.find("radar_rear_right") != std::string::npos)) {
      perception::Object* obj = radar_obj_ptr->mutable_obj();
      obj->mutable_radar_supplement()->set_is_rear(true);
    }
    double ego_vx = radar_obj_ptr->obj().velocity().x() + local_current.longitudinal_v();
    double ego_vy = radar_obj_ptr->obj().velocity().y();
    abs_speed_ = std::sqrt(std::pow(ego_vx, 2) + std::pow(ego_vy, 2));
    
    //judge distance
    delta_distance_ = 10.0;
    bool isAsso = false;
    for (size_t j = 0; j < radar_objects_fusion->objs_size(); ++j) {
      perception::RadarObject* radar_obj_fusion_ptr = radar_objects_fusion->mutable_objs(j);
      delta_distance_ = GetRadarObjDis2D(radar_obj_ptr, radar_obj_fusion_ptr);
      if (delta_distance_ < std::max(lowspeed_distance_threshold_, highspeed_distance_threshold_)) {
        //judge velocity
        delta_speed_ = GetRadarObjSpeed2D(radar_obj_ptr, radar_obj_fusion_ptr);

        bool isReplace = false;
        if (((abs_speed_ < lowspeed_threshold_) && (delta_distance_ < lowspeed_distance_threshold_) && (delta_speed_ < lowspeed_speed_threshold_)) ||
          ((abs_speed_ >= lowspeed_threshold_) && (delta_distance_ < highspeed_distance_threshold_) && (delta_speed_ < highspeed_speed_threshold_)) ||
           (delta_distance_ < match_min_distance_thresh_)) {
          isAsso = true;
          if (!is_main_sensor) {
            size_t down_id_value = 0;
            size_t up_id_value = 1*single_radar_max_id_;
            size_t id = radar_obj_fusion_ptr->obj().id();
            if ((id >= down_id_value) && (id < up_id_value)) {
              isReplace = false;
              break;
            }
          }

          double fusion_probexist = radar_obj_fusion_ptr->probexist();
          double obj_probexist = radar_obj_ptr->probexist();
          double delta_probexist = fusion_probexist - obj_probexist;
          if (delta_probexist > 0.00001) {//bigger
            isReplace = false;
          }else if (delta_probexist > -0.00001) {//equal
            double fusion_rcs = radar_obj_fusion_ptr->rcs();
            double obj_rcs = radar_obj_ptr->rcs();
            if (fusion_rcs > obj_rcs) {
              isReplace = false;
            }else {
              isReplace = true;
            }
          }else {//less
            isReplace = true;
          }
          if (true == isReplace) {
            radar_obj_fusion_ptr->CopyFrom(*radar_obj_ptr);
            break;
          }else {
            break;
          }
        }
      }
    }

    if (false == isAsso) {
      if (false == is_main_sensor) {
        if (radar_obj_ptr->obj().center().x() > 0) {
          if (filter_front_308(*radar_obj_ptr))
            continue;
        }
      }
      radar_objects_fusion->add_objs()->CopyFrom(*radar_obj_ptr);
    }
  }
}

void RadarFusion::ModifyShape(perception::RadarObjects& radar_objects_fusion, localization::Localization local_current) {
  double yaw_host = local_current.yaw();
  for (size_t i = 0; i < radar_objects_fusion.objs_size(); ++i) {
    perception::RadarObject *radar_object_ptr = radar_objects_fusion.mutable_objs(i);
    perception::Object *object = radar_object_ptr->mutable_obj();
    double vx_ego = object->velocity().x() + local_current.longitudinal_v();
    double vy_ego = object->velocity().y();
    double yaw_ori = radar_object_ptr->yaw();
    double yaw = 0.0;
    int32_t id = object->id();

    //yaw
    double angle = atan2(vy_ego, vx_ego);
    yaw = angle + yaw_host;
    yaw = yaw < 0 ? yaw + 2 * M_PI : yaw;
    yaw = yaw > 2 * M_PI ? yaw - 2 * M_PI : yaw;
    object->set_angle(angle);
    radar_object_ptr->set_yaw(yaw);

    //size
    double v_abs = std::sqrt(std::pow(vx_ego,1) + std::pow(vy_ego,2));
    double size_x = lowspeed_size_length_threshold_;
    double size_y = lowspeed_size_wideth_threshold_;
    double size_z = lowspeed_size_height_threshold_;
    if (v_abs > size_lowspeed_threshold_) {
      size_x = highspeed_size_length_threshold_;
      size_y = highspeed_size_wideth_threshold_;
      size_z = highspeed_size_height_threshold_;
    }
    object->mutable_size()->set_x(size_x);
    object->mutable_size()->set_y(size_y);
    object->mutable_size()->set_z(size_z);

    //polygon_ego
    geometry::Point contour_point;
    double sin_angle = std::sin(angle);
    double cos_angle = std::cos(angle);
    // LF
    double corner_x = size_x / 2;
    double corner_y = size_y / 2;
    ModifyPolygon(object, contour_point, sin_angle, cos_angle, corner_x, corner_y);
    // LB
    corner_x = -size_x / 2;
    corner_y = size_y / 2;
    ModifyPolygon(object, contour_point, sin_angle, cos_angle, corner_x, corner_y);
    // RB
    corner_x = -size_x / 2;
    corner_y = -size_y / 2;
    ModifyPolygon(object, contour_point, sin_angle, cos_angle, corner_x, corner_y);
    // RF
    corner_x = size_x / 2;
    corner_y = -size_y / 2;
    ModifyPolygon(object, contour_point, sin_angle, cos_angle, corner_x, corner_y);
  }
}

void RadarFusion::ModifyPolygon(perception::Object* object, geometry::Point& contour_point, 
                                double sin_angle, double cos_angle, 
                                double corner_x, double corner_y) {

    double rotate_corner_x = corner_x * cos_angle - corner_y * sin_angle + object->x_distance();
    double rotate_corner_y = corner_y * cos_angle + corner_x * sin_angle + object->y_distance();
    contour_point.set_x(rotate_corner_x);
    contour_point.set_y(rotate_corner_y);
    contour_point.set_z(0.0);
    object->add_contour()->CopyFrom(contour_point);
}



void RadarFusion::LocalizationCallback(const autopilot_msgs::BinaryDataConstPtr& msg) {
  while (global_localizations_.size() >= 200) {
    // if (lock_localization_)
    //   return;
    global_localizations_.pop_front();
  }
  localization::Localization global_localization;
  RosToProto(*msg, global_localization);
  static int s_utm_zone_last = -1024;  // no meaning
  int cur_utm_zone = global_localization.utm_zone();
  ROS_DEBUG_STREAM(HDEBUG_R << "LocalizationCallback: cur_utm_zone:" << cur_utm_zone);
  if (s_utm_zone_last != cur_utm_zone) {
    std::string child_name = "NoMap";
    for (auto yi = yaml_roadmap.begin(); yi != yaml_roadmap.end(); yi++) {
      ROS_DEBUG_STREAM(HDEBUG_B << "LocalizationCallback: yi->first:" << yi->first);
      ROS_DEBUG_STREAM(HDEBUG_B << "LocalizationCallback: cur_utm_zone:" << cur_utm_zone);
      if (yi->first == cur_utm_zone) {
        child_name = yi->second.mappath;
        break;
      }
    }
    grid_map_data_path_ = perception_grid_map_path_ + "/" + child_name;
    struct stat dir_info;
    if (!(stat(grid_map_data_path_.c_str(), &dir_info) == 0 && (dir_info.st_mode & S_IFDIR))) {
      ROS_WARN_STREAM("LocalizationCallback: Fail to get grid map directory.");
    }
    ROS_DEBUG_STREAM(HDEBUG_R << "LocalizationCallback: Load grid map:" << grid_map_data_path_);
    s_utm_zone_last = global_localization.utm_zone();
  }
  global_localizations_.push_back(global_localization);
}

bool RadarFusion::UpdateRadarToImu(const double& timestamp,
                                   localization::Localization& localization) {
  if (global_localizations_.empty()) {
    ROS_ERROR_STREAM("UpdateRadarToImu: Localization message NOT received.");
    return false;
  }

  localization = global_localizations_.front();
  // if (lock_localization_)
  //   return true;
  // reduce timestamp by time delay of sensor data transmission and perception consuming. now 0.0
  double stamp = timestamp + 0.05;
  double loc_ts;
  localization::Localization global_localization;
  for (auto it = global_localizations_.rbegin(); it != global_localizations_.rend(); it++) {
    global_localization = *it;
    loc_ts = global_localization.header().stamp().sec() +
             global_localization.header().stamp().nsec() * 1e-9;
    if (loc_ts > stamp) {
      continue;
    } else {
      localization = global_localization;
      break;
    }
  }
  if (abs(stamp - loc_ts) > 0.3) {
    ROS_ERROR_STREAM("UpdateRadarToImu: Time distance "
                     << std::setprecision(3) << abs(stamp - loc_ts)
                     << " between  Radar:: " << std::setprecision(18) << stamp
                     << " and localization: " << std::setprecision(18) << loc_ts
                     << " is too long.");
    return false;
  }
  return true;
}

}  // namespace radar
}  // namespace perception
