#include "output_convertor.h"
namespace perception {
namespace fusion {

double OutputConvertor::overlap_iou_thresh_ = 0.15;
double OutputConvertor::center_distance_thresh_ = 20;

bool OutputConvertor::OutRoiObjectFilter(const fusion::ObjectPtr& fusion_object, bool has_obu) {
    if (has_obu)
        return false;
    //Modify @jiangnan: filter area [-50,80][-10,10]
    if (fusion_object->center_ego.x() > 80.0 || fusion_object->center_ego.x() < -50.0 ||
        fusion_object->center_ego.y() > 10.0 || fusion_object->center_ego.y() < -10.0) {
      return true;
    }
    return false;
}

void OutputConvertor::Fusion2App(const fusion::FrameConstPtr& frame,
                                 const std::vector<fusion::ObjectPtr>& fusion_objects,
                                 mogo::telematics::pad::TrackedObjects& app_tracked_objects,
                                 const localization::Localization localization,
                                 double zombie_thr) {
    ros::Time ts;
    ts.fromSec(frame->timestamp);
    double fusion_ts = ts.sec + ts.nsec * 1e-9;
    for (size_t i = 0; i < fusion_objects.size(); ++i) {
        const fusion::ObjectPtr& fusion_object_ptr = fusion_objects.at(i);
        // add source
        bool _has_camera = fusion_object_ptr->camera_supplement.on_use;
        bool _has_lidar = fusion_object_ptr->lidar_supplement.on_use;
        bool _has_radar = fusion_object_ptr->radar_supplement.on_use;
        bool _has_falcon_lidar = fusion_object_ptr->falcon_lidar_supplement.on_use;
        bool _has_vidar = fusion_object_ptr->vidar_supplement.on_use;
        bool _has_obu = fusion_object_ptr->obu_supplement.on_use;

        // object filter
        if (this->OutRoiObjectFilter(fusion_object_ptr, _has_obu))
            continue;

        //(1:falcon; 2:roadType is roadside; 3:type is PED)
        if (_has_falcon_lidar && fusion_object_ptr->status == 1 &&
            fusion_object_ptr->type == ObjectType::PEDESTRIAN)
          continue;

        mogo::telematics::pad::TrackedObject* app_object = app_tracked_objects.add_objs();

        app_object->set_uuid(fusion_object_ptr->track_id);

        // type
        ClassID classid = typeMap.find(fusion_object_ptr->type) != typeMap.end() ? typeMap[fusion_object_ptr->type] : ClassID::Unknown;
        app_object->set_type((unsigned int)classid);

        // set longitude/latitude/altitude
        double host_altitude = localization.altitude();
        double host_utm_pz = localization.position().z();
        double utm_gps_px = fusion_object_ptr->position.x();
        double utm_gps_py = fusion_object_ptr->position.y();
        UtmTransformGps(utm_gps_px, utm_gps_py, host_utm_pz);

        app_object->set_longitude(utm_gps_px);
        app_object->set_latitude(utm_gps_py);
        app_object->set_altitude(host_altitude);

        // set center: relative distance
        double cos_host = std::cos(fusion_object_ptr->host_yaw);
        double sin_host = std::sin(fusion_object_ptr->host_yaw);
        double dev_px_ego = fusion_object_ptr->center.x() * cos_host + fusion_object_ptr->center.y() * sin_host;
        double dev_py_ego = -fusion_object_ptr->center.x() * sin_host + fusion_object_ptr->center.y() * cos_host;
        // relative position
        app_object->mutable_center()->set_x(dev_px_ego);
        app_object->mutable_center()->set_y(dev_py_ego);
        app_object->mutable_center()->set_z(fusion_object_ptr->center.z());
        // relative angle
        app_object->set_angle(fusion_object_ptr->theta);

        // set heading
        // 0~2*PI:East:0, Counterclockwise => 0~2*PI:North:0, Clockwise
        auto heading = M_PI_2 - fusion_object_ptr->yaw;
        if (heading > 2 * M_PI) {
            heading -= 2 * M_PI;
        } else if (heading < 0) {
            heading += 2 * M_PI;
        }
        heading *= RAD_TO_DEG;
        float yaw_converted = this->ConvertToAngle(heading, utm_gps_px, utm_gps_py);
        app_object->set_heading(yaw_converted);

        // set speed
        double speed = std::sqrt(std::pow(fusion_object_ptr->velocity.x(), 2) + std::pow(fusion_object_ptr->velocity.y(), 2));
        app_object->set_speed(speed);

        // set static object's speed
        if (fusion_object_ptr->is_static) {
            app_object->set_speed(0.0);
        }

        // set time
        app_object->set_systemtime(ros::Time::now().toSec());
        app_object->set_satellitetime(fusion_ts);
        if (_has_obu) {
            mogo::telematics::pad::TrackedSource* main_source = app_object->add_tracked_source();
            mogo::telematics::pad::SubSource* sub_source = main_source->add_sub_source();
            main_source->set_source(2);
            perception::ObjectSource mogo_source;
            this->ApolloSource2MogoConvertor(fusion_object_ptr->obu_supplement.obu_sub_source, mogo_source);
            sub_source->set_source(mogo_source);
            if (mogo_source == perception::ObjectSource::V2V_BSM) {
                std::string srt_hex = this->DecStrToHexStr(std::to_string(fusion_object_ptr->obu_supplement.measurement_id));
                sub_source->set_id(srt_hex);
            } else if (mogo_source == perception::ObjectSource::V2N_RSI) {
                // TODO(@liuxinyu): 待验证
                //  to app 暂时不需要转
                // int rte_size = tracked_object.polygon_p_size();
                // for (size_t i = 0; i < rte_size; ++i) {
                //   geometry::Point rte_polygon = tracked_object.polygon_p(i);
                //   mogo::telematics::pad::Location* polygon_point_pad =
                //   object_app->add_polygon();
                //   polygon_point_pad->set_longitude(rte_polygon.x());
                //   polygon_point_pad->set_latitude(rte_polygon.y());
                // }
            }
            ROS_DEBUG_STREAM("ObjectsToApp: obu_sub_source: " << main_source->source() << " sub " << mogo_source << " "
                                                              << sub_source->source() << " id " << sub_source->id());
        }
        if (_has_lidar || _has_camera || _has_radar || _has_vidar || _has_falcon_lidar) {
            mogo::telematics::pad::TrackedSource* main_source = app_object->add_tracked_source();
            mogo::telematics::pad::SubSource* sub_source = main_source->add_sub_source();
            main_source->set_source(1);
            if (_has_lidar) {
                mogo::telematics::pad::SubSource* sub_source = main_source->add_sub_source();
                sub_source->set_source(1);
            }
            if (_has_camera) {
                mogo::telematics::pad::SubSource* sub_source = main_source->add_sub_source();
                sub_source->set_source(2);
            }
            if (_has_radar) {
                mogo::telematics::pad::SubSource* sub_source = main_source->add_sub_source();
                sub_source->set_source(3);
            }
            if (_has_vidar) {
                mogo::telematics::pad::SubSource* sub_source = main_source->add_sub_source();
                sub_source->set_source(4);
            }
            if (_has_falcon_lidar) {
                mogo::telematics::pad::SubSource* sub_source = main_source->add_sub_source();
                sub_source->set_source(5);
            }
        }

        int event_res = event_processor_.ZombieCarProcessor(fusion_object_ptr->obu_supplement.status_duration, zombie_thr);
        app_object->set_add_attribute(event_map.at(event_res));
    }
}

void OutputConvertor::Fusion2Mogo(const fusion::FrameConstPtr& frame,
                                  const std::vector<fusion::ObjectPtr>& fusion_objects,
                                  TrackedObjects& tracked_objects,
                                  const localization::Localization& localization) {
    static int seq = 0;
    tracked_objects.mutable_header()->set_seq(seq++);
    ros::Time ts;
    ts.fromSec(frame->timestamp);
    tracked_objects.mutable_header()->mutable_stamp()->set_sec(ts.sec);
    tracked_objects.mutable_header()->mutable_stamp()->set_nsec(ts.nsec);
    tracked_objects.mutable_header()->set_frame_id("base_link");
    tracked_objects.mutable_header()->set_module_name("perception_fusion");
    for (size_t i = 0; i < fusion_objects.size(); ++i) {
        TrackedObject* obj = tracked_objects.add_objs();
        this->ObjectFusion2Mogo(fusion_objects.at(i), obj, localization);
    }
}

void OutputConvertor::ObjectFusion2Mogo(const fusion::ObjectPtr& fusion_object_ptr,
                                        perception::TrackedObject* tracker_obj_ptr,
                                        const localization::Localization& localization) {
    perception::ObjectType mogo_type;
    perception::ObjectSource mogo_source;

    double host_yaw_global = localization.yaw();
    double cos_host = std::cos(host_yaw_global);
    double sin_host = std::sin(host_yaw_global);
    double devx = fusion_object_ptr->position[0] - localization.position().x();
    double devy = fusion_object_ptr->position[1] - localization.position().y();
    double dev_px_ego = devx * cos_host + devy * sin_host;
    double dev_py_ego = -devx * sin_host + devy * cos_host;

    tracker_obj_ptr->set_yaw(fusion_object_ptr->yaw);

    perception::Object* obj_ptr = tracker_obj_ptr->mutable_obj();
    {
        // obj_ptr->set_id(fusion_object_ptr->id);
        obj_ptr->set_id(fusion_object_ptr->track_id);  // TODO:check if id is used.

        obj_ptr->set_x_distance(dev_px_ego);
        obj_ptr->set_y_distance(dev_py_ego);
        obj_ptr->set_angle(fusion_object_ptr->theta);

        obj_ptr->mutable_center()->set_x(dev_px_ego);
        obj_ptr->mutable_center()->set_y(dev_py_ego);
        obj_ptr->mutable_center()->set_z(fusion_object_ptr->center_ego(2));

        obj_ptr->set_road_status(RoadType.at(fusion_object_ptr->status));

        if (obj_ptr->match_ids_size() == 0) {
            obj_ptr->add_match_ids(fusion_object_ptr->match_id_lidar);
            obj_ptr->add_match_ids(fusion_object_ptr->match_id_camera_60f);
            obj_ptr->add_match_ids(fusion_object_ptr->match_id_radar);
            obj_ptr->add_match_ids(fusion_object_ptr->match_id_falcon);
            obj_ptr->add_match_ids(fusion_object_ptr->match_id_vidar);
            obj_ptr->add_match_ids(fusion_object_ptr->match_id_camera_30f);
            obj_ptr->add_match_ids(fusion_object_ptr->match_id_camera_120r);
            obj_ptr->add_match_ids(fusion_object_ptr->match_id_camera_120f);
            obj_ptr->add_match_ids(fusion_object_ptr->match_id_obu);
        } else {
            obj_ptr->set_match_ids(0, fusion_object_ptr->match_id_lidar);
            obj_ptr->set_match_ids(1, fusion_object_ptr->match_id_camera_60f);
            obj_ptr->set_match_ids(2, fusion_object_ptr->match_id_radar);
            obj_ptr->set_match_ids(3, fusion_object_ptr->match_id_falcon);
            obj_ptr->set_match_ids(4, fusion_object_ptr->match_id_vidar);
            obj_ptr->set_match_ids(5, fusion_object_ptr->match_id_camera_30f);
            obj_ptr->set_match_ids(6, fusion_object_ptr->match_id_camera_120r);
            obj_ptr->set_match_ids(7, fusion_object_ptr->match_id_camera_120f);
            obj_ptr->set_match_ids(8, fusion_object_ptr->match_id_obu);
        }
        // this->AssignPolygonToRadarTrack(fusion_object_ptr, localization);

        obj_ptr->mutable_size()->set_x(fusion_object_ptr->size(0));
        obj_ptr->mutable_size()->set_y(fusion_object_ptr->size(1));
        obj_ptr->mutable_size()->set_z(fusion_object_ptr->size(2));
        // Modify @jiangnan
        geometry::Point* contour_p;
        geometry::Point* polygon_p;
        double xi{0}, yi{0};
        for (size_t i = 0; i < fusion_object_ptr->polygon_utm.size(); ++i) {
            contour_p = obj_ptr->add_contour();
            xi = fusion_object_ptr->polygon_utm.at(i).x - localization.position().x();
            yi = fusion_object_ptr->polygon_utm.at(i).y - localization.position().y();
            double contour_px = xi * cos_host + yi * sin_host;
            double contour_py = -xi * sin_host + yi * cos_host;
            contour_p->set_x(contour_px);
            contour_p->set_y(contour_py);
            contour_p->set_z(fusion_object_ptr->polygon_utm.at(i).z);
            // Modify(@liuxinyu): add output polygon_utm
            polygon_p = tracker_obj_ptr->add_contour_p();
            polygon_p->set_x(fusion_object_ptr->polygon_utm.at(i).x);
            polygon_p->set_y(fusion_object_ptr->polygon_utm.at(i).y);
            polygon_p->set_z(fusion_object_ptr->polygon_utm.at(i).z);
        }

        // Modify @jiangnan :添加轨迹到polygon字段
        for (const auto p : fusion_object_ptr->trajectory) {
          auto traj_p = obj_ptr->add_polygon();

          double devx = p[0] - localization.position().x();
          double devy = p[1] - localization.position().y();
          double dev_px_ego = devx * cos_host + devy * sin_host;
          double dev_py_ego = -devx * sin_host + devy * cos_host;

          traj_p->set_x(dev_px_ego);
          traj_p->set_y(dev_py_ego);
          traj_p->set_z(fusion_object_ptr->center_ego(2));
        }

        this->typeApollo2MogoConvertor(fusion_object_ptr->type, fusion_object_ptr->sub_type, mogo_type);
        obj_ptr->set_type(mogo_type);
        obj_ptr->set_confidence(fusion_object_ptr->confidence);

        //    Modify@ jiangnan: set motion state:
        //    MOTION_STATIC = 0;   MOTION_SUSPECTED_STATIC = 1;
        //    MOTION_WORM = 2;   MOTION_NORMAL = 3;
        if (fusion_object_ptr->is_static) {
            obj_ptr->set_motion_state(MOTION_STATIC);
        } else {
            obj_ptr->set_motion_state(MOTION_NORMAL);
        }
        //  STATE_DETECT = 0;  STATE_PREDICT = 1;
        if (fusion_object_ptr->is_predicted) {
            obj_ptr->set_detect_state(STATE_PREDICT);
        } else {
            obj_ptr->set_detect_state(STATE_DETECT);
        }

        // Modify @jiangnan: compute  relative velocity  in ego;
        double dev_vx_ego =
            fusion_object_ptr->velocity(0) * cos_host + fusion_object_ptr->velocity(1) * sin_host - localization.longitudinal_v();
        double dev_vy_ego =
            -fusion_object_ptr->velocity(0) * sin_host + fusion_object_ptr->velocity(1) * cos_host - localization.lateral_v();

        obj_ptr->mutable_velocity()->set_x(dev_vx_ego);
        obj_ptr->mutable_velocity()->set_y(dev_vy_ego);
        obj_ptr->mutable_velocity()->set_z(fusion_object_ptr->velocity(2));

        tracker_obj_ptr->mutable_velocity()->set_x(fusion_object_ptr->velocity(0));
        tracker_obj_ptr->mutable_velocity()->set_y(fusion_object_ptr->velocity(1));
        tracker_obj_ptr->mutable_velocity()->set_z(fusion_object_ptr->velocity(2));

        // Modify @jiangnan : computer accleration
        // double dev_ax_ego = fusion_object_ptr->acceleration(0) * cos_host + fusion_object_ptr->acceleration(1) * sin_host;
        // double dev_ay_ego = -fusion_object_ptr->acceleration(0) * sin_host + fusion_object_ptr->acceleration(1) * cos_host;

        // acceleration in Ego
        if (!fusion_object_ptr->is_static) {
            tracker_obj_ptr->set_absolute_longitude_a(fusion_object_ptr->acceleration_ego(0));
        }

        // acceleration in UTM
        obj_ptr->mutable_acceleration()->set_x(fusion_object_ptr->acceleration(0));
        obj_ptr->mutable_acceleration()->set_y(fusion_object_ptr->acceleration(1));
        obj_ptr->mutable_acceleration()->set_z(fusion_object_ptr->acceleration(2));

        obj_ptr->set_tracking_time(fusion_object_ptr->tracking_time);
        // Modify(@liuxinyu): v2x obs need know source
        if (fusion_object_ptr->obu_supplement.on_use) {
            // Modify(@liuxinyu): v2v_bsm - v2i_rsm - v2v_ssm - v2n_rsm
            this->ApolloSource2MogoConvertor(fusion_object_ptr->obu_supplement.obu_sub_source, mogo_source);
        } else {
            this->ApolloSource2MogoConvertor(fusion_object_ptr->source, mogo_source);
        }
        tracker_obj_ptr->set_source(mogo_source);

        if (fusion_object_ptr->lidar_supplement.on_use)
            obj_ptr->mutable_lidar_supplement()->set_on_use(true);
        if (fusion_object_ptr->camera_supplement.on_use)
            obj_ptr->mutable_camera_supplement()->set_on_use(true);
        if (fusion_object_ptr->radar_supplement.on_use)
            obj_ptr->mutable_radar_supplement()->set_on_use(true);
        if (fusion_object_ptr->obu_supplement.on_use) {
            obj_ptr->mutable_obu_supplement()->set_on_use(true);
            obj_ptr->mutable_obu_supplement()->set_measurement_id(fusion_object_ptr->obu_supplement.measurement_id);
            obj_ptr->mutable_obu_supplement()->set_status_duration(fusion_object_ptr->obu_supplement.status_duration);
        }
        if (fusion_object_ptr->vidar_supplement.on_use)
            obj_ptr->mutable_vidar_supplement()->set_on_use(true);
        if (fusion_object_ptr->falcon_lidar_supplement.on_use)
            obj_ptr->mutable_falcon_lidar_supplement()->set_on_use(true);
    }
}

void OutputConvertor::ApolloSource2MogoConvertor(const perception::fusion::ObjectSource& apollo_source,
                                                 perception::ObjectSource& mogo_source) {
    switch (apollo_source) {
        case perception::fusion::ObjectSource::V2V_BSM:
            mogo_source = perception::ObjectSource::V2V_BSM;
            break;
        case perception::fusion::ObjectSource::V2I_RSM:
            mogo_source = perception::ObjectSource::V2I_RSM;
            break;
        case perception::fusion::ObjectSource::V2V_SSM:
            mogo_source = perception::ObjectSource::V2V_SSM;
            break;
        case perception::fusion::ObjectSource::V2N_RSM:
            mogo_source = perception::ObjectSource::V2N_RSM;
            break;
        case perception::fusion::ObjectSource::V2I_SSM:
            mogo_source = perception::ObjectSource::V2I_SSM;
            break;
        case perception::fusion::ObjectSource::V2N_RSI:
            mogo_source = perception::ObjectSource::V2N_RSI;
            break;
        default:
            mogo_source = perception::ObjectSource::UNKNOWN;
            break;
    }
}

void OutputConvertor::OverlapObjectFilter(
    const std::vector<fusion::ObjectPtr> &fusion_objects,
    std::vector<fusion::ObjectPtr> &filter_objects) {
  std::vector<bool> overlap_result(fusion_objects.size(), false);
  std::vector<bool> is_unknown(fusion_objects.size(), false);

 // check that the type of the target is UNKNOWN
  for (size_t i = 0; i < fusion_objects.size(); i++) {
    if (fusion_objects[i]->type == ObjectType::UNKNOWN)
      is_unknown[i] = true;
  }

// check the overlap of two objects
  for (size_t i = 0; i < fusion_objects.size(); i++) {
    if (overlap_result[i])
      continue;
    fusion::ObjectPtr object_1 = fusion_objects[i];
    for (size_t j = 0; j < fusion_objects.size() && j != i; j++) {
      fusion::ObjectPtr object_2 = fusion_objects[j];
      if (overlap_result[j])
        continue;
      double center_dist = (object_1->position - object_2->position).norm();
      if (center_dist > this->center_distance_thresh_)
        continue;
      IOU iou;
      double iou_area = iou.Compute(object_1->track_id, object_2->track_id,
                      object_1->polygon_utm, object_2->polygon_utm);
      if (iou_area > this->overlap_iou_thresh_) {
        if (is_unknown[i] || is_unknown[j]) {
          // 1: object's type is UNKNOWN
          if (is_unknown[i])
            overlap_result[i] = true;
          else
            overlap_result[j] = true;
        } else if (object_1->tracking_time < object_2->tracking_time) {
          // 2: tracking_time
          overlap_result[i] = true;
        } else {
          overlap_result[j] = true;
        }
      }
    } // inner loop
  }   // outer loop

 // remove overlap object
  for (size_t i = 0; i < overlap_result.size(); i++) {
    if (overlap_result[i])
      continue;
    fusion::ObjectPtr object = fusion::ObjectPool::Instance().Get();
    *object = *(fusion_objects[i]);
    filter_objects.emplace_back(object);
  }
}

void OutputConvertor::AssignPolygonToRadarTrack(const fusion::ObjectPtr& fusion_object_ptr,
                                                const localization::Localization& localization) {
    if (fusion_object_ptr->radar_supplement.is_rear == true && fusion_object_ptr->lidar_supplement.on_use == false) {
        fusion_object_ptr->size(0) = 1.0;
        fusion_object_ptr->size(1) = 1.0;
        fusion_object_ptr->size(2) = 1.0;
        geometry::Point point[4];
        // LF
        point[0].set_x(fusion_object_ptr->position[0] - fusion_object_ptr->size(0) / 2);
        point[0].set_y(fusion_object_ptr->position[1] + fusion_object_ptr->size(1) / 2);
        point[0].set_z(fusion_object_ptr->position[2] + fusion_object_ptr->size(2) / 2);
        // LB
        point[1].set_x(fusion_object_ptr->position[0] - fusion_object_ptr->size(0) / 2);
        point[1].set_y(fusion_object_ptr->position[1] - fusion_object_ptr->size(1) / 2);
        point[1].set_z(0.0);
        // RB
        point[2].set_x(fusion_object_ptr->position[0] + fusion_object_ptr->size(0) / 2);
        point[2].set_y(fusion_object_ptr->position[1] - fusion_object_ptr->size(1) / 2);
        point[2].set_z(0.0);
        // RF
        point[3].set_x(fusion_object_ptr->position[0] + fusion_object_ptr->size(0) / 2);
        point[3].set_y(fusion_object_ptr->position[1] + fusion_object_ptr->size(1) / 2);
        point[3].set_z(fusion_object_ptr->position[2] + fusion_object_ptr->size(2) / 2);

        fusion_object_ptr->polygon_utm.clear();
        fusion::PointD contour_p;
        for (size_t i = 0; i < 4; ++i) {
            contour_p.x = point[i].x();
            contour_p.y = point[i].y();
            contour_p.z = point[i].z();
            fusion_object_ptr->polygon_utm.push_back(contour_p);
        }
    }
}

void OutputConvertor::AddVirtualObj(perception::TrackedObjects &tracked_objects,
                          const std::vector<perception::VirtualObject> &virtual_objects,
                          const localization::Localization &localization) {

    double host_yaw_global = localization.yaw();
    double cos_host = std::cos(host_yaw_global);
    double sin_host = std::sin(host_yaw_global);

    for (size_t i = 0; i < virtual_objects.size(); ++i) {

        double devx = virtual_objects[i].position().x() - localization.position().x();
        double devy = virtual_objects[i].position().y() - localization.position().y();
        double dev_px_ego = devx * cos_host + devy * sin_host;
        double dev_py_ego = -devx * sin_host + devy * cos_host;

        if (dev_px_ego > 100.0 || dev_px_ego < -50.0 ||
            dev_py_ego > 50.0 || dev_py_ego < -50.0)
            continue;

        TrackedObject *object_ptr = tracked_objects.add_objs();
        perception::Object *obj_ptr = object_ptr->mutable_obj();

        // ROS_INFO_THROTTLE(5, "PublishVirtualObject: id %d, ego_px %f, ego_py %f", virtual_objects[i].id(), dev_px_ego, dev_py_ego);

        obj_ptr->set_id(virtual_objects[i].id());
        obj_ptr->set_status(100);
        obj_ptr->set_type(perception::ObjectType::TYPE_UNKNOWN_STATIC);
        obj_ptr->set_confidence(1.0);
        obj_ptr->set_motion_state(MOTION_STATIC);
        obj_ptr->set_detect_state(STATE_DETECT);
        obj_ptr->set_tracking_time(1);
        obj_ptr->set_road_status(RoadType.at(0));

        // set relative position in ego
        obj_ptr->set_x_distance(dev_px_ego);
        obj_ptr->set_y_distance(dev_py_ego);
        obj_ptr->mutable_center()->set_x(dev_px_ego);
        obj_ptr->mutable_center()->set_y(dev_py_ego);
        obj_ptr->mutable_center()->set_z(virtual_objects[i].size().z() / 2.0);

        // set size
        obj_ptr->mutable_size()->set_x(virtual_objects[i].size().x());
        obj_ptr->mutable_size()->set_y(virtual_objects[i].size().y());
        obj_ptr->mutable_size()->set_z(virtual_objects[i].size().z());

        // set relative velocity in ego
        obj_ptr->mutable_velocity()->set_x(0.0);
        obj_ptr->mutable_velocity()->set_y(0.0);
        obj_ptr->mutable_velocity()->set_z(0.0);

        // set acceleration in ego
        obj_ptr->mutable_acceleration()->set_x(-localization.longitudinal_v());
        obj_ptr->mutable_acceleration()->set_y(-localization.lateral_v());
        obj_ptr->mutable_acceleration()->set_z(0.0);

        // set relative angle and absolute  yaw
        float angle = virtual_objects[i].yaw() - host_yaw_global;
        // float angle = 0.0;
        float cos_angle = std::cos(angle);
        float sin_angle = std::sin(angle);
        obj_ptr->set_angle(angle);
        object_ptr->set_yaw(virtual_objects[i].yaw());

        // set absolute velocity in UTM
        object_ptr->mutable_velocity()->set_x(0.0);
        object_ptr->mutable_velocity()->set_y(0.0);
        object_ptr->mutable_velocity()->set_z(0.0);

        // set absolute position in UTM
        object_ptr->set_longitude_p(virtual_objects[i].position().x());
        object_ptr->set_latitude_p(virtual_objects[i].position().y());
        object_ptr->set_alt(localization.position().z());

        // set relative polygon(contour) and absolute polygon(contour_p)
        geometry::Point *contour;
        geometry::Point *contour_p;
        double dev_px, contour_px, polygon_px;
        double dev_py, contour_py, polygon_py;
        // LF
        dev_px = virtual_objects[i].size().x() / 2.0;
        dev_py = virtual_objects[i].size().y() / 2.0;
        contour_px = dev_px * cos_angle + dev_py * sin_angle + dev_px_ego;
        contour_py = -dev_px * sin_angle + dev_py * cos_angle + dev_py_ego;
        contour = obj_ptr->add_contour();
        contour->set_x(contour_px);
        contour->set_y(contour_py);
        contour->set_z(0); // ego
        polygon_px = contour_px * cos_host - contour_py * sin_host + localization.position().x();
        polygon_py = contour_px * sin_host + contour_py * cos_host + localization.position().y();
        contour_p = object_ptr->add_contour_p();
        contour_p->set_x(polygon_px);
        contour_p->set_y(polygon_py);
        contour_p->set_z(localization.position().z()); // utm
        // LB
        dev_px = -virtual_objects[i].size().x() / 2.0;
        dev_py = virtual_objects[i].size().y() / 2.0;
        contour_px = dev_px * cos_angle + dev_py * sin_angle + dev_px_ego;
        contour_py = -dev_px * sin_angle + dev_py * cos_angle + dev_py_ego;
        contour = obj_ptr->add_contour();
        contour->set_x(contour_px);
        contour->set_y(contour_py);
        contour->set_z(0); // ego
        polygon_px = contour_px * cos_host - contour_py * sin_host + localization.position().x();
        polygon_py = contour_px * sin_host + contour_py * cos_host + localization.position().y();
        contour_p = object_ptr->add_contour_p();
        contour_p->set_x(polygon_px);
        contour_p->set_y(polygon_py);
        contour_p->set_z(localization.position().z()); // utm
        // RB
        dev_px = -virtual_objects[i].size().x() / 2.0;
        dev_py = -virtual_objects[i].size().y() / 2.0;
        contour_px = dev_px * cos_angle + dev_py * sin_angle + dev_px_ego;
        contour_py = -dev_px * sin_angle + dev_py * cos_angle + dev_py_ego;
        contour = obj_ptr->add_contour();
        contour->set_x(contour_px);
        contour->set_y(contour_py);
        contour->set_z(0); // ego
        polygon_px = contour_px * cos_host - contour_py * sin_host + localization.position().x();
        polygon_py = contour_px * sin_host + contour_py * cos_host + localization.position().y();
        contour_p = object_ptr->add_contour_p();
        contour_p->set_x(polygon_px);
        contour_p->set_y(polygon_py);
        contour_p->set_z(localization.position().z()); // utm
        // RF
        dev_px = virtual_objects[i].size().x() / 2.0;
        dev_py = -virtual_objects[i].size().y() / 2.0;
        contour_px = dev_px * cos_angle + dev_py * sin_angle + dev_px_ego;
        contour_py = -dev_px * sin_angle + dev_py * cos_angle + dev_py_ego;
        contour = obj_ptr->add_contour();
        contour->set_x(contour_px);
        contour->set_y(contour_py);
        contour->set_z(0); // ego
        polygon_px = contour_px * cos_host - contour_py * sin_host + localization.position().x();
        polygon_py = contour_px * sin_host + contour_py * cos_host + localization.position().y();
        contour_p = object_ptr->add_contour_p();
        contour_p->set_x(polygon_px);
        contour_p->set_y(polygon_py);
        contour_p->set_z(localization.position().z()); // utm
    }
}

}  // namespace fusion
}  // namespace perception