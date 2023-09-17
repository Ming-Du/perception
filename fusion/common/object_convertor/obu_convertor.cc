#include "obu_convertor.h"

namespace perception {
namespace fusion {

bool ObuConvertor::Mogo2Fusion(const perception::ObuObject& obu_object,
                               const localization::Localization& localization,
                               fusion::ObjectPtr& fusion_object) {
    perception::Object object = obu_object.obj();
    /*
    // convert obu position from gps to ego frame
    perception::fusion::point object_gps;
    object_gps.x = obu_object.longitude();
    object_gps.y = obu_object.latitude();
    // utm convert to vehicle of obj
    perception::fusion::point object_utm = GPS2MCT(object_gps);
    double utm_x_diff = object_utm.x - localization.position().x();
    double utm_y_diff = object_utm.y - localization.position().y();
    double host_yaw_global = localization.yaw();
    double cos_host = std::cos(host_yaw_global);
    double sin_host = std::sin(host_yaw_global);
    // Debug(@liuxinyu): temporarity
    double pos_x_ego =
        utm_x_diff * cos_host + utm_y_diff * sin_host - object.size().x() / 2;
    double pos_y_ego = -utm_x_diff * sin_host + utm_y_diff * cos_host;
    // filter over 300
    if (abs(pos_x_ego) > 300.0 || abs(pos_y_ego) > 300.0)
      return false;
    std::cout << " **** " << std::endl;
    std::cout << " " << object.center().x() << " " << object.center().y() <<
    object.center().z() << std::endl; std::cout << " " << pos_x_ego << " " <<
    pos_y_ego <<  std::endl; object.mutable_center()->set_x(pos_x_ego);
    object.mutable_center()->set_y(pos_y_ego);
    object.mutable_center()->set_z(0);
    // convert obu velocity from gps to ego frame
    double obj_head_gps = obu_object.heading() * DEG_TO_RAD; // delta_head
    double x_velociety_global = obu_object.speed() * std::sin(obj_head_gps);
    double y_velociety_global = obu_object.speed() * std::cos(obj_head_gps);
    double host_velocity = std::sqrt(std::pow(localization.longitudinal_v(), 2) +
                                     std::pow(localization.lateral_v(), 2));
    double vel_x_ego = x_velociety_global * cos_host +
                       y_velociety_global * sin_host - host_velocity;
    double vel_y_ego =
        -x_velociety_global * sin_host + y_velociety_global * cos_host;
    std::cout << " ****++++ " << std::endl;
    std::cout << " " << object.velocity().x() << " " << object.velocity().y() <<
    std::endl; std::cout << " " << vel_x_ego << " " << vel_y_ego << std::endl;
    object.mutable_velocity()->set_x(vel_x_ego);
    object.mutable_velocity()->set_y(vel_y_ego);
    // 0~2*PI:North:0, Clockwise => 0~2*PI:East:0, Counterclockwise
    // convert obu angle from gps to utm
    float angle = obu_object.heading();
    auto yaw = M_PI_2 - angle * DEG_TO_RAD;
    if (yaw > M_PI)
      yaw -= 2 * M_PI;
    else if (yaw < -M_PI)
      yaw += 2 * M_PI;

    float angle_temp = yaw - host_yaw_global;
    if (angle_temp > M_PI)
      angle_temp -= 2 * M_PI;
    else if (angle_temp < -M_PI)
      angle_temp += 2 * M_PI;
    std::cout << " ****++++---- " << std::endl;
    std::cout << " " << object.angle() << std::endl;
    std::cout << " " << angle_temp  << std::endl;
    object.set_angle(angle_temp);
    */
    if (!this->ObjectConvertor(object, fusion_object))
        return false;
    // generate contour for obu object
    this->AssginContoursForObuObj(&object);

    fusion_object->yaw = obu_object.yaw();
    fusion_object->host_yaw = localization.yaw();
    fusion_object->host_position << localization.position().x(), localization.position().y(), localization.position().z();
    Eigen::Vector4d center_3d;
    center_3d << object.center().x(), object.center().y(), object.center().z(), 1;
    Eigen::Matrix4d transform_mat_global;
    transVehicle2GlobalMat(localization, transform_mat_global);
    center_3d = transform_mat_global * center_3d;
    fusion_object->position << center_3d.x(), center_3d.y(),center_3d.z();
    fusion_object->velocity << obu_object.velocity().x(), obu_object.velocity().y(), obu_object.velocity().z();
    fusion_object->velocity << obu_object.velocity().x(), obu_object.velocity().y(), obu_object.velocity().z();
    fusion_object->center[0] = center_3d.x() - localization.position().x();
    fusion_object->center[1] = center_3d.y() - localization.position().y();
    fusion_object->center[2] = center_3d.z() - localization.position().z();
    fusion::PointD contour_point;
    fusion::PointD contour_point_utm;
    fusion::PointD contour_point_ego;
    for (size_t i = 0; i < object.contour_size(); ++i) {
        Eigen::Vector4d polygon_3d;
        polygon_3d << object.contour(i).x(), object.contour(i).y(), object.contour(i).z(), 1;
        polygon_3d = transform_mat_global * polygon_3d;
        // abs  polygon in UTM
        contour_point_utm.x = polygon_3d.x();
        contour_point_utm.y = polygon_3d.y();
        contour_point_utm.z = polygon_3d.z();
        fusion_object->polygon_utm.push_back(contour_point_utm);
        // polygon in ego
        contour_point_ego.x = object.contour(i).x();
        contour_point_ego.y = object.contour(i).y();
        contour_point_ego.z = object.contour(i).z();
        fusion_object->polygon_ego.push_back(contour_point_ego);
        // relative polygon in UTM
        polygon_3d << object.contour(i).x(), object.contour(i).y(), object.contour(i).z(), 0;
        polygon_3d = transform_mat_global * polygon_3d;
        contour_point.x = polygon_3d.x();
        contour_point.y = polygon_3d.y();
        contour_point.z = polygon_3d.z();
        fusion_object->polygon.push_back(contour_point);
    }

    fusion_object->obu_supplement.on_use = true;
    fusion_object->obu_supplement.measurement_id = object.id();
    fusion_object->obu_supplement.status_duration = object.obu_supplement().status_duration();
    this->V2xSource2FusionConvertor(obu_object.source(), fusion_object->source);
    this->V2xSource2FusionConvertor(obu_object.source(), fusion_object->obu_supplement.obu_sub_source);
    fusion_object->sub_type = fusion::kType2TypeMap.at(object.type());
    fusion_object->type = fusion::kSubType2TypeMap.at(fusion_object->sub_type);
    fusion_object->type_probs.assign(static_cast<int>(fusion::ObjectType::MAX_OBJECT_TYPE),
                                     (1.0 - fusion_object->confidence) / (static_cast<int>(fusion::ObjectType::MAX_OBJECT_TYPE) - 1));
    fusion_object->type_probs[static_cast<int>(fusion_object->type)] = fusion_object->confidence;

#if DEBUG_OBU
    ROS_INFO("\033[31m object_gps.x:%lf .\033[0m\n", object_gps.x);
    ROS_INFO("\033[31m object_gps.y:%lf.\033[0m\n", object_gps.y);
    ROS_INFO("\033[31m object_utm.x:%lf .\033[0m\n", object_utm.x);
    ROS_INFO("\033[31m object_utm.y:%lf.\033[0m\n", object_utm.y);
    ROS_INFO("\033[31m localization.position().x():%lf .\033[0m\n", localization.position().x());
    ROS_INFO("\033[31m localization.position().y():%lf.\033[0m\n", localization.position().y());
    ROS_INFO("\033[31m localization.yaw():%lf.\033[0m\n", localization.yaw());
    ROS_INFO("\033[31m utm_x_diff:%lf.\033[0m\n", utm_x_diff);
    ROS_INFO("\033[31m utm_y_diff:%lf.\033[0m\n", utm_y_diff);
    ROS_INFO("\033[31m pos_x_ego:%lf .\033[0m\n", pos_x_ego);
    ROS_INFO("\033[31m pos_y_ego:%lf.\033[0m\n", pos_y_ego);
    ROS_INFO("\033[31m vel_x_ego:%lf.\033[0m\n", vel_x_ego);
    ROS_INFO("\033[31m vel_y_ego:%lf.\033[0m\n", vel_y_ego);
    ROS_INFO("\033[31m type:%d.\033[0m\n", object.type());
#endif
    return true;
}

void ObuConvertor::AddObuToPnc(std::list<ObuObjects> obu_objects_list,
                               const localization::Localization& localization,
                               perception::TrackedObjects& output_objects,
                               const v2x_param& v2x_param) {
    if (obu_objects_list.empty()) {
        ROS_ERROR("AddObuobjsToOutputobjs: obu v2n message NOT received.");
        return;
    }
    double timestamp = output_objects.header().stamp().sec() + output_objects.header().stamp().nsec() * 1e-9;
    double stamp = timestamp - 0.0;
    // double obu_ts;
    // perception::ObuObjects obu_objects;
    // for (auto it = obu_objects_list.begin(); it != obu_objects_list.end();
    // it++) {
    //   obu_objects = *it;
    //   obu_ts = obu_objects.header().stamp().sec() +
    //   obu_objects.header().stamp().nsec() * 1e-9;
    // if (obu_ts < stamp)
    //     continue;
    // else
    //     break;
    // }
    // if (abs(stamp - obu_ts) > 0.3) {
    //     ROS_ERROR_STREAM("AddObuobjsToOutputobjs: Time distance "
    //                      << std::setprecision(3) << abs(stamp - obu_ts)
    //                      << " between obu_objects: " << std::setprecision(18)
    //                      << stamp << " and output_objects: "
    //                      << std::setprecision(18) << obu_ts << " is too
    //                      long.");
    //     if (abs(stamp - obu_ts) > 10)
    //         obu_objects_list.clear();
    //     return;
    // }
    perception::ObuObjects obu_objects = obu_objects_list.back();
    double obu_ts = obu_objects.header().stamp().sec() + obu_objects.header().stamp().nsec() * 1e-9;
    if (abs(stamp - obu_ts) > 1.0) {
        obu_objects_list.clear();
        return;
    }
    for (size_t i = 0; i < obu_objects.objs_size(); ++i) {
        perception::ObuObject* obu_object = obu_objects.mutable_objs(i);
        if (obu_object->source() == perception::ObjectSource::V2N_RSI) {
          this->RewriteObuRSIObject(obu_object, localization);
        } else {
          if (v2x_param.v2x_process_mode == 1) {
            continue;
          }
          if (!this->RewriteObuPNTObject(obu_object, localization)) {
            continue;
          }
          // 非全透传模式
          if (v2x_param.v2x_process_mode != 4) {
            if ((v2x_param.v2x_process_mode == 3) && !BeyondRangeMode(*obu_object, v2x_param)) {
              continue;
            }
            if (v2x_param.v2x_process_mode == 2 &&
                !BlindMode(*obu_object, localization, output_objects, v2x_param)) {
              continue;
            }
          }
        }

        TrackedObject* output_object = output_objects.add_objs();
        output_object->set_longitude(obu_object->longitude());
        output_object->set_latitude(obu_object->latitude());
        output_object->set_source(obu_object->source());

        perception::fusion::point object_gps;
        object_gps.x = obu_object->longitude();
        object_gps.y = obu_object->latitude();
        // utm convert to vehicle of obj
        perception::fusion::point object_utm = GPS2MCT(object_gps);
        output_object->set_longitude_p(object_utm.x);
        output_object->set_latitude_p(object_utm.y);

        if (obu_object->source() != perception::ObjectSource::V2N_RSI) {
            output_object->set_alt(obu_object->alt());
            output_object->set_speed(obu_object->speed());
            output_object->set_heading(obu_object->heading());
            output_object->set_yaw(obu_object->yaw());
            output_object->mutable_velocity()->set_x(obu_object->mutable_velocity()->x());
            output_object->mutable_velocity()->set_y(obu_object->mutable_velocity()->y());
            output_object->mutable_velocity()->set_z(obu_object->mutable_velocity()->z());
        }
        perception::Object* obu_obj = obu_object->mutable_obj();
        perception::Object* output_obj = output_object->mutable_obj();
        *output_obj = *obu_obj;
        if (obu_object->source() == perception::ObjectSource::V2N_RSI) {
            geometry::Point* polygon_p;
            for (size_t j = 0; j < output_obj->polygon_size(); ++j) {
                polygon_p = output_object->add_polygon_p();
                polygon_p->set_x(output_obj->polygon(j).x());
                polygon_p->set_y(output_obj->polygon(j).y());
                polygon_p->set_z(output_obj->polygon(j).z());
            }
        }
        output_obj->set_id(output_obj->id() + 86400);  // 86400s = 24hour
        output_obj->mutable_obu_supplement()->set_on_use(true);
        // TODO：需要随着output_convertor.cc：206行而变化
        output_obj->add_match_ids(0);
        output_obj->add_match_ids(0);
        output_obj->add_match_ids(0);
        output_obj->add_match_ids(0);
        output_obj->add_match_ids(0);
        output_obj->add_match_ids(0);
        output_obj->add_match_ids(0);
        output_obj->add_match_ids(0);
        output_obj->add_match_ids(0);
        output_obj->set_match_ids(8, obu_obj->id());
    }
}

void ObuConvertor::Mogo2OBU(perception::TrackedObjects& output_objects, perception::TrackedObjects& output_objects_obu) {
    output_objects_obu.mutable_header()->set_seq(output_objects.header().seq());
    output_objects_obu.mutable_header()->mutable_stamp()->set_sec(output_objects.header().stamp().sec());
    output_objects_obu.mutable_header()->mutable_stamp()->set_nsec(output_objects.header().stamp().nsec());
    output_objects_obu.mutable_header()->set_frame_id(output_objects.header().frame_id());
    output_objects_obu.mutable_header()->set_module_name(output_objects.header().module_name());
    for (size_t i = 0; i < output_objects.objs_size(); ++i) {
        TrackedObject output_object = *output_objects.mutable_objs(i);
        // reset ID
        output_object.mutable_obj()->set_id(i + 1);
        // type conversion
        switch (output_object.obj().type()) {
            case perception::ObjectType::TYPE_UNKNOWN:
                output_object.set_obu_participant_type(0);
                break;
            case perception::ObjectType::TYPE_UNKNOWN_DYNAMIC:
                output_object.set_obu_participant_type(0);
                break;
            case perception::ObjectType::TYPE_UNKNOWN_STATIC:
                output_object.set_obu_participant_type(0);
                break;
            case perception::ObjectType::TYPE_PEDESTRIAN:
                output_object.set_obu_participant_type(3);
                break;
            case perception::ObjectType::TYPE_BICYCLE:
                output_object.set_obu_participant_type(2);
                break;
            case perception::ObjectType::TYPE_CAR:
                output_object.set_obu_participant_type(10);
                break;
            case perception::ObjectType::TYPE_TRUCK:
                output_object.set_obu_participant_type(20);
                break;
            case perception::ObjectType::TYPE_BUS:
                output_object.set_obu_participant_type(50);
                break;
            default:
                output_object.set_obu_participant_type(0);
                break;
        }
        TrackedObject* object_obu = output_objects_obu.add_objs();
        object_obu->CopyFrom(output_object);
    }
}

void ObuConvertor::AssginContoursForObuObj(perception::Object* obj) {
    geometry::Point point[4];
    SetPointLFU(*obj, point[0]);
    SetPointLBU(*obj, point[1]);
    SetPointRBU(*obj, point[2]);
    SetPointRFU(*obj, point[3]);
    double yaw = obj->angle();
    if (abs(yaw) > 0.01) {
        double cx = obj->center().x();
        double cy = obj->center().y();

        double cosYaw = std::cos(yaw);
        double sinYaw = std::sin(yaw);
        double dcx = -cx * cosYaw + cy * sinYaw + cx;
        double dcy = -cx * sinYaw - cy * cosYaw + cy;
        for (size_t i = 0; i < 4; ++i) {
            this->PointRotate(cosYaw, sinYaw, dcx, dcy, point[i]);
        }
    }
    obj->clear_contour();
    geometry::Point* contour_p;
    for (size_t i = 0; i < 4; ++i) {
        contour_p = obj->add_contour();
        contour_p->set_x(point[i].x());
        contour_p->set_y(point[i].y());
        contour_p->set_z(point[i].z());
    }
}

/* l=left  f=front d=down  r=right b=back u=up */
/* lfd lbd  rfd rbd   lfu lbu  rfu rbu*/
void ObuConvertor::SetPointLFD(const perception::Object& object, geometry::Point& point) {
    point.set_x(object.center().x() - object.size().x() / 2.);
    point.set_y(object.center().y() - object.size().y() / 2.);
    point.set_z(object.center().z() - object.size().z() / 2.);
    point.set_z(0.);
}
void ObuConvertor::SetPointLBD(const perception::Object& object, geometry::Point& point) {
    point.set_x(object.center().x() - object.size().x() / 2.);
    point.set_y(object.center().y() + object.size().y() / 2.);
    point.set_z(object.center().z() - object.size().z() / 2.);
    point.set_z(0.);
}
void ObuConvertor::SetPointRFD(const perception::Object& object, geometry::Point& point) {
    point.set_x(object.center().x() + object.size().x() / 2.);
    point.set_y(object.center().y() - object.size().y() / 2.);
    point.set_z(object.center().z() - object.size().z() / 2.);
    point.set_z(0.);
}
void ObuConvertor::SetPointRBD(const perception::Object& object, geometry::Point& point) {
    point.set_x(object.center().x() + object.size().x() / 2.);
    point.set_y(object.center().y() + object.size().y() / 2.);
    point.set_z(object.center().z() - object.size().z() / 2.);
    point.set_z(0.);
}
void ObuConvertor::SetPointLFU(const perception::Object& object, geometry::Point& point) {
    point.set_x(object.center().x() - object.size().x() / 2.);
    point.set_y(object.center().y() - object.size().y() / 2.);
    point.set_z(object.center().z() + object.size().z() / 2.);
}
void ObuConvertor::SetPointLBU(const perception::Object& object, geometry::Point& point) {
    point.set_x(object.center().x() - object.size().x() / 2.);
    point.set_y(object.center().y() + object.size().y() / 2.);
    point.set_z(object.center().z() + object.size().z() / 2.);
}
void ObuConvertor::SetPointRFU(const perception::Object& object, geometry::Point& point) {
    point.set_x(object.center().x() + object.size().x() / 2.);
    point.set_y(object.center().y() - object.size().y() / 2.);
    point.set_z(object.center().z() + object.size().z() / 2.);
}
void ObuConvertor::SetPointRBU(const perception::Object& object, geometry::Point& point) {
    point.set_x(object.center().x() + object.size().x() / 2.);
    point.set_y(object.center().y() + object.size().y() / 2.);
    point.set_z(object.center().z() + object.size().z() / 2.);
}

void ObuConvertor::PointRotate(const double cosYaw, const double sinYaw, const double dcx, const double dcy, geometry::Point& point) {
    double px = point.x();
    double py = point.y();
    point.set_x(px * cosYaw - py * sinYaw + dcx);
    point.set_y(px * sinYaw + py * cosYaw + dcy);
}

void ObuConvertor::V2xSource2FusionConvertor(const perception::ObjectSource& mogo_source, perception::fusion::ObjectSource& fusion_source) {
    switch (mogo_source) {
        case perception::ObjectSource::V2V_BSM:
            fusion_source = perception::fusion::ObjectSource::V2V_BSM;
            break;
        case perception::ObjectSource::V2I_RSM:
            fusion_source = perception::fusion::ObjectSource::V2I_RSM;
            break;
        case perception::ObjectSource::V2V_SSM:
            fusion_source = perception::fusion::ObjectSource::V2V_SSM;
            break;
        case perception::ObjectSource::V2N_RSM:
            fusion_source = perception::fusion::ObjectSource::V2N_RSM;
            break;
        case perception::ObjectSource::V2N_RSI:
            fusion_source = perception::fusion::ObjectSource::V2N_RSI;
            break;
        case perception::ObjectSource::V2I_SSM:
            fusion_source = perception::fusion::ObjectSource::V2I_SSM;
            break;
        default:
            fusion_source = perception::fusion::ObjectSource::UNKNOWN;
            break;
    }
}

void ObuConvertor::StoreV2xObjs(perception::ObuObjects& obu_objects,
                                const localization::Localization& localization,
                                std::list<localization::Localization> global_localizations,
                                std::list<ObuObjects>& obu_objects_v2n_,
                                std::list<ObuObjects>& obu_objects_v2i_) {
    perception::ObuObjects obu_objects_v2n;
    obu_objects_v2n.mutable_header()->set_seq(obu_objects.header().seq());
    obu_objects_v2n.mutable_header()->mutable_stamp()->set_sec(obu_objects.header().stamp().sec());
    obu_objects_v2n.mutable_header()->mutable_stamp()->set_nsec(obu_objects.header().stamp().nsec());
    obu_objects_v2n.mutable_header()->set_frame_id(obu_objects.header().frame_id());
    obu_objects_v2n.mutable_header()->set_module_name(obu_objects.header().module_name());

    perception::ObuObjects obu_objects_v2i;
    obu_objects_v2i.mutable_header()->set_seq(obu_objects.header().seq());
    obu_objects_v2i.mutable_header()->mutable_stamp()->set_sec(obu_objects.header().stamp().sec());
    obu_objects_v2i.mutable_header()->mutable_stamp()->set_nsec(obu_objects.header().stamp().nsec());
    obu_objects_v2i.mutable_header()->set_frame_id(obu_objects.header().frame_id());
    obu_objects_v2i.mutable_header()->set_module_name(obu_objects.header().module_name());

    bool has_v2n_data = false;
    bool has_v2i_data = false;
    double obu_timestamp = obu_objects.header().stamp().sec() + obu_objects.header().stamp().nsec() * 1e-9;
    for (size_t i = 0; i < obu_objects.objs_size(); ++i) {
        perception::ObuObject* obu_object = obu_objects.mutable_objs(i);
        if (obu_object->source() == perception::ObjectSource::V2N_RSI) {
            obu_object->mutable_obj()->set_time_stamp(obu_timestamp);
            RewriteObuRSIObject(obu_object, localization);
            perception::ObuObject* obu_object_v2n = obu_objects_v2n.add_objs();
            *obu_object_v2n = *obu_object;
            has_v2n_data = true;
        }
        if (obu_object->source() == perception::ObjectSource::V2N_RSM) {
            obu_object->mutable_obj()->set_time_stamp(obu_timestamp);
            if (!RewriteObuPNTObject(obu_object, localization))
                continue;
            perception::ObuObject* obu_object_v2n = obu_objects_v2n.add_objs();
            *obu_object_v2n = *obu_object;
            has_v2n_data = true;
        }
        if (obu_object->source() == perception::ObjectSource::V2I_SSM) {
            double timestamp_obj = obu_object->mutable_obj()->time_stamp();
            obu_object->mutable_obj()->set_time_stamp(timestamp_obj);
            localization::Localization localization;
            if (!QueryNearestLoc(timestamp_obj, global_localizations, localization)) {
                ROS_WARN_STREAM("StoreV2xObjs: Fail to get localization for obu object timestamp.");
                // continue;
            }
            if (!RewriteObuPNTObject(obu_object, localization))
                continue;
            perception::ObuObject* obu_object_v2i = obu_objects_v2i.add_objs();
            *obu_object_v2i = *obu_object;
            has_v2i_data = true;
        }
    }
    if (has_v2n_data) {
        obu_objects_v2n_.push_back(obu_objects_v2n);
    }

    if (has_v2i_data) {
        obu_objects_v2i_.push_back(obu_objects_v2i);
    }
}

bool ObuConvertor::QueryNearestLoc(const double& timestamp,
                                   std::list<localization::Localization> global_localizations,
                                   localization::Localization& localization) {
    if (global_localizations.empty()) {
        ROS_ERROR("QueryNearestLoc: Localization message NOT received.");
        return false;
    }
    // reduce timestamp by time delay of sensor data transmission and perception
    // consuming. now 0.0
    double stamp = timestamp - 0.0;
    double loc_ts;
    for (auto it = global_localizations.begin(); it != global_localizations.end(); it++) {
        localization = *it;
        loc_ts = localization.header().stamp().sec() + localization.header().stamp().nsec() * 1e-9;
        if (loc_ts < stamp)
            continue;
        else
            break;
    }
    if (abs(stamp - loc_ts) > 0.3) {
        ROS_ERROR_STREAM("Time distance " << std::setprecision(3) << abs(stamp - loc_ts)
                                          << " between obu objects: " << std::setprecision(18) << stamp
                                          << " and localization: " << std::setprecision(18) << loc_ts << " is too long.");
        return false;
    }
    return true;
}
bool ObuConvertor::RewriteObuPNTObject(perception::ObuObject* obu_object_ptr, const localization::Localization& localization) {
    perception::Object* obu_obj = obu_object_ptr->mutable_obj();
    this->RewriteObuObject(obu_object_ptr, localization);
    this->WGS84Angle2Mct(obu_object_ptr, localization);
    this->AssginContoursForObuObj(obu_obj);
    return true;
}
void ObuConvertor::RewriteObuRSIObject(perception::ObuObject* obu_object_ptr, const localization::Localization& localization) {
    perception::Object* obu_obj = obu_object_ptr->mutable_obj();
    RewriteObuObject(obu_object_ptr, localization);
    geometry::Point rte_polygon;
    geometry::Point* contour_p;
    double cos_host = std::cos(localization.yaw());
    double sin_host = std::sin(localization.yaw());
    int rte_size = obu_obj->polygon_size();
    obu_obj->clear_contour();
    for (size_t i = 0; i < rte_size; ++i) {
        rte_polygon = obu_obj->polygon(i);
        perception::fusion::point ponit_gps;
        ponit_gps.x = rte_polygon.x();
        ponit_gps.y = rte_polygon.y();
        perception::fusion::point ponit_utm = GPS2MCT(ponit_gps);
        double polygon_utm_x_diff = ponit_utm.x - localization.position().x();
        double polygon_utm_y_diff = ponit_utm.y - localization.position().y();
        double polygon_pos_x_ego = polygon_utm_x_diff * cos_host + polygon_utm_y_diff * sin_host;
        double polygon_pos_y_ego = -polygon_utm_x_diff * sin_host + polygon_utm_y_diff * cos_host;
        contour_p = obu_obj->add_contour();
        contour_p->set_x(polygon_pos_x_ego);
        contour_p->set_y(polygon_pos_y_ego);
        contour_p->set_z(0);
    }
}
void ObuConvertor::RewriteObuObject(perception::ObuObject* obu_object_ptr, const localization::Localization& localization) {
    perception::Object* obu_obj = obu_object_ptr->mutable_obj();
    double my_host_vx = localization.longitudinal_v();
    double my_host_vy = localization.lateral_v();
    double host_yaw_global = localization.yaw();  // 东0，北90，西180，南270
    double cos_host = std::cos(host_yaw_global);
    double sin_host = std::sin(host_yaw_global);

    perception::fusion::point object_gps;
    object_gps.x = obu_object_ptr->longitude();
    object_gps.y = obu_object_ptr->latitude();
    // utm convert to vehicle of obj
    perception::fusion::point object_utm = GPS2MCT(object_gps);
    double utm_x_diff = object_utm.x - localization.position().x();
    double utm_y_diff = object_utm.y - localization.position().y();
    // Debug(@liuxinyu): temporarity
    double pos_x_ego = utm_x_diff * cos_host + utm_y_diff * sin_host;
    double pos_y_ego = -utm_x_diff * sin_host + utm_y_diff * cos_host;
    double pos_z_ego = obu_object_ptr->alt() - localization.position().z();
    ROS_DEBUG_STREAM("alt: " << obu_object_ptr->alt() << " " << localization.position().z());
    obu_obj->mutable_center()->set_x(pos_x_ego);
    obu_obj->mutable_center()->set_y(pos_y_ego);
    obu_obj->mutable_center()->set_z(0);
    obu_obj->set_x_distance(pos_x_ego);
    obu_obj->set_y_distance(pos_y_ego);
    // convert obu velocity from gps to ego frame
    // 0~2*PI:North:0, Clockwise => 0~2*PI:East:0, Counterclockwise
    double obj_head_gps = obu_object_ptr->heading() * DEG_TO_RAD;  // delta_head
    double x_velociety_global = obu_object_ptr->speed() * std::sin(obj_head_gps);
    double y_velociety_global = obu_object_ptr->speed() * std::cos(obj_head_gps);
    obu_object_ptr->mutable_velocity()->set_x(x_velociety_global);
    obu_object_ptr->mutable_velocity()->set_y(y_velociety_global);
    obu_object_ptr->mutable_velocity()->set_z(0);
    double host_velocity = std::sqrt(std::pow(my_host_vx, 2) + std::pow(my_host_vy, 2));
    double vel_x_ego = x_velociety_global * cos_host + y_velociety_global * sin_host - host_velocity;
    double vel_y_ego = -x_velociety_global * sin_host + y_velociety_global * cos_host;
    obu_obj->mutable_velocity()->set_x(vel_x_ego);
    obu_obj->mutable_velocity()->set_y(vel_y_ego);
    obu_obj->mutable_velocity()->set_z(0);
    obu_obj->set_confidence(1.0);
}

void ObuConvertor::WGS84Angle2Mct(perception::ObuObject* obu_object_ptr, const localization::Localization& localization) {
    perception::Object* obu_obj = obu_object_ptr->mutable_obj();
    // 0~2*PI:North:0, Clockwise => 0~2*PI:East:0, Counterclockwise
    // convert obu angle from gps to utm
    float angle = obu_object_ptr->heading();
    auto yaw = M_PI_2 - angle * DEG_TO_RAD;
    if (yaw > M_PI)
        yaw -= 2 * M_PI;
    else if (yaw < -M_PI)
        yaw += 2 * M_PI;
    obu_object_ptr->set_yaw(yaw);
    // convert obu angle from global to vehicle
    float angle_temp = obu_object_ptr->yaw() - localization.yaw();
    if (angle_temp > M_PI)
        angle_temp -= 2 * M_PI;
    else if (angle_temp < -M_PI)
        angle_temp += 2 * M_PI;
    obu_obj->set_angle(angle_temp);
}

bool ObuConvertor::BeyondRangeMode(const perception::ObuObject obu_object,
                                   const v2x_param& v2x_param) {
    if (abs(obu_object.obj().center().x()) < v2x_param.beyond_vis_range &&
        abs(obu_object.obj().center().y()) < v2x_param.beyond_vis_range) {
        return false;
    }
    return true;
}

bool ObuConvertor::BlindMode(const perception::ObuObject obu_object,
                             const localization::Localization& localization,
                             perception::TrackedObjects& output_objects,
                             const v2x_param& v2x_param) {
    // 自车过滤
    if (abs(obu_object.obj().center().x()) < v2x_param.radius_for_fusion_object &&
        abs(obu_object.obj().center().y()) < v2x_param.radius_for_fusion_object) {
        return false;
    }
    double obu_object_x = obu_object.obj().center().x();
    double obu_object_y = obu_object.obj().center().y();
    double distance_min = 1000.0;
    size_t fusion_object_id = -1;
    for (size_t m = 0; m < output_objects.objs_size(); ++m) {
      if (output_objects.objs(m).obj().has_obu_supplement()) {
        continue;
      }
      double fusion_object_x = output_objects.objs(m).obj().center().x();
      double fusion_object_y = output_objects.objs(m).obj().center().y();
      double distance = std::sqrt(std::pow(obu_object_x - fusion_object_x, 2) +
                                  std::pow(obu_object_y - fusion_object_y, 2));
      if (distance < distance_min) {
        distance_min = distance;
        fusion_object_id = m;
      }
    }
    if (distance_min < v2x_param.radius_for_fusion_object) {
      return false;
    }
    if (fusion_object_id != -1 &&
        (output_objects.objs(fusion_object_id).obj().type() == perception::ObjectType::TYPE_BUS ||
         output_objects.objs(fusion_object_id).obj().type() ==
             perception::ObjectType::TYPE_TRUCK)) {
      if (distance_min < v2x_param.radius_for_fusion_object + 3 &&
          output_objects.objs(fusion_object_id).obj().type() == obu_object.obj().type()) {
        return false;
      }
    }
    // std::map<double, size_t> objects_distance;
    // double distance_gap = 3.0;  // units of 1 m
    // // 考虑中心点距离最近，不一定是同一个障碍物。
    // auto start = ros::Time::now().toSec();
    // for (size_t m = 0; m < output_objects.objs_size(); ++m) {
    //     perception::Object output_obj = output_objects.objs(m).obj();
    //     if (!output_obj.has_lidar_supplement() && output_obj.has_obu_supplement()) {
    //         continue;
    //     }
    //     if (output_obj.type() == perception::ObjectType::TYPE_UNKNOWN ||
    //         output_obj.type() == perception::ObjectType::TYPE_UNKNOWN_DYNAMIC ||
    //         output_obj.type() == perception::ObjectType::TYPE_UNKNOWN_STATIC) {
    //         continue;
    //     }
    //     double fusion_object_x = output_obj.center().x();
    //     double fusion_object_y = output_obj.center().y();
    //     double distance =
    //         std::hypot(obu_object_x - fusion_object_x, obu_object_y - fusion_object_y);
    //     if (distance < v2x_param.radius_for_fusion_object + distance_gap) {
    //         std::map<double, size_t>::iterator iter = objects_distance.find(distance);
    //         if (iter != objects_distance.end())
    //             (objects_distance)[distance + 0.0001] = m;
    //         else
    //             (objects_distance)[distance] = m;
    //     }
    // }
    // auto end = ros::Time::now().toSec();  
    // std::cout << "TIME-COST:Find Nearnest Distance:" <<std::setprecision(18)<< (end - start)*1000<<" ms" << std::endl;
    // // 针对以fusion障碍物位置为圆心，半径为?m筛选的obu障碍物，再次检查IOU
    // for (auto it = objects_distance.begin(); it != objects_distance.end(); ++it) {
    //     perception::TrackedObject* output_object = output_objects.mutable_objs(it->second);
    //     perception::Object* output_obj = output_object->mutable_obj();
    //     float iou = ComputeObjectPolygonIouDistance(obu_object, localization, *output_object);
    //     if (iou > 0.0 && obu_object.obj().type() == output_obj->type()) {
    //         return false;
    //     }
    // }
    return true;
}
    float ObuConvertor::ComputeObjectPolygonIouDistance(
        const perception::ObuObject obu_object, const localization::Localization& localization,
        const perception::TrackedObject& fusion_object) {
        if (fusion_object.contour_p_size() ==0 ) {
            ROS_WARN("ComputeObjectPolygonIouDistance: fusion object contour size is null.");
            return -1;
        }
        auto start = ros::Time::now().toSec();
        Eigen::Matrix4d transform_mat_l2g;
        transVehicle2GlobalMat(localization, transform_mat_l2g);
        geos::geom::GeometryFactory::Ptr factory = geos::geom::GeometryFactory::create();
        std::vector<geos::geom::Coordinate> coordinates_fusion;
        for (size_t i = 0; i < fusion_object.contour_p_size(); ++i) {
            coordinates_fusion.push_back(
                geos::geom::Coordinate(fusion_object.contour_p(i).x(), fusion_object.contour_p(i).y()));
        }
        coordinates_fusion.push_back(
            geos::geom::Coordinate(fusion_object.contour_p(0).x(), fusion_object.contour_p(0).y()));
        std::unique_ptr<geos::geom::LinearRing> fusion_lr =
            factory->createLinearRing(std::move(coordinates_fusion));

        std::vector<geos::geom::Coordinate> coordinates_obu;
        Eigen::Vector4d point_contour_obu_first;
        for (size_t j = 0; j < obu_object.obj().contour_size(); ++j) {
            Eigen::Vector4d point_contour_obu;
            point_contour_obu << obu_object.obj().contour(j).x(), obu_object.obj().contour(j).y(),
                obu_object.obj().contour(j).z(), 1;
            point_contour_obu = transform_mat_l2g * point_contour_obu;
            coordinates_obu.push_back(
                geos::geom::Coordinate(point_contour_obu.x(), point_contour_obu.y()));
            if (j == 0) point_contour_obu_first = point_contour_obu;
        }
        coordinates_obu.push_back(
            geos::geom::Coordinate(point_contour_obu_first.x(), point_contour_obu_first.y()));
        std::unique_ptr<geos::geom::LinearRing> obu_lr =
            factory->createLinearRing(std::move(coordinates_obu));

        std::unique_ptr<geos::geom::Polygon> fusion_poly =
            factory->createPolygon(std::move(fusion_lr));
        std::unique_ptr<geos::geom::Polygon> obu_poly = factory->createPolygon(std::move(obu_lr));
        std::unique_ptr<geos::geom::Geometry> inter = fusion_poly->intersection(obu_poly.get());
        double inter_area = inter->getArea();
        double iou = inter_area / (fusion_poly->getArea() + obu_poly->getArea() - inter_area);
        auto end = ros::Time::now().toSec();
        std::cout << "TIME-COST:ComputeObjectPolygonIouDistance:" <<std::setprecision(18)<< (end - start)*1000<<" ms" << std::endl;
        return iou;
    }

}  // namespace fusion
}  // namespace perception
