#include "lidar_convertor.h"

namespace perception {
namespace fusion {

bool LidarConvertor::Mogo2Fusion(const perception::TrackedObject& lidar_object,
                                 const localization::Localization& localization,
                                 fusion::ObjectPtr& fusion_object,
                                  input_sensor_ input_sensor) {
    perception::Object object = lidar_object.obj();

    double host_yaw_global = localization.yaw();
    Eigen::Matrix4d transform_mat_ego, transform_mat_global;

    transGlobal2VehicleMat(localization, transform_mat_ego);
    transVehicle2GlobalMat(localization, transform_mat_global);

    Eigen::Vector4d velocity_3d;
    double host_velocity = std::sqrt(std::pow(localization.longitudinal_v(), 2) + std::pow(localization.lateral_v(), 2));
    velocity_3d << lidar_object.velocity().x(), lidar_object.velocity().y(), lidar_object.velocity().z(), 0;
    velocity_3d = transform_mat_ego * velocity_3d;

    if (!this->ObjectConvertor(object, fusion_object)) {
        return false;
    }
    // ego yaw ,position in UTM
    fusion_object->host_yaw = host_yaw_global;
    fusion_object->host_position[0] = localization.position().x();
    fusion_object->host_position[1] = localization.position().y();
    fusion_object->host_position[2] = localization.position().z();
    Eigen::Vector4d position_3d, center_3d;
    position_3d << object.center().x(), object.center().y(), object.center().z(), 1;
    center_3d << object.center().x(), object.center().y(), object.center().z(), 0;
    // abs position in UTM
    position_3d = transform_mat_global * position_3d;
    fusion_object->position[0] = lidar_object.longitude_p();
    fusion_object->position[1] = lidar_object.latitude_p();
    fusion_object->position[2] = position_3d.z();
    // abs velocity in UTM
    fusion_object->velocity[0] = lidar_object.velocity().x();
    fusion_object->velocity[1] = lidar_object.velocity().y();
    // relative position in UTM
    center_3d = transform_mat_global * center_3d;
    fusion_object->center[0] = lidar_object.longitude_p() - localization.position().x();
    fusion_object->center[1] = lidar_object.latitude_p() - localization.position().y();
    fusion_object->center[2] = center_3d.z();
    // object yaw in UTM
    fusion_object->yaw = lidar_object.yaw();
    // computer polygon
    fusion::PointD contour_point;
    fusion::PointD contour_point_utm;
    fusion::PointD contour_point_ego;
    if(0 == object.contour_size()){
        return false;
    }
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
    
    // Modify @jiangnan : GroupType: Group 1  Single 0;
    if (lidar_object.obj().group_type() == perception::GroupType::TYPE_GROUP) {
      fusion_object->track_id = lidar_object.obj().id() + 10000;
      fusion_object->match_id_lidar = lidar_object.obj().id();
      fusion_object->group_type = 1;
    }

    fusion_object->status = lidar_object.obj().road_status();
    fusion_object->noise_state = (int) lidar_object.obj().noise_state();
    fusion_object->confidence = object.exist_confidence();
    if (lidar_object.obj().type() == perception::ObjectType::TYPE_UNKNOWN ||
        lidar_object.obj().type() == perception::ObjectType::TYPE_VEGETATION) {
        fusion_object->is_lidar_rb = true;
    }
    //  when type is  UNKNOWN  ,set  type_confidence  0.1
    if (lidar_object.obj().type() == perception::ObjectType::TYPE_UNKNOWN) {
      fusion_object->confidence = 0.2;
    }

    fusion_object->sub_type = fusion::kType2TypeMap.at(object.type());
    fusion_object->type = fusion::kSubType2TypeMap.at(fusion_object->sub_type);
    fusion_object->type_probs.assign(static_cast<int>(fusion::ObjectType::MAX_OBJECT_TYPE),
                                     (1.0 - fusion_object->confidence) / (static_cast<int>(fusion::ObjectType::MAX_OBJECT_TYPE) - 1));
    fusion_object->type_probs[static_cast<int>(fusion_object->type)] = fusion_object->confidence;

    if (input_sensor == 1) {
        fusion_object->lidar_supplement.on_use = true;
        fusion_object->lidar_supplement.is_background = false;
        fusion_object->lidar_supplement.is_in_roi = true;
    } else if(input_sensor == 2){
        fusion_object->falcon_lidar_supplement.on_use = true;
        fusion_object->falcon_lidar_supplement.is_background = false;
        fusion_object->falcon_lidar_supplement.is_in_roi = true;
        fusion_object->is_falcon = true;
    }
    return true;
}
}  // namespace fusion
}  // namespace perception