#include "visual_convertor.h"

namespace perception {
namespace fusion {

bool VisualConvertor::Mogo2Fusion(const perception::TrackedObject& vidar_object,
                                  const localization::Localization& localization,
                                  fusion::ObjectPtr& fusion_object,
                                   input_sensor_ input_sensor) {
    perception::Object object = vidar_object.obj();

    double host_yaw_global = localization.yaw();
    Eigen::Matrix4d transform_mat_ego, transform_mat_global;

    transGlobal2VehicleMat(localization, transform_mat_ego);
    transVehicle2GlobalMat(localization, transform_mat_global);

    if (!this->ObjectConvertor(object, fusion_object)) {
        return false;
    }
    // ego yaw ,position in UTM
    fusion_object->host_yaw = host_yaw_global;
    fusion_object->host_position[0] = localization.position().x();
    fusion_object->host_position[1] = localization.position().y();
    fusion_object->host_position[2] = localization.position().z();
    // abs velocity in UTM
    fusion_object->velocity[0] = vidar_object.velocity().x();
    fusion_object->velocity[1] = vidar_object.velocity().y();
    fusion_object->velocity[2] = vidar_object.velocity().z();
    // object yaw in UTM
    fusion_object->yaw = vidar_object.yaw();

    Eigen::Vector4d position_3d, center_3d;
    position_3d << object.center().x(), object.center().y(), object.center().z(), 1;
    center_3d << object.center().x(), object.center().y(), object.center().z(), 0;
    // abs position in UTM
    position_3d = transform_mat_global * position_3d;
    fusion_object->position[0] = position_3d.x();
    fusion_object->position[1] = position_3d.y();
    fusion_object->position[2] = position_3d.z();
    // relative position in UTM
    center_3d = transform_mat_global * center_3d;
    fusion_object->center[0] = center_3d.x();
    fusion_object->center[1] = center_3d.y();
    fusion_object->center[2] = center_3d.z();

    // computer polygon
    fusion::PointD contour_point;
    fusion::PointD contour_point_utm;
    fusion::PointD contour_point_ego;
    for (size_t i = 0; i < object.polygon_size(); ++i) {
        Eigen::Vector4d polygon_3d;
        polygon_3d << object.polygon(i).x(), object.polygon(i).y(), object.polygon(i).z(), 1;
        polygon_3d = transform_mat_global * polygon_3d;
        // abs  polygon in UTM
        contour_point_utm.x = polygon_3d.x();
        contour_point_utm.y = polygon_3d.y();
        contour_point_utm.z = polygon_3d.z();
        fusion_object->polygon_utm.push_back(contour_point_utm);
        // polygon in ego
        contour_point_ego.x = object.polygon(i).x();
        contour_point_ego.y = object.polygon(i).y();
        contour_point_ego.z = object.polygon(i).z();
        fusion_object->polygon_ego.push_back(contour_point_ego);
        // relative polygon in UTM
        polygon_3d << object.polygon(i).x(), object.polygon(i).y(), object.polygon(i).z(), 0;
        polygon_3d = transform_mat_global * polygon_3d;
        contour_point.x = polygon_3d.x();
        contour_point.y = polygon_3d.y();
        contour_point.z = polygon_3d.z();
        fusion_object->polygon.push_back(contour_point);
    }

    fusion_object->vidar_supplement.on_use = true;
    // fusion_object->vidar_supplement.is_background = false;
    // fusion_object->vidar_supplement.is_in_roi = true;
    fusion_object->sub_type = fusion::kType2TypeMap.at(object.type());
    fusion_object->type = fusion::kSubType2TypeMap.at(fusion_object->sub_type);
    fusion_object->type_probs.assign(static_cast<int>(fusion::ObjectType::MAX_OBJECT_TYPE),
                                     (1.0 - fusion_object->confidence) / (static_cast<int>(fusion::ObjectType::MAX_OBJECT_TYPE) - 1));
    fusion_object->type_probs[static_cast<int>(fusion_object->type)] = fusion_object->confidence;
    return true;
}

bool VisualConvertor::Mogo2Fusion(const perception::VisualObject& camera_object,
                                  const localization::Localization& localization,
                                  fusion::ObjectPtr& fusion_object) {
    perception::Object object = camera_object.obj();
    if (object.id() == 0xffffffff) {
        // ROS_DEBUG_DEBUG(5,"objectCamera2ApolloConvertor: Visual object received
        // not tracked.");
        return false;
    }
    if (std::isnan(object.camera_supplement().box().xmin()) || std::isnan(object.camera_supplement().box().xmax()) ||
        std::isnan(object.camera_supplement().box().ymin()) || std::isnan(object.camera_supplement().box().ymax())) {
        ROS_WARN_THROTTLE(5, "objectCamera2ApolloConvertor: Camera box vertices are nan.");
        return false;
    }
    fusion_object->id = object.id();
    fusion_object->track_id = object.id();
    fusion_object->theta = 1.57;  // no angle from camera
    // Now convert from 2D to 3D with camera intrinsic
    auto& box = object.camera_supplement().box();
    Eigen::Vector2d position;
    const float car_w = 2.0 * base::track_object_types.at(perception::ObjectType::TYPE_CAR)[0];  // car width
    const float car_l = 2.0 * base::track_object_types.at(perception::ObjectType::TYPE_CAR)[1];  // car length
    const float car_h = 2.3;                                                                     // car length
    if (!calculate_car_position(object.sensor_name(), box, position, car_w, car_l)) {
        return false;
    }
    fusion_object->center_ego << position.x(), position.y(), 0.5 * car_h;
    fusion_object->size << car_l, car_w, car_h;
    fusion_object->anchor_point = fusion_object->center;  // Modify-guoxiaoxiao
    fusion_object->confidence = object.confidence();
    fusion_object->tracking_time = object.tracking_time();
    fusion_object->acceleration << object.acceleration().x(), object.acceleration().y(), object.acceleration().z();

    double host_yaw_global = localization.yaw();
    double cos_host = std::cos(host_yaw_global);
    double sin_host = std::sin(host_yaw_global);
    double host_velocity = std::sqrt(std::pow(localization.longitudinal_v(), 2) + std::pow(localization.lateral_v(), 2));

    fusion_object->host_yaw = host_yaw_global;
    fusion_object->center[0] = fusion_object->center_ego[0] * cos_host - fusion_object->center_ego[1] * sin_host;
    fusion_object->center[1] = fusion_object->center_ego[0] * sin_host + fusion_object->center_ego[1] * cos_host;
    fusion_object->center[2] = fusion_object->center_ego[2];
    fusion_object->host_position[0] = localization.position().x();
    fusion_object->host_position[1] = localization.position().y();
    fusion_object->host_position[2] = localization.position().z();
    fusion_object->position[0] = fusion_object->center[0] + localization.position().x();
    fusion_object->position[1] = fusion_object->center[1] + localization.position().y();
    fusion_object->velocity[0] = (object.velocity().x() + host_velocity) * cos_host - object.velocity().y() * sin_host;
    fusion_object->velocity[1] = (object.velocity().x() + host_velocity) * sin_host + object.velocity().y() * cos_host;
    fusion::PointD polygon_center;
    polygon_center.x = fusion_object->center[0];
    polygon_center.y = fusion_object->center[1];
    polygon_center.z = object.center().z();
    fusion_object->polygon.push_back(polygon_center);

    // Modify-guoxiaoxiao
    //subTypeMogo2ApolloConvertor(object.sensor_name(), object.type(), fusion_object->sub_type);
    fusion_object->sub_type = fusion::kType2TypeMap.at(object.type());
    fusion_object->type = fusion::kSubType2TypeMap.at(fusion_object->sub_type);
    fusion_object->type_probs.assign(static_cast<int>(fusion::ObjectType::MAX_OBJECT_TYPE),
                                     (1.0 - fusion_object->confidence) / (static_cast<int>(fusion::ObjectType::MAX_OBJECT_TYPE) - 1));
    fusion_object->type_probs[static_cast<int>(fusion_object->type)] = fusion_object->confidence;

    fusion_object->camera_supplement.box.xmin = object.camera_supplement().box().xmin();
    fusion_object->camera_supplement.box.ymin = object.camera_supplement().box().ymin();
    fusion_object->camera_supplement.box.xmax = object.camera_supplement().box().xmax();
    fusion_object->camera_supplement.box.ymax = object.camera_supplement().box().ymax();
    fusion_object->camera_supplement.on_use = true;
    return true;
}
}  // namespace fusion
}  // namespace perception