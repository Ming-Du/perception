#include "radar_convertor.h"

namespace perception {
namespace fusion {

bool RadarConvertor::Mogo2Fusion(const perception::RadarObject& radar_object,
                                 const localization::Localization& localization,
                                 fusion::ObjectPtr& fusion_object) {
    perception::Object object = radar_object.obj();
    if (!this->ObjectConvertor(object, fusion_object)) {
        return false;
    }
    double host_yaw_global = localization.yaw();
    double host_velocity = std::sqrt(std::pow(localization.longitudinal_v(), 2) + std::pow(localization.lateral_v(), 2));
    Eigen::Matrix4d transform_mat_ego, transform_mat_global;
    transGlobal2VehicleMat(localization, transform_mat_ego);
    transVehicle2GlobalMat(localization, transform_mat_global);

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
    fusion_object->position[0] = position_3d.x();
    fusion_object->position[1] = position_3d.y();
    fusion_object->position[2] = position_3d.z();
    // relative position in UTM
    center_3d = transform_mat_global * center_3d;
    fusion_object->center[0] = center_3d.x();
    fusion_object->center[1] = center_3d.y();
    fusion_object->center[2] = center_3d.z();

    // abs velocity in UTM
    Eigen::Vector4d velocity_3d;
    velocity_3d << object.velocity().x() + host_velocity, object.velocity().y(), 0, 0;
    velocity_3d = transform_mat_global * velocity_3d;
    fusion_object->velocity[0] = velocity_3d.x();
    fusion_object->velocity[1] = velocity_3d.y();
    fusion_object->velocity[2] = velocity_3d.z();

    //abs acceleration in UTM
    Eigen::Vector4d acceleration_3d;
    acceleration_3d << object.acceleration().x() + localization.longitudinal_a(), object.acceleration().y() + localization.lateral_a(),  localization.vertical_a(), 0;
    acceleration_3d = transform_mat_global * acceleration_3d;
    fusion_object->acceleration[0] = acceleration_3d.x();
    fusion_object->acceleration[1] = acceleration_3d.y();
    fusion_object->acceleration[2] = acceleration_3d.z();

    fusion_object->acceleration_ego[0] = object.acceleration().x() + localization.longitudinal_a();
    fusion_object->acceleration_ego[1] = object.acceleration().y() + localization.lateral_a();
    fusion_object->acceleration_ego[2] = object.acceleration().z() + localization.vertical_a();

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

    // variance of center  velocity(need modify)
    double init_center_variance_ = 4.0;
    double init_velocity_variance_ = 0.1;
    double init_acceleration_variance_ = 1000.0;
    fusion_object->center_uncertainty = Eigen::Matrix3f::Identity() * init_center_variance_;
    fusion_object->velocity_uncertainty = Eigen::Matrix3f::Identity() * init_velocity_variance_;
    fusion_object->acceleration_uncertainty = Eigen::Matrix3f::Identity() * init_acceleration_variance_;

    fusion_object->radar_supplement.on_use = true;
    fusion_object->radar_supplement.range = sqrt(pow(object.x_distance(), 2) + pow(object.y_distance(), 2));
    fusion_object->radar_supplement.angle = object.angle();
    fusion_object->radar_supplement.relative_radial_velocity = object.radar_supplement().relative_vel_x();
    fusion_object->radar_supplement.relative_tangential_velocity = object.radar_supplement().relative_vel_y();
    // Modify(@liuxinyu): Distinguish front and rear radar
    fusion_object->radar_supplement.is_rear = object.radar_supplement().is_rear();
    return true;
}
}  // namespace fusion
}  // namespace perception