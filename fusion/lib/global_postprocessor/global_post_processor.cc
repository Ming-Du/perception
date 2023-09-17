#include "global_post_processor.h"
#include "common/global_util.h"
#include "common/gps_proj.h"

namespace perception {
namespace fusion {
const double RAD_TO_DEG = 180.0 / M_PI;
const double DEG_TO_RAD = M_PI / 180.0;

void GlobalPostProcessor::ComputeGlobalState(TrackedObjects& objects,
                                             localization::Localization& global_odom) {
  // traversal objects
  for (int i = 0; i < objects.objs_size(); i++) {
    TrackedObject* track_object_ptr = objects.mutable_objs(i);
    Object* object_ptr = track_object_ptr->mutable_obj();
    ConvertLocalToGlobal(track_object_ptr, global_odom);
  }
}

void GlobalPostProcessor::ConvertLocalToGlobal(TrackedObject* tracked_object_ptr,
                                               const localization::Localization localization) {
  std::shared_ptr<TrackedObject> tracker_object_src_ptr(new TrackedObject());
  tracker_object_src_ptr->CopyFrom(*tracked_object_ptr);
  Object* object_src_ptr = tracker_object_src_ptr->mutable_obj();
  Object* object_ptr = tracked_object_ptr->mutable_obj();
  // ego velocity  always positive
  double host_velocity =
      std::sqrt(std::pow(localization.longitudinal_v(), 2) + std::pow(localization.lateral_v(), 2));
  double my_host_heading = M_PI_2 - localization.yaw();
  double xy[2] = {object_src_ptr->x_distance(), object_src_ptr->y_distance()};
  double host_altitude = localization.altitude();
  double host_utm_z = localization.position().z();
  double my_utm_gps_x = localization.position().x();
  double my_utm_gps_y = localization.position().y();
  double host_yaw_global = localization.yaw();
  double cos_host = std::cos(host_yaw_global);
  double sin_host = std::sin(host_yaw_global);

  // Modify @jiangnan; computer  ego_global_velocity(UTM->ego)
  double vel_x_ego_global =
      tracked_object_ptr->velocity().x() * cos_host + tracked_object_ptr->velocity().y() * sin_host;
  double vel_y_ego_global = -tracked_object_ptr->velocity().x() * sin_host +
                            tracked_object_ptr->velocity().y() * cos_host;
  tracked_object_ptr->set_absolute_longitude_v(vel_x_ego_global);
  tracked_object_ptr->set_absolute_lateral_v(vel_y_ego_global);

  // object heading
  double object_yaw_local = object_src_ptr->angle();
  // double  object_yaw_global = host_yaw_global + object_yaw_local;
  double object_yaw_global = tracked_object_ptr->yaw();

  // //0~2*PI->East:0, Counterclockwise
  // if(object_yaw_global > 2*M_PI)
  //   object_yaw_global -= 2*M_PI;
  // else if(object_yaw_global < 0)
  //   object_yaw_global += 2*M_PI;
  // tracked_object_ptr->set_yaw(object_yaw_global);

  // get obstacle longitude and latitude position (in meter)
  double p_angle = UtmDeviation(&my_utm_gps_x, &my_utm_gps_y, my_host_heading, xy[0], xy[1]);
  tracked_object_ptr->set_longitude_p(my_utm_gps_x);
  tracked_object_ptr->set_latitude_p(my_utm_gps_y);
  // position transform from meter to degree
  UtmTransformGps(my_utm_gps_x, my_utm_gps_y, host_utm_z);
  tracked_object_ptr->set_longitude(my_utm_gps_x);
  tracked_object_ptr->set_latitude(my_utm_gps_y);
  tracked_object_ptr->set_alt(host_altitude);

  // Modify(@liuxinyu): get speed(m/s) and heading(in degree)
  double speed = std::sqrt(std::pow(tracked_object_ptr->velocity().x(), 2) +
                           std::pow(tracked_object_ptr->velocity().y(), 2));
  tracked_object_ptr->set_speed(speed);
  double heading = M_PI / 2 - object_yaw_global;
  if (heading < 0)
    heading += 2 * M_PI;
  tracked_object_ptr->set_heading(heading);
}

double GlobalPostProcessor::UtmDeviation(double* host_utm_x,
                                         double* host_utm_y,
                                         const double host_heading,
                                         const double distance_x,
                                         const double distance_y) {
  double dist = std::sqrt(distance_x * distance_x + distance_y * distance_y);
  double p_angle = host_heading - std::atan2(distance_y, distance_x);
  *host_utm_x += dist * std::sin(p_angle);
  *host_utm_y += dist * std::cos(p_angle);
  return p_angle;
}

}  // end of namespace fusion
}  // end of namespace perception
