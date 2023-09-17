#include "common/global_util.h"

namespace perception {
namespace fusion {

geometry_msgs::Point UTM2FLU(geometry::Point3D global_pos,
                             geometry::Point global_host,
                             double heading) {
  heading -= 0.058;
  geometry_msgs::Point local_pos;
  local_pos.x = std::cos(heading) * (global_pos.x() - global_host.x()) +
                std::sin(heading) * (global_pos.y() - global_host.y());
  local_pos.y = std::cos(heading + M_PI_2) * (global_pos.x() - global_host.x()) +
                std::sin(heading + M_PI_2) * (global_pos.y() - global_host.y());
  return local_pos;
}

geometry_msgs::Point UTM2FLU(geometry::Point global_pos,
                             geometry::Point global_host,
                             double heading) {
  heading -= 0.058;
  geometry_msgs::Point local_pos;
  local_pos.x = std::cos(heading) * (global_pos.x() - global_host.x()) +
                std::sin(heading) * (global_pos.y() - global_host.y());
  local_pos.y = std::cos(heading + M_PI_2) * (global_pos.x() - global_host.x()) +
                std::sin(heading + M_PI_2) * (global_pos.y() - global_host.y());
  return local_pos;
}

Eigen::Vector2d FLU2UTM(geometry::Point3D local_pos, double heading) {
  heading -= 0.058;
  return Eigen::Vector2d(
      std::cos(heading) * local_pos.x() + std::cos(heading + M_PI_2) * local_pos.y(),
      std::sin(heading) * local_pos.x() + std::sin(heading + M_PI_2) * local_pos.y());
}

}  // namespace fusion
}  // namespace perception