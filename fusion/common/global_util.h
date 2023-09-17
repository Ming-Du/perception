#include "Eigen/Core"
#include "common/proto/geometry.pb.h"
#include "geometry_msgs/Point.h"

namespace perception {
namespace fusion {

geometry_msgs::Point UTM2FLU(geometry::Point3D global_pos,
                             geometry::Point global_host,
                             double heading);

geometry_msgs::Point UTM2FLU(geometry::Point global_pos,
                             geometry::Point global_host,
                             double heading);

Eigen::Vector2d FLU2UTM(geometry::Point3D local_pos, double heading);

}  // namespace fusion
}  // namespace perception