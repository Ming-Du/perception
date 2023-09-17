#pragma once

#include <memory>
#include <vector>

#include "Eigen/Core"

namespace perception {
namespace fusion {

struct alignas(16) ImpendingCollisionEdge {
  // edge id per frame
  int id = 0;

  // age of the tracked object
  double tracking_time = 0.0;

  // points of the edge
  std::vector<Eigen::Vector3d> points;
};

// TODO(all): to remove
// typedef std::shared_ptr<ImpendingCollisionEdge> ImpendingCollisionEdgePtr;
// typedef std::shared_ptr<const ImpendingCollisionEdge>
//     ImpendingCollisionEdgeConstPtr;

// Sensor single frame objects.
struct ImpendingCollisionEdges {
  double timestamp = 0.0;

  // edges
  std::vector<std::shared_ptr<ImpendingCollisionEdge>> impending_collision_edges;

  // sensor to world position
  Eigen::Matrix4d sensor2world_pose = Eigen::Matrix4d::Zero();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// TODO(all): to remove
// typedef std::shared_ptr<ImpendingCollisionEdges> ImpendingCollisionEdgesPtr;
// typedef std::shared_ptr<const ImpendingCollisionEdges>
//    ImpendingCollisionEdgesConstPtr;

}  // namespace fusion
}  // namespace perception
