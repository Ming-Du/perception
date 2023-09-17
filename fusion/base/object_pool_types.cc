
#include "object_pool_types.h"

#include "common/include/log.h"

namespace perception {
namespace fusion {

// @brief call pool instance once to initialize memory
__attribute__((constructor)) void PoolInitialize() {
  ObjectPool::Instance();
  PointFCloudPool::Instance();
  PointDCloudPool::Instance();
  FramePool::Instance();
#ifndef PERCEPTION_FUSION_BASE_DISABLE_POOL
  ROS_DEBUG("PoolInitialize: Initialize base object pool (no-malloc).");
#else
  ROS_DEBUG("PoolInitialize: Initialize base object pool (malloc).");
#endif
}

}  // namespace fusion
}  // namespace perception
