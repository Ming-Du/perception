
#include "track_pool_types.h"

#include "common/include/log.h"

namespace perception {
namespace fusion {

// @brief call pool instance once to initialize memory
__attribute__((constructor)) void FusionPoolInitialize() {
  TrackPool::Instance();
  ROS_DEBUG("FusionPoolInitialize: Initialize FusionPool.");
}

}  // namespace fusion
}  // namespace perception
