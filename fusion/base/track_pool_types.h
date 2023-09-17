#pragma once

#include <ros/ros.h>
#include "base/track.h"
#include "concurrent_object_pool.h"

namespace perception {
namespace fusion {

struct TrackInitializer {
  void operator()(Track* track) const { track->Reset(); }
};

static const size_t kTrackPoolSize = 3000;

// pool typedef collections,
// developer should add pool type to the PoolInitialize function in .cc file
typedef ConcurrentObjectPool<Track, kTrackPoolSize, TrackInitializer> TrackPool;
}  // namespace fusion
}  // namespace perception
