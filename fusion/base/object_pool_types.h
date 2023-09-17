#pragma once

#include "concurrent_object_pool.h"
#include "frame.h"
#include "object.h"
#include "point_cloud.h"

namespace perception {
namespace fusion {

struct ObjectInitializer {
  void operator()(Object* object) const { object->Reset(); }
};

template <typename T>
struct PointCloudInitializer {
  void operator()(AttributePointCloud<Point<T>>* cloud) const { cloud->clear(); }
};

struct FrameInitializer {
  void operator()(Frame* frame) const { frame->Reset(); }
};

static const size_t kObjectPoolSize = 10000;
static const size_t kPointCloudPoolSize = 1000;
static const size_t kFramePoolSize = 100;

using ObjectPool = ConcurrentObjectPool<Object, kObjectPoolSize, ObjectInitializer>;
using PointFCloudPool = ConcurrentObjectPool<AttributePointCloud<PointF>,
                                             kPointCloudPoolSize,
                                             PointCloudInitializer<float>>;
using PointDCloudPool = ConcurrentObjectPool<AttributePointCloud<PointD>,
                                             kPointCloudPoolSize,
                                             PointCloudInitializer<double>>;
using FramePool = ConcurrentObjectPool<Frame, kFramePoolSize, FrameInitializer>;

}  // namespace fusion
}  // namespace perception
