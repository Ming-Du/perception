#pragma once

#include <memory>

#include "blob.h"
#include "image.h"
#include "impending_collision_edge.h"
#include "point_cloud.h"

namespace perception {
namespace fusion {
// sensor-specific frame supplements: Lidar, Radar, Camera, Obu
struct alignas(16) LidarFrameSupplement {
  // @brief valid only when on_use = true
  bool on_use = false;

  // @brief only reference of the original cloud in lidar coordinate system
  std::shared_ptr<AttributePointCloud<PointF>> cloud_ptr;

  void Reset() {
    on_use = false;
    cloud_ptr = nullptr;
  }
};

typedef std::shared_ptr<LidarFrameSupplement> LidarFrameSupplementPtr;
typedef std::shared_ptr<const LidarFrameSupplement> LidarFrameSupplementConstPtr;

struct alignas(16) RadarFrameSupplement {
  // @brief valid only when on_use = true
  bool on_use = false;
  void Reset() { on_use = false; }
};
typedef std::shared_ptr<RadarFrameSupplement> RadarFrameSupplementPtr;
typedef std::shared_ptr<const RadarFrameSupplement> RadarFrameSupplementConstPtr;

struct alignas(16) ObuFrameSupplement {
  // @brief valid only when on_use = true
  bool on_use = false;
  void Reset() { on_use = false; }
};
typedef std::shared_ptr<ObuFrameSupplement> ObuFrameSupplementPtr;
typedef std::shared_ptr<const ObuFrameSupplement> ObuFrameSupplementConstPtr;

struct alignas(16) CameraFrameSupplement {
  // @brief valid only when on_use = true
  bool on_use = false;

  // @brief only reference of the image data
  Image8UPtr image_ptr = nullptr;

  // TODO(guiyilin): modify interfaces of visualizer, use Image8U
  std::shared_ptr<Blob<uint8_t>> image_blob = nullptr;

  void Reset() {
    on_use = false;
    image_ptr = nullptr;
    image_blob = nullptr;
  }
};

typedef std::shared_ptr<CameraFrameSupplement> CameraFrameSupplementPtr;
typedef std::shared_ptr<const CameraFrameSupplement> CameraFrameSupplementConstPtr;

// vidar use camera supplement
typedef std::shared_ptr<CameraFrameSupplement> VidarFrameSupplementPtr;
typedef std::shared_ptr<const CameraFrameSupplement> VidarFrameSupplementConstPtr;

struct alignas(16) UltrasonicFrameSupplement {
  // @brief valid only when on_use = true
  bool on_use = false;

  // @brief only reference of the image data
  std::shared_ptr<ImpendingCollisionEdges> impending_collision_edges_ptr;

  void Reset() {
    on_use = false;
    impending_collision_edges_ptr = nullptr;
  }
};

typedef std::shared_ptr<UltrasonicFrameSupplement> UltrasonicFrameSupplementPtr;
typedef std::shared_ptr<const UltrasonicFrameSupplement> UltrasonicFrameSupplementConstPtr;

// Modify @jiangnan :add falcon lidar
// struct alignas(16) FalconLidarFrameSupplement {
//   // @brief valid only when on_use = true
//   bool on_use = false;
//   // @brief only reference of the original cloud in lidar coordinate system
//   std::shared_ptr<AttributePointCloud<PointF>> cloud_ptr;
//
//   void Reset() {
//     on_use = false;
//     cloud_ptr = nullptr;
//   }
// };
typedef std::shared_ptr<LidarFrameSupplement> FalconLidarFrameSupplementPtr;
typedef std::shared_ptr<const LidarFrameSupplement> FalconLidarFrameSupplementConstPtr;

}  // namespace fusion
}  // namespace perception
