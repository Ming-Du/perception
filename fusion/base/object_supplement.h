#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <boost/circular_buffer.hpp>

#include "box.h"
#include "object_types.h"
#include "perception/common/include/eigen_defs.h"
#include "point_cloud.h"

namespace perception {
namespace fusion {

struct alignas(16) LidarObjectSupplement {
  void Reset() {
    is_orientation_ready = false;
    on_use = false;
    cloud.clear();
    cloud_world.clear();
    is_fp = false;
    fp_prob = 0.f;
    is_background = false;
    is_in_roi = false;
    num_points_in_roi = 0;
    height_above_ground = std::numeric_limits<float>::max();
    raw_probs.clear();
    raw_classification_methods.clear();
  }
  // @brief orientation estimateed indicator
  bool is_orientation_ready = false;
  // @brief valid only for on_use = true
  bool on_use = false;
  // @brief cloud of the object in lidar coordinates
  perception::fusion::AttributePointCloud<PointF> cloud;
  // @brief cloud of the object in world coordinates
  perception::fusion::AttributePointCloud<PointD> cloud_world;
  // @brief background indicator
  bool is_background = false;
  // @brief false positive indicator
  bool is_fp = false;
  // @brief false positive probability
  float fp_prob = 0.f;
  // @brief whether this object is in roi
  bool is_in_roi = false;
  // @brief number of cloud points in roi
  size_t num_points_in_roi = 0;
  // @brief object height above ground
  float height_above_ground = std::numeric_limits<float>::max();

  // @brief raw probability of each classification method
  std::vector<std::vector<float>> raw_probs;
  std::vector<std::string> raw_classification_methods;
};
typedef std::shared_ptr<LidarObjectSupplement> LidarObjectSupplementPtr;
typedef std::shared_ptr<const LidarObjectSupplement> LidarObjectSupplementConstPtr;

struct alignas(16) RadarObjectSupplement {
  void Reset() {
    on_use = false;
    range = 0.0f;
    angle = 0.0f;
    relative_radial_velocity = 0.0f;
    relative_tangential_velocity = 0.0f;
    radial_velocity = 0.0f;
    is_rear = false;
  }
  // @brief valid only for on_use = true
  bool on_use = false;
  /* @brief (range, angle) in radar polar coordinate system
     x for forward and y for left
  */
  float range = 0.0f;
  float angle = 0.0f;

  float relative_radial_velocity = 0.0f;
  float relative_tangential_velocity = 0.0f;
  float radial_velocity = 0.0f;
  bool is_rear = false;
};

typedef std::shared_ptr<RadarObjectSupplement> RadarObjectSupplementPtr;
typedef std::shared_ptr<const RadarObjectSupplement> RadarObjectSupplementConstPtr;

struct alignas(16) CameraObjectSupplement {
  CameraObjectSupplement() { Reset(); }

  void Reset() {
    on_use = false;
    has_3d = false;
    sensor_name.clear();
    local_track_id = -1;
    pts8.clear();
    object_feature.clear();
    alpha = 0.0;
    box = BBox2D<float>();
    projected_box = BBox2D<float>();
    front_box = BBox2D<float>();
    back_box = BBox2D<float>();
    local_center = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    visual_type = VisualObjectType::MAX_OBJECT_TYPE;
    visual_type_probs.resize(static_cast<int>(VisualObjectType::MAX_OBJECT_TYPE), 0);

    area_id = 0;
    visible_ratios[0] = visible_ratios[1] = 0;
    visible_ratios[2] = visible_ratios[3] = 0;
    cut_off_ratios[0] = cut_off_ratios[1] = 0;
    cut_off_ratios[2] = cut_off_ratios[3] = 0;
  }

  // @brief valid only for on_use = true
  bool on_use = false;

  // 是否填充了3D信息
  bool has_3d = false;

  // @brief camera sensor name
  std::string sensor_name;

  // @brief  2d box
  BBox2D<float> box;

  // @brief projected 2d box
  BBox2D<float> projected_box;

  // @brief local track id
  int local_track_id = -1;

  // @brief 2Dto3D, pts8.resize(16), x, y...
  std::vector<float> pts8;

  // @brief front box
  BBox2D<float> front_box;

  // @brief back box
  BBox2D<float> back_box;
  std::vector<float> object_feature;

  // @brief alpha angle from KITTI: Observation angle of object, in [-pi..pi]
  double alpha = 0.0;
  double truncated_horizontal = 0.0;
  double truncated_vertical = 0.0;
  // @brief center in camera coordinate system
  Eigen::Vector3f local_center = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

  // @brief visual object type, only used in camera module
  VisualObjectType visual_type = VisualObjectType::MAX_OBJECT_TYPE;
  std::vector<float> visual_type_probs;

  //----------------------------------------------------------------
  //    area ID, corner ID and face ID
  //----------------------------------------------------------------
  //    8 | 1 | 2       a
  //    ---------    0-----1   ^
  //      |   |       |   |    |
  //    7 | 0 | 3    d|   |b
  //      |   |       |   |
  //    ---------    3-----2
  //    6 | 5 | 4       c
  //----------------------------------------------------------------
  int area_id;
  // @brief visible ratios
  float visible_ratios[4];
  // @brief cut off ratios on width, length (3D)
  //        cut off ratios on left, right (2D)
  float cut_off_ratios[4];
};
typedef std::shared_ptr<CameraObjectSupplement> CameraObjectSupplementPtr;
typedef std::shared_ptr<const CameraObjectSupplement> CameraObjectSupplementConstPtr;

// vidar has same struct with camera
typedef std::shared_ptr<CameraObjectSupplement> VidarObjectSupplementPtr;
typedef std::shared_ptr<const CameraObjectSupplement> VidarObjectSupplementConstPtr;

typedef Eigen::Matrix4f MotionType;
struct alignas(16) VehicleStatus {
  float roll_rate = 0;
  float pitch_rate = 0;
  float yaw_rate = 0;
  float velocity = 0;
  float velocity_x = 0;
  float velocity_y = 0;
  float velocity_z = 0;
  double time_ts = 0;                          // time stamp
  double time_d = 0;                           // time stamp difference in image
  MotionType motion = MotionType::Identity();  // Motion Matrix
};

typedef boost::circular_buffer<VehicleStatus> MotionBuffer;
typedef std::shared_ptr<MotionBuffer> MotionBufferPtr;
typedef std::shared_ptr<const MotionBuffer> MotionBufferConstPtr;

struct alignas(16) Vehicle3DStatus {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  float yaw_delta;  // azimuth angle change
  float pitch_delta;
  float roll_delta;
  float velocity_x;          // east
  float velocity_y;          // north
  float velocity_z;          // up
  double time_ts;            // time stamp
  double time_d;             // time stamp difference in image
  Eigen::Matrix4f motion3d;  // 3-d Motion Matrix
};

typedef boost::circular_buffer<Vehicle3DStatus> Motion3DBuffer;
typedef std::shared_ptr<Motion3DBuffer> Motion3DBufferPtr;
typedef std::shared_ptr<const Motion3DBuffer> Motion3DBufferConstPtr;

struct SensorObjectMeasurement {
  void Reset() {
    sensor_id = "unknonw_sensor";
    timestamp = 0.0;
    track_id = -1;
    center = Eigen::Vector3d(0, 0, 0);
    theta = 0.0f;
    size = Eigen::Vector3f(0, 0, 0);
    velocity = Eigen::Vector3f(0, 0, 0);
    type = ObjectType::UNKNOWN;
    box = BBox2D<float>();
  }

  std::string sensor_id = "unknown_sensor";
  double timestamp = 0.0;
  int track_id = -1;
  Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);
  float theta = 0.0f;
  Eigen::Vector3f size = Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f velocity = Eigen::Vector3f(0, 0, 0);
  ObjectType type = ObjectType::UNKNOWN;
  // @brief only for camera measurement
  BBox2D<float> box;
};

struct alignas(16) FusionObjectSupplement {
  FusionObjectSupplement() { measurements.reserve(5); }
  void Reset() {
    on_use = false;
    measurements.clear();
  }
  bool on_use = false;
  std::vector<SensorObjectMeasurement> measurements;
};

// modify @jiangnan:add solid state lidar
// struct alignas(16) FalconLidarObjectSupplement {
//   void Reset() {
//     is_orientation_ready = false;
//     on_use = false;
//     cloud.clear();
//     cloud_world.clear();
//     is_fp = false;
//     fp_prob = 0.f;
//     is_background = false;
//     is_in_roi = false;
//     num_points_in_roi = 0;
//     height_above_ground = std::numeric_limits<float>::max();
//     raw_probs.clear();
//     raw_classification_methods.clear();
//   }
//   // @brief orientation estimateed indicator
//   bool is_orientation_ready = false;
//   // @brief valid only for on_use = true
//   bool on_use = false;
//   // @brief cloud of the object in lidar coordinates
//   perception::fusion::AttributePointCloud<PointF> cloud;
//   // @brief cloud of the object in world coordinates
//   perception::fusion::AttributePointCloud<PointD> cloud_world;
//   // @brief background indicator
//   bool is_background = false;
//   // @brief false positive indicator
//   bool is_fp = false;
//   // @brief false positive probability
//   float fp_prob = 0.f;
//   // @brief whether this object is in roi
//   bool is_in_roi = false;
//   // @brief number of cloud points in roi
//   size_t num_points_in_roi = 0;
//   // @brief object height above ground
//   float height_above_ground = std::numeric_limits<float>::max();
//
//   // @brief raw probability of each classification method
//   std::vector<std::vector<float>> raw_probs;
//   std::vector<std::string> raw_classification_methods;
// };
typedef std::shared_ptr<LidarObjectSupplement> FalconLidarObjectSupplementPtr;
typedef std::shared_ptr<const LidarObjectSupplement> FalconLidarObjectSupplementConstPtr;

struct alignas(16) ObuObjectSupplement {
  void Reset() {
    on_use = false;
    range = 0.0f;
    angle = 0.0f;
    relative_radial_velocity = 0.0f;
    relative_tangential_velocity = 0.0f;
    radial_velocity = 0.0f;
    measurement_id = 0;
    obu_sub_source = ObjectSource::UNKNOWN;
    status_duration = 0.0;
  }
  // @brief valid only for on_use = true
  bool on_use = false;
  /* @brief (range, angle) in obu polar coordinate system
     x for forward and y for left
  */
  float range = 0.0f;
  float angle = 0.0f;

  float relative_radial_velocity = 0.0f;
  float relative_tangential_velocity = 0.0f;
  float radial_velocity = 0.0f;
  int measurement_id = 0;
  ObjectSource obu_sub_source;
  double status_duration = 0;
};

typedef std::shared_ptr<ObuObjectSupplement> ObuObjectSupplementPtr;
typedef std::shared_ptr<const ObuObjectSupplement> ObuObjectSupplementConstPtr;

}  // namespace fusion
}  // namespace perception
