#pragma once

#include <memory>
#include <string>

#include "Eigen/Core"
#include "gtest/gtest_prod.h"

#include "base/base_forward_declaration.h"
#include "object.h"
#include "common/proto/sensor_meta.pb.h"

namespace perception {
namespace fusion {

class SensorObject {
 public:
  SensorObject() = delete;

  explicit SensorObject(const std::shared_ptr<const Object>& object_ptr);

  SensorObject(const std::shared_ptr<const Object>& object_ptr,
               const std::shared_ptr<const SensorFrameHeader>& frame_header);

  SensorObject(const std::shared_ptr<const Object>& object_ptr,
               const std::shared_ptr<SensorFrame>& frame_ptr);

  // Getter
  // @brief get frame timestamp which might be different with object timestamp
  double GetTimestamp() const;
  bool GetRelatedFramePose(Eigen::Affine3d* pose) const;

  std::string GetSensorId() const;
  base::SensorType GetSensorType() const;

  inline std::shared_ptr<const Object> GetBaseObject() const { return object_; }

  inline double GetInvisiblePeriod() const { return invisible_period_; }

  inline void SetInvisiblePeriod(double period) { invisible_period_ = period; }

 private:
  FRIEND_TEST(SensorObjectTest, test);

  std::shared_ptr<const Object> object_;
  double invisible_period_ = 0.0;
  std::shared_ptr<const SensorFrameHeader> frame_header_ = nullptr;
};

typedef std::shared_ptr<SensorObject> SensorObjectPtr;
typedef std::shared_ptr<const SensorObject> SensorObjectConstPtr;

class FusedObject {
 public:
  FusedObject();
  ~FusedObject() = default;

  inline double GetTimestamp() const { return object_->latest_tracked_time; }

  inline std::shared_ptr<Object> GetBaseObject() { return object_; }

 private:
  std::shared_ptr<Object> object_;
};

typedef std::shared_ptr<FusedObject> FusedObjectPtr;

bool IsLidar(const SensorObjectConstPtr& obj);
bool IsRadar(const SensorObjectConstPtr& obj);
bool IsCamera(const SensorObjectConstPtr& obj);
//  Modify @jiangnan:add falcon lidar
bool IsFalconLidar(const SensorObjectConstPtr& obj);
// Modify(@liuxinyu): obu_test
bool IsObu(const SensorObjectConstPtr& obj);
bool IsVidar(const SensorObjectConstPtr& obj);  // add ming.du

}  // namespace fusion
}  // namespace perception
