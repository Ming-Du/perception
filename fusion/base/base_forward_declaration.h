#pragma once

#include <memory>

namespace perception {
namespace fusion {

struct SensorFrameHeader;
typedef std::shared_ptr<SensorFrameHeader> SensorFrameHeaderPtr;
typedef std::shared_ptr<const SensorFrameHeader> SensorFrameHeaderConstPtr;

class SensorFrame;
typedef std::shared_ptr<SensorFrame> SensorFramePtr;
typedef std::shared_ptr<const SensorFrame> SensorFrameConstPtr;

class Sensor;
typedef std::shared_ptr<Sensor> SensorPtr;
typedef std::shared_ptr<const Sensor> SensorConstPtr;

}  // namespace fusion
}  // namespace perception
