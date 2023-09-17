#pragma once

#include <map>
#include <string>
#include <common/proto/object.pb.h>
namespace perception {
namespace fusion {

// @brief general object type
enum class ObjectType {
  UNKNOWN = 0,
  PEDESTRIAN = 1,
  BICYCLE = 2,
  MOTOR = 3,
  CAR = 4,
  BUS = 5,
  TRUCK = 6,
  VEGETATION = 7,
  TRAFFICCONE = 8,
  MAX_OBJECT_TYPE = 9,
};

// @brief internal object type used by visual perception
enum class VisualObjectType {
  CAR,
  VAN,
  BUS,
  TRUCK,
  BICYCLE,
  TRICYCLE,
  PEDESTRIAN,
  TRAFFICCONE,
  UNKNOWN_MOVABLE,
  UNKNOWN_UNMOVABLE,
  MOTOR,
  MAX_OBJECT_TYPE,
};

// @brief fine-grained object types
enum class ObjectSubType {
  UNKNOWN = 0,
  PEDESTRIAN = 1,
  BICYCLE = 2,
  MOTOR = 3,
  RIDER = 4,
  CAR = 5,
  TRUCK = 6,
  BUS = 7,
  TRAIN = 8,
  SIGN = 9,
  LIGHT = 10,
  RED = 11,
  GREEN = 12,
  YELLOW = 13,
  BLACK = 14,
  TRIANGLEROADBLOCK = 15,
  WARNINGTRIANGLE = 16,
  VEGETATION = 17,
  TRICYCLE = 18,
  BICYCLE_RIDER = 19,
  TRICYCLE_RIDER = 20,
  UNKNOWN_SMALL = 21,
  UNKNOWN_BIG = 22,
  UNKNOWN_STATIC = 23,
  UNKNOWN_DYNAMIC = 24,
  ROADWORK_OCCUPY_0501 = 25,
  ROADWORK_BREAK_0502 = 26,
  MOTOR_RIDER =27,
  MAX_OBJECT_TYPE = 28,
};

// @brief motion state
enum class MotionState {
  UNKNOWN = 0,
  MOVING = 1,
  STATIONARY = 2,
};
  enum class NoiseState {
  NOISE_OBJECT = 0,
  NOISE_NOISE = 1,
  NOISE_SUSPECTED = 2,
  NOISE_FLOWERBEDS = 3
};

/**
 * v2x object source
 */
enum class ObjectSource {
  UNKNOWN = 0,
  V2V_BSM = 1,
  V2I_RSM = 2,
  V2V_SSM = 3,
  V2N_RSM = 4,
  V2N_RSI = 5,
  V2I_SSM = 6,
};

const std::map<perception::ObjectType, ObjectSubType> kType2TypeMap = {
    {perception::ObjectType::TYPE_UNKNOWN, ObjectSubType::UNKNOWN},
    {perception::ObjectType::TYPE_PEDESTRIAN,ObjectSubType::PEDESTRIAN},
    {perception::ObjectType::TYPE_BICYCLE,ObjectSubType::BICYCLE},
    {perception::ObjectType::TYPE_MOTOR,ObjectSubType::MOTOR},
    {perception::ObjectType::TYPE_RIDER,ObjectSubType::RIDER},
    {perception::ObjectType::TYPE_CAR,ObjectSubType::CAR},
    {perception::ObjectType::TYPE_TRUCK,ObjectSubType::TRUCK},
    {perception::ObjectType::TYPE_BUS,ObjectSubType::BUS},
    {perception::ObjectType::TYPE_TRAIN,ObjectSubType::TRAIN},

    {perception::ObjectType::TYPE_SIGN,ObjectSubType::SIGN},
    {perception::ObjectType::TYPE_LIGHT,ObjectSubType::LIGHT},
    {perception::ObjectType::TYPE_RED,ObjectSubType::RED},
    {perception::ObjectType::TYPE_GREEN,ObjectSubType::GREEN},
    {perception::ObjectType::TYPE_YELLOW,ObjectSubType::YELLOW},
    {perception::ObjectType::TYPE_BLACK,ObjectSubType::BLACK},

    {perception::ObjectType::TYPE_TRIANGLEROADBLOCK,ObjectSubType::TRIANGLEROADBLOCK},
    {perception::ObjectType::TYPE_WARNINGTRIANGLE,ObjectSubType::WARNINGTRIANGLE},
    {perception::ObjectType::TYPE_VEGETATION,ObjectSubType::VEGETATION},
    {perception::ObjectType::TYPE_TRICYCLE,ObjectSubType::TRICYCLE},
    {perception::ObjectType::TYPE_BICYCLE_RIDER,ObjectSubType::BICYCLE_RIDER},
    {perception::ObjectType::TYPE_TRICYCLE_RIDER,ObjectSubType::TRICYCLE_RIDER},
    {perception::ObjectType::TYPE_UNKNOWN_SMALL,ObjectSubType::UNKNOWN_SMALL},
    {perception::ObjectType::TYPE_UNKNOWN_BIG,ObjectSubType::UNKNOWN_BIG},
    {perception::ObjectType::TYPE_UNKNOWN_STATIC,ObjectSubType::UNKNOWN_STATIC},
    {perception::ObjectType::TYPE_UNKNOWN_DYNAMIC,ObjectSubType::UNKNOWN_DYNAMIC},    
};

/**
 * ObjectSubType mapping
 */
const std::map<ObjectSubType, ObjectType> kSubType2TypeMap = {
    {ObjectSubType::UNKNOWN, ObjectType::UNKNOWN},
    {ObjectSubType::UNKNOWN_DYNAMIC, ObjectType::UNKNOWN},
    {ObjectSubType::UNKNOWN_STATIC, ObjectType::UNKNOWN},
    {ObjectSubType::UNKNOWN_BIG, ObjectType::UNKNOWN},
    {ObjectSubType::UNKNOWN_SMALL, ObjectType::UNKNOWN},
    {ObjectSubType::RIDER, ObjectType::BICYCLE},
    {ObjectSubType::CAR, ObjectType::CAR},
    {ObjectSubType::TRUCK, ObjectType::TRUCK},
    {ObjectSubType::BUS, ObjectType::BUS},
    {ObjectSubType::BICYCLE, ObjectType::BICYCLE},
    {ObjectSubType::MOTOR, ObjectType::MOTOR},
    {ObjectSubType::TRAIN, ObjectType::TRUCK},
    {ObjectSubType::TRIANGLEROADBLOCK, ObjectType::TRAFFICCONE},
    {ObjectSubType::WARNINGTRIANGLE, ObjectType::TRAFFICCONE},
    {ObjectSubType::TRICYCLE, ObjectType::BICYCLE},
    {ObjectSubType::PEDESTRIAN, ObjectType::PEDESTRIAN},
    {ObjectSubType::BICYCLE_RIDER, ObjectType::BICYCLE},
    {ObjectSubType::VEGETATION, ObjectType::VEGETATION},
    {ObjectSubType::TRICYCLE_RIDER, ObjectType::BICYCLE},
    {ObjectSubType::MAX_OBJECT_TYPE, ObjectType::MAX_OBJECT_TYPE},
};
}  // namespace fusion
}  // namespace perception
