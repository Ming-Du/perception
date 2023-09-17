#pragma once

namespace perception {
namespace fusion {

enum class VEHICLE {
  TAXI = 0,
  M1 = 1,
  BUS = 2,
  SWEEPER = 3,
  M2 = 4
};

struct CarLight {
  float brake_visible = 0.0f;
  float brake_switch_on = 0.0f;
  float left_turn_visible = 0.0f;
  float left_turn_switch_on = 0.0f;
  float right_turn_visible = 0.0f;
  float right_turn_switch_on = 0.0f;

  void Reset() {
    brake_visible = 0;
    brake_switch_on = 0;
    left_turn_visible = 0;
    left_turn_switch_on = 0;
    right_turn_visible = 0;
    right_turn_switch_on = 0;
  }
};

}  // namespace fusion
}  // namespace perception
