#include "tracker_optimizer.h"
#include "math.h"

namespace perception {
namespace fusion {
// Add by jiangnan :tracker optimizer
tracker_optimizer::tracker_optimizer() {}
tracker_optimizer::~tracker_optimizer() {}

// Add by jiangnan :tracker optimizer
void tracker_optimizer::Init() {
  window_size_ = 6;
  position_threshold = 1.5;
  static_count = 0;
  motion_count = 0;
  static_position = Eigen::Vector3d(0, 0, 0);

  if (!history_position.empty()) {
    history_position.clear();
    history_velocity.clear();
  }
}

void tracker_optimizer::StoreHistoryTrajectory(const Eigen::Vector3d& position,
                                               const Eigen::Vector3f& velocity) {
  if (history_position.size() < window_size_) {
    history_position.emplace_back(position);
    history_velocity.emplace_back(velocity);
  } else {
    history_position.erase(history_position.begin());
    history_velocity.erase(history_velocity.begin());
    history_position.emplace_back(position);
    history_velocity.emplace_back(velocity);
  }
}

bool tracker_optimizer::CheckMotionStateIsStatic() {
  if (history_position.size() < window_size_)
    return false;

  fusion::ObjectPtr obj = track_ref_->GetFusedObject()->GetBaseObject();

  double x_temp = 0.0;
  double y_temp = 0.0;

  // for (int i = 1; i < history_position.size(); i++) {
  //   x_temp += (history_position[i].x() - history_position[i - 1].x());
  //   y_temp += (history_position[i].y() - history_position[i - 1].y());
  // }
  GetStaticPosition();
  for (int i = 0; i < history_position.size(); i++) {
    x_temp += (history_position[i].x() - static_position.x());
    y_temp += (history_position[i].y() - static_position.y());
  }

  double result = std::sqrt(std::pow(x_temp, 2) + std::pow(y_temp, 2));

// use the speed of fusion as a constraint
  double abs_speed = std::sqrt(std::pow(obj->velocity[0], 2) +
                               std::pow(obj->velocity[1], 2));

  if (result < position_threshold && abs_speed < 2.0) {
    static_count++;
    motion_count = 0;
    if (static_count == 2) {
      obj->is_static = true;
      motion_count = 0;
      // static_position = history_position.front();
      GetStaticPosition();
    }
  } else {
    motion_count++;
    if (motion_count >= 2) {
      obj->is_static = false;
      static_count = 0;
      //若运动状态为动态，清空缓存的位置、速度信息;
      // history_position.clear();
      // history_velocity.clear();
    }
  }
  return true;
}

void tracker_optimizer::SetStaticMotion() {
  fusion::ObjectPtr obj = track_ref_->GetFusedObject()->GetBaseObject();

  //自行车 行人 不做任何处理;
  if (obj->type == ObjectType::BICYCLE || obj->type == ObjectType::PEDESTRIAN) {
    return ;
  }
  if (obj->is_static) {
    // 静态的位置、速度暂不设置。
    // obj->velocity[0] = 0.0;
    // obj->velocity[1] = 0.0;
    // obj->position = static_position;
  }
}

void tracker_optimizer::CheckYawIsReasonable() {
  if (history_position.size() < window_size_) return;
  fusion::ObjectPtr obj = track_ref_->GetFusedObject()->GetBaseObject();
  // //目前只检查 行人和自行车的航向;
  // if (obj->type != ObjectType::BICYCLE && obj->type != ObjectType::PEDESTRIAN) {
  //   return;
  // }
  if (obj->type != ObjectType::PEDESTRIAN) {
    return;
  }

  // 选择距离差最大的两个点，避免航向交错;
  int max_i = 0, max_j = 1;
  double max_distance = 0;
  for (size_t j = 1; j < history_position.size(); j++) {
    for (size_t i = 0; i < j; i++) {
      double temp_distance = std::sqrt(
          std::pow(history_position[max_i].x() - history_position[max_j].x(), 2) +
          std::pow(history_position[max_i].y() - history_position[max_j].y(), 2));
      if (temp_distance > max_distance) {
        max_distance = temp_distance;
        max_i = i;
        max_j = j;
      }
    }
  }

  double delt_px = history_position[max_j].x() - history_position[max_i].x();
  double delt_py = history_position[max_j].y() - history_position[max_i].y();

  double yaw_temp = atan2(delt_py, delt_px);
  if (yaw_temp > 2 * M_PI) {
    yaw_temp -= 2 * M_PI;
  } else if (yaw_temp < 0)
    yaw_temp += 2 * M_PI;

  if (fabs(yaw_temp - obj->yaw) > 0.4) {
    obj->yaw = yaw_temp;
  }
  return;
}

void tracker_optimizer::GetStaticPosition() {
  // 计算一个稳定的静态位置;
  if (history_position.size() < window_size_)
    return;

  double px_sum = 0.0, py_sum = 0.0;
  for (int j = 0; j < history_position.size(); j++) {
    px_sum += history_position[j].x();
    py_sum += history_position[j].y();
  }
  static_position[0] = px_sum / window_size_;
  static_position[1] = py_sum / window_size_;
}

}  // namespace fusion
}  // namespace perception