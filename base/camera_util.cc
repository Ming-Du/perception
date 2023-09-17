/**
 * @file camera_util.cc
 **/

#include "perception/base/camera_util.h"
#include "perception/base/sensor_manager/sensor_manager.h"

#include <cmath>
#include <limits>

namespace perception {
namespace base {


void calculate_car_position(
  const std::string& camera_id,
  const ::perception::BBox2D& box, Eigen::Vector2d &position,
  const float obj_width, const float obj_length) {
  base::SensorInfo sensor_info;
  base::SensorManager* sensor_manager = base::SensorManager::Instance();
  if (!sensor_manager->GetSensorInfo(camera_id, &sensor_info)) {
    LOG(FATAL) << "wrong sensor input!!, " << camera_id;
  }
  const ::perception::base::Intrinsic &intrinsic = sensor_info.intrinsic();

  const float f = 0.006;      //focal length, m
  const float fx = intrinsic.matrix(0);
  const float center_x = intrinsic.width() * 0.5;
  const float cell_x = f/fx;
  float x = 0;
  float y = 0;

  float c1 = (center_x - box.xmin()) * cell_x;
  float c2 = (center_x - box.xmax()) * cell_x;
  if(c1>0 && c2>0){
    x = (f*obj_width + c2*obj_length + c2*f - c1*f)/(c1-c2);
    y = c1*(x+f)/f;
  } else if (c1 < 0 && c2 < 0) {
    x = (f*obj_width - c1*obj_length + c2*f - c1*f)/(c1-c2);
    y = c2*(x+f)/f + obj_width;
  } else{
    x = (f*obj_width)/(c1-c2)-f;
    y = c1*(f+x)/f;
  }
  x += obj_length/2;
  y -= obj_width/2;

  position.x() = x;
  position.y() = y;
}

bool Pt3dToCamera2d(const Eigen::Vector3d& pt3d,
                    const Eigen::Matrix4d& world2camera_pose,
                    base::BaseCameraModelPtr camera_model,
                    Eigen::Vector2d* pt2d) {
  Eigen::Vector4d local_pt = static_cast<Eigen::Matrix<double, 4, 1, 0, 4, 1>>(
      world2camera_pose * Eigen::Vector4d(pt3d(0), pt3d(1), pt3d(2), 1));
  if (local_pt[2] > 0) {
    *pt2d =
        (camera_model->Project(Eigen::Vector3f(
             static_cast<float>(local_pt[0]), static_cast<float>(local_pt[1]),
             static_cast<float>(local_pt[2]))))
            .cast<double>();
    return true;
  }
  return false;
}

bool Camera2dToPt3d(const Eigen::Vector2d& pt2d,
                    const Eigen::Affine3d& camera2world_pose,
                    base::BaseCameraModelPtr camera_model,
                    Eigen::Vector3d* pt3d) {
  auto pinhole = dynamic_cast<base::PinholeCameraModel *>(camera_model.get());
  if (!pinhole) {
    return false;
  }

  float rows = pinhole->get_height(), cols = pinhole->get_width();
  // 焦距
  float fx = pinhole->get_intrinsic_params()(0, 0);
  float fy = pinhole->get_intrinsic_params()(1, 1);
  // 相机俯仰角
  double pitch = camera2world_pose.rotation().eulerAngles(2,1,0).x();
  // 像素点相对于z方向(天向)的角度
  double alpha = std::atan((pt2d.y() - 0.5 * rows) / fy) + pitch;
  // 假设障碍物在地面, 那么在地平线之上的不考虑
  if (alpha < M_PI_2) {
    return false;
  }
  // 相机坐标系z轴到目标平面的距离
  double d0 = camera2world_pose.translation().z() / std::sin(alpha - M_PI_2);
  pt3d->x() = camera2world_pose.translation().z() / std::tan(alpha - M_PI_2);
  pt3d->y() = d0 * (0.5 * cols - pt2d.x()) / fx;
  pt3d->z() = 0; // 地平面
  return true;
}


}  // namespace fusion
}  // namespace perception
