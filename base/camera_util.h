#pragma once

#include <memory>
#include <vector>

#include "Eigen/Dense"
#include "perception/base/camera.h"
#include "object.h"

namespace perception {
namespace base {

// @brief: get object eight vertices
// @param [in]: object
// @param [in/out]: vertices
// void GetObjectEightVertices(std::shared_ptr<const base::Object> obj,
//                             std::vector<Eigen::Vector3d>* vertices);

void calculate_car_position(
  const std::string& camera_id,
  const ::perception::BBox2D& box, Eigen::Vector2d &position,
  const float obj_width, const float obj_length);

template <typename VectorType>
bool IsPtInFrustum(const VectorType& pt, double width, double height) {
  if (pt[0] < 0 || pt[0] > width || pt[1] < 0 || pt[1] > height) {
    return false;
  }
  return true;
}

// @brief: project 3d point to 2d point in camera
// @param [in]: pt3d
// @param [in]: world2camera_pose
// @param [in]: camera_model
// @param [in/out]: pt2d
// @return: true if local 3d point z > 0, else false
bool Pt3dToCamera2d(const Eigen::Vector3d& pt3d,
                    const Eigen::Matrix4d& world2camera_pose,
                    base::BaseCameraModelPtr camera_model,
                    Eigen::Vector2d* pt2d);

bool Camera2dToPt3d(const Eigen::Vector2d& pt2d,
                    const Eigen::Affine3d& camera2world_pose,
                    base::BaseCameraModelPtr camera_model,
                    Eigen::Vector3d* pt3d);


}  // namespace fusion
}  // namespace perception
