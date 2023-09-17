/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "perception/base/object.h"

namespace perception {
namespace base {

// template<typename T>
// Eigen::Matrix<T, 3, 1> PointToEigenVector(::geometry::Point3D point3d)
// {
//   return Eigen::Matrix<T, 3, 1>(point3d.x(), point3d.y(), point3d.z());
// }

bool SetSensorExtrinsics(
  const Extrinsic extrinsic, Eigen::Affine3d& sensor2world_pose) {
  // 数据变换方向与坐标轴变换方向相反
  // axis_world -> axis_sensor == data_sensor -> data_world
  sensor2world_pose.setIdentity();
  if (!extrinsic.has_translation() || !extrinsic.has_rotation()) {
    return false;
  }
  sensor2world_pose.translation() << extrinsic.translation().x(),
                                     extrinsic.translation().y(),
                                     extrinsic.translation().z();
  sensor2world_pose.rotate(Eigen::Quaterniond(extrinsic.rotation().w(),
                                              extrinsic.rotation().x(),
                                              extrinsic.rotation().y(),
                                              extrinsic.rotation().z()));
  return true;
}

std::vector<std::string> split(const std::string& s, char delimiter) {
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream tokenStream(s);
  while (std::getline(tokenStream, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}

Eigen::Vector3i QueryRandomColor(int seed) {
  int base = 1024;
  int offset = seed * 123457 % base;

  float colors[6][3] = { {1,0,1}, {0,0,1}, {0,1,1}, {0,1,0}, {1,1,0}, {1,0,0} };

  float ratio = (static_cast<float>(offset) / base) * 5.f;
  int i = floor(ratio);
  int j = ceil(ratio);
  ratio -= i;

  Eigen::Vector3i random_color;
  for (int n = 0; n < random_color.size(); ++n) {
    float color = (1 - ratio) * colors[i][n] + ratio * colors[j][n];
    random_color[n] = static_cast<int>(color * 255) & 255; // BGR
  }
  return random_color;
}

}  // namespace base
}  // namespace perception
