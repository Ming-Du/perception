#include "perception/base/distortion_model.h"

#include "common/include/pb_utils.h"
#include "common/include/log.h"

#include "yaml-cpp/yaml.h"

namespace perception {
namespace base {

Eigen::Vector2f BrownCameraDistortionModel::Project(
    const Eigen::Vector3f& point3d) {
  if (std::isless(point3d[2], 0.f)) {
    // AERROR << "The input point (" << point3d
          //  << ") should be in front of the camera";
  }
  // radial distortion coefficients
  const float k1 = distort_params_[0];
  const float k2 = distort_params_[1];
  const float k3 = distort_params_[4];
  // tangential distortion coefficients
  const float p1 = distort_params_[2];
  const float p2 = distort_params_[3];

  Eigen::Vector2f pt2d_img;
  // normalized
  const Eigen::Vector2f pt_normalized(point3d[0] / point3d[2],
                                      point3d[1] / point3d[2]);
  const float x_n = pt_normalized[0];
  const float y_n = pt_normalized[1];
  const float x_mul_x = x_n * x_n;
  const float y_mul_y = y_n * y_n;
  const float x_mul_y = x_n * y_n;
  const float r_squared = x_mul_x + y_mul_y;
  const float r_to_the_4th = r_squared * r_squared;
  const float r_to_the_6th = r_squared * r_to_the_4th;

  // radial distortion
  pt2d_img = pt_normalized *
             (1 + k1 * r_squared + k2 * r_to_the_4th + k3 * r_to_the_6th);

  // tangential distortion
  pt2d_img[0] += 2 * p1 * x_mul_y + p2 * (r_squared + 2 * x_mul_x);
  pt2d_img[1] += p1 * (r_squared + 2 * y_mul_y) + 2 * p2 * x_mul_y;

  // transform to image coordinates
  const float fx = intrinsic_params_(0, 0);
  const float fy = intrinsic_params_(1, 1);
  const float cx = intrinsic_params_(0, 2);
  const float cy = intrinsic_params_(1, 2);
  pt2d_img[0] = fx * pt2d_img[0] + cx;
  pt2d_img[1] = fy * pt2d_img[1] + cy;

  return pt2d_img;
}

std::shared_ptr<BaseCameraModel>
BrownCameraDistortionModel::get_camera_model() {
  std::shared_ptr<PinholeCameraModel> camera_model(new PinholeCameraModel());
  camera_model->set_width(width_);
  camera_model->set_height(height_);
  camera_model->set_intrinsic_params(intrinsic_params_);

  return std::dynamic_pointer_cast<BaseCameraModel>(camera_model);
}

bool BrownCameraDistortionModel::set_params(size_t width, size_t height,
                                            const Eigen::VectorXf& params) {
  if (params.size() != 14) {
    return false;
  }

  width_ = width;
  height_ = height;
  intrinsic_params_(0, 0) = params(0);
  intrinsic_params_(0, 1) = params(1);
  intrinsic_params_(0, 2) = params(2);
  intrinsic_params_(1, 0) = params(3);
  intrinsic_params_(1, 1) = params(4);
  intrinsic_params_(1, 2) = params(5);
  intrinsic_params_(2, 0) = params(6);
  intrinsic_params_(2, 1) = params(7);
  intrinsic_params_(2, 2) = params(8);

  distort_params_[0] = params[9];
  distort_params_[1] = params[10];
  distort_params_[2] = params[11];
  distort_params_[3] = params[12];
  distort_params_[4] = params[13];

  return true;
}

bool LoadBrownCameraIntrinsic(const std::string &yaml_file,
                              BrownCameraDistortionModel *model) {
  if (!common::PathExists(yaml_file) || model == nullptr) {
    return false;
  }

  YAML::Node node = YAML::LoadFile(yaml_file);
  if (node.IsNull()) {
    AINFO << "Load " << yaml_file << " failed! please check!";
    return false;
  }

  float camera_width = 0.0f;
  float camera_height = 0.0f;
  Eigen::VectorXf params(9 + 5);
  try {
    camera_width = node["width"].as<float>();
    camera_height = node["height"].as<float>();
    for (size_t i = 0; i < 9; ++i) {
      params(i) = node["K"][i].as<float>();
    }
    for (size_t i = 0; i < 5; ++i) {
      params(9 + i) = node["D"][i].as<float>();
    }

    model->set_params(static_cast<size_t>(camera_width),
                      static_cast<size_t>(camera_height), params);
  } catch (YAML::Exception &e) {
    AERROR << "load camera intrisic file " << yaml_file
           << " with error, YAML exception: " << e.what();
    return false;
  }

  return true;
}

}  // namespace base
}  // namespace perception
