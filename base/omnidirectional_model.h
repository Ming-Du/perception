#pragma once

#include <memory>
#include <string>

#include "Eigen/Core"

#include "perception/base/camera.h"
#include "perception/base/distortion_model.h"
#include "perception/base/polynomial.h"

namespace perception {
namespace base {

class OmnidirectionalCameraDistortionModel : public BaseCameraDistortionModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  OmnidirectionalCameraDistortionModel() = default;
  ~OmnidirectionalCameraDistortionModel() = default;

  Eigen::Vector2f Project(const Eigen::Vector3f& point3d) override;

  std::shared_ptr<BaseCameraModel> get_camera_model() override;

  std::string name() const override {
    return "OmnidirectionalCameraDistortionModel";
  }

  bool set_params(size_t width, size_t height,
                  const Eigen::VectorXf& params) override;

 protected:
  Eigen::Matrix3f intrinsic_params_;
  Polynomial cam2world_;
  Polynomial world2cam_;
  float center_[2];  // x, y
  float affine_[3];  // c, d, e
};

/* TODO(all): to remove
typedef std::shared_ptr<OmnidirectionalCameraDistortionModel>
    OmnidirectionalCameraDistortionModelPtr;
typedef std::shared_ptr<const OmnidirectionalCameraDistortionModel>
    OmnidirectionalCameraDistortionModelConstPtr;
*/

}  // namespace base
}  // namespace perception

