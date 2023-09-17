#ifndef PERCEPTION_FUSION_COMMON_FRAME_TRANSFORM_H_
#define PERCEPTION_FUSION_COMMON_FRAME_TRANSFORM_H_


#include <common/proto/localization.pb.h>

#include <Eigen/Core>
#include <cmath>

namespace perception {
namespace fusion {

struct Pose {
  using Ptr = std::shared_ptr<Pose>;

  double x = 0.f, y = 0.f, z = 0.f, roll = 0.f, pitch = 0.f, yaw = 0.f;
};

Eigen::Matrix4d poseToEigenMat(const Pose &pose);
void transGlobal2VehicleMat(const localization::Localization &localization,
                            Eigen::Matrix4d &transform_mat);
void transVehicle2GlobalMat(const localization::Localization &localization,
                            Eigen::Matrix4d &transform_mat);

void transGlobal2VehicleMat(const Eigen::VectorXd &localization_pose,
                            Eigen::Matrix4d &transform_mat);
void transVehicle2GlobalMat(const Eigen::VectorXd &localization_pose,
                            Eigen::Matrix4d &transform_mat);

// only global to vehicle   Transform_matrix
inline void
transGlobal2VehicleMat(const localization::Localization &localization,
                       Eigen::Matrix4d &transform_mat) {
  Pose::Ptr host_pose(new Pose);
  Pose::Ptr ego_pose(new Pose);

  host_pose->x = localization.position().x();
  host_pose->y = localization.position().y();
  host_pose->z = localization.position().z();
  host_pose->yaw = localization.yaw();
  host_pose->pitch = localization.pitch();
  host_pose->roll = localization.roll();

  ego_pose->x = 0;
  ego_pose->y = 0;
  ego_pose->z = 0;
  ego_pose->yaw = 0;
  ego_pose->pitch = 0;
  ego_pose->roll = 0;

  Eigen::Matrix4d ego_mat, host_mat;
  host_mat = poseToEigenMat(*host_pose);
  ego_mat = poseToEigenMat(*ego_pose);

  transform_mat = ego_mat * host_mat.inverse();
}

// Modify-guoxiaoxiao
inline void transGlobal2VehicleMat(const Eigen::VectorXd &localization_pose,
                                   Eigen::Matrix4d &transform_mat) {
  Pose::Ptr host_pose(new Pose);
  Pose::Ptr ego_pose(new Pose);

  host_pose->x = localization_pose(0);
  host_pose->y = localization_pose(1);
  host_pose->z = localization_pose(2);
  host_pose->roll = localization_pose(3);
  host_pose->pitch = localization_pose(4);
  host_pose->yaw = localization_pose(5);

  ego_pose->x = 0;
  ego_pose->y = 0;
  ego_pose->z = 0;
  ego_pose->yaw = 0;
  ego_pose->pitch = 0;
  ego_pose->roll = 0;

  Eigen::Matrix4d ego_mat, host_mat;
  host_mat = poseToEigenMat(*host_pose);
  ego_mat = poseToEigenMat(*ego_pose);

  transform_mat = ego_mat * host_mat.inverse();
}

// only vehicle to global   Transform_matrix
inline void
transVehicle2GlobalMat(const localization::Localization &localization,
                       Eigen::Matrix4d &transform_mat) {
  Pose::Ptr host_pose(new Pose);
  Pose::Ptr ego_pose(new Pose);

  host_pose->x = localization.position().x();
  host_pose->y = localization.position().y();
  host_pose->z = localization.position().z();
  host_pose->yaw = localization.yaw();
  host_pose->pitch = localization.pitch();
  host_pose->roll = localization.roll();

  ego_pose->x = 0;
  ego_pose->y = 0;
  ego_pose->z = 0;
  ego_pose->yaw = 0;
  ego_pose->pitch = 0;
  ego_pose->roll = 0;

  Eigen::Matrix4d cur_mat, next_mat;
  Eigen::Matrix4d ego_mat, host_mat;
  ego_mat = poseToEigenMat(*ego_pose);
  host_mat = poseToEigenMat(*host_pose);

  transform_mat = host_mat * ego_mat.inverse();
}

inline void transVehicle2GlobalMat(const Eigen::VectorXd &localization_pose,
                                   Eigen::Matrix4d &transform_mat) {
  Pose::Ptr host_pose(new Pose);
  Pose::Ptr ego_pose(new Pose);

  host_pose->x = localization_pose(0);
  host_pose->y = localization_pose(1);
  host_pose->z = localization_pose(2);
  host_pose->roll = localization_pose(3);
  host_pose->pitch = localization_pose(4);
  host_pose->yaw = localization_pose(5);

  ego_pose->x = 0;
  ego_pose->y = 0;
  ego_pose->z = 0;
  ego_pose->yaw = 0;
  ego_pose->pitch = 0;
  ego_pose->roll = 0;

  Eigen::Matrix4d cur_mat, next_mat;
  Eigen::Matrix4d ego_mat, host_mat;
  ego_mat = poseToEigenMat(*ego_pose);
  host_mat = poseToEigenMat(*host_pose);

  transform_mat = host_mat * ego_mat.inverse();
}

inline Eigen::Matrix4d poseToEigenMat(const Pose &pose) {
  Eigen::AngleAxisd init_rotation_x(pose.roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd init_rotation_y(pose.pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd init_rotation_z(pose.yaw, Eigen::Vector3d::UnitZ());

  Eigen::Translation3d init_translation(pose.x, pose.y, pose.z);

  return (init_translation * init_rotation_z * init_rotation_y *
          init_rotation_x)
      .matrix();
}

} // namespace fusion
} // namespace perception

#endif // PERCEPTION_FUSION_COMMON_FRAME_TRANSFORM_H_
