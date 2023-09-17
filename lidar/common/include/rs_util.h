/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#ifndef RS_COMMON_RS_UTIL_H_
#define RS_COMMON_RS_UTIL_H_

#include <Eigen/Dense>
#include <chrono>
#include "basic_type/pose.h"

namespace robosense{

inline double control_psi(double psi) {
    double pi = 3.14159265358979323846;
    if (psi > pi) {
        psi = psi - 2 * pi;
    } else if (psi < -pi) {
        psi = psi + 2 * pi;
    }
    return psi;
}

inline bool IsPointInBBox(const Eigen::Vector2d &gnd_c, const Eigen::Vector2d &dir_x, const Eigen::Vector2d &dir_y,
                          const Eigen::Vector2d &size, const Eigen::Vector2d &point) {
    // double zmin = gnd_c[2] - size[2] * 0.5;
    // double zmax = gnd_c[2] + size[2] * 0.5;

    // if (point.z < (zmin - 0.05) || point.z > (zmax + 0.05)) {
    //     return false;
    // }
    Eigen::Vector2d eig(point(0), point(1));
    Eigen::Vector2d diff = eig - gnd_c;
    double x = diff.dot(dir_x);
    if (fabs(x) > (size[0] * 0.5 + 0.05)) {
        return false;
    }
    double y = diff.dot(dir_y);
    if (fabs(y) > (size[1] * 0.5 + 0.05)) {
        return false;
    }
    return true;
}

inline double getTime(void) {
    const auto t = std::chrono::system_clock::now();
    const auto t_sec = std::chrono::duration_cast<std::chrono::duration<double>>(t.time_since_epoch());
    return t_sec.count();
}

inline double normalizeAngle(const double &a) {
    double b = a;
    while (b < -M_PI) {
        b += 2 * M_PI;
    }
    while (b > M_PI) {
        b -= 2 * M_PI;
    }
    return b;
}


inline Eigen::Matrix4d poseToEigenMat(const Pose& pose) {
    Eigen::AngleAxisd init_rotation_x(pose.roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd init_rotation_y(pose.pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd init_rotation_z(pose.yaw, Eigen::Vector3d::UnitZ());

    Eigen::Translation3d init_translation(pose.x, pose.y, pose.z);

    return (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
}

inline Pose eigenMatToPose(const Eigen::Matrix4d& mat) {
    Pose pose;
    pose.x = mat(0, 3);
    pose.y = mat(1, 3);
    pose.z = mat(2, 3);
    pose.roll = std::atan2(mat(2, 1), mat(2, 2));
    pose.pitch = std::atan2(-mat(2, 0), sqrtf(mat(2, 1) * mat(2, 1) + mat(2, 2) * mat(2, 2)));
    pose.yaw = std::atan2(mat(1, 0), mat(0, 0));
    return pose;
}

}  // namespace robosense

#endif  // RS_COMMON_RS_UTIL_H_
