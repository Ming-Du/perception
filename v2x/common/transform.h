#pragma once

#include "common/proto/localization.pb.h"
#include <proj_api.h>
#include <std_msgs/String.h>
#include <cmath>
#include <Eigen/Core>

namespace perception{
namespace v2x{

const double DEG_TO_RAD_V2X = M_PI / 180.0;
const double RAD_TO_DEG_V2X = 180.0 / M_PI;

struct Pose{
  using Ptr = std::shared_ptr<Pose>;
  double x = 0.f, y = 0.f, z = 0.f, roll = 0.f, pitch = 0.f, yaw = 0.f;
};

class point {
public:
    double x; /**< 经度 度. */
    double y; /**< 纬度 度. */
    point() {
        x = 0;
        y = 0;
    }
    point(double xx, double yy) {
        x = xx;
        y = yy;
    }
    template <typename T>
    point(const T &p) {
        x = p.x;
        y = p.y;
    }
    template <typename T>
    void setPoint(T p) {
        x = p.x;
        y = p.y;
    }
    template <typename T>
    void setPoint(T _x, T _y) {
        x = _x;
        y = _y;
    }
    ~point() {}
    bool operator==(const point &A);
    point &operator=(const point &A);
    point operator-();
    point operator-(const point &A);
    point operator+(const point &A);
    template <typename T>
    point operator*(T b);
    template <typename T>
    point operator/(T b);
    friend point dotMultiMat(const point A, const point B);
    friend point dotDevideMat(const point A, const point B);
    friend double dot(const point A, const point B);
    friend double cross(const point A, const point B);
};
point dotMultiMat(const point A, const point B);
point dotDevideMat(const point A, const point B);
double dot(const point A, const point B);
double cross(const point A, const point B);

template <typename T>
point point::operator*(T b) {
    point tmp;
    tmp.x = x * b;
    tmp.y = y * b;
    return tmp;
}

template <typename T>
point point::operator/(T b) {
  return *(this) * (1.0 / b);
}

// Modify(@liuxinyu): gps covert to MCT(UTM)
template <typename TT>
point GPS2MCT(TT gpsP) {
  point ret;
  const float c_fA = 6378.137;  // km
  const float c_fE = 0.0818192; //;sqrt(0.00669438)
  const float c_fK0 = 0.9996;
  const float c_fE0 = 500;
  const float c_fN0 = 0;
  long zoneNum;
  double phi, lamda, lamda0;

  zoneNum = (long)(gpsP.x / 6) + 31;
  lamda0 = (zoneNum - 1) * 6 - 180 + 3;
  lamda0 = lamda0 * M_PI / 180.0;
  phi = gpsP.y * M_PI / 180.0;
  lamda = gpsP.x * M_PI / 180.0;
  double v, A, T, C, s, UTME, UTMN;
  v = 1.0 / sqrt(1 - pow(c_fE, 2) * pow(sin(phi), 2));
  A = (lamda - lamda0) * cos(phi);
  T = tan(phi) * tan(phi);
  C = pow(c_fE, 2) * cos(phi) * cos(phi) / (1 - pow(c_fE, 2));
  s = (1 - pow(c_fE, 2) / 4.0 - 3 * pow(c_fE, 4) / 64.0 - 5 * pow(c_fE, 6) / 256.0) * phi - (3 * pow(c_fE, 2) / 8.0 + 3 * pow(c_fE, 4) / 32.0 + 45 * pow(c_fE, 6) / 1024.0) * sin(2 * phi) + (15 * pow(c_fE, 4) / 256.0 + 45 * pow(c_fE, 6) / 1024.0) * sin(4 * phi) - 35 * pow(c_fE, 6) / 3072.0 * sin(6 * phi);
  UTME = c_fE0 + c_fK0 * c_fA * v * (A + (1 - T + C) * pow(A, 3) / 6 + (5 - 18 * T + pow(T, 2)) * pow(A, 5) / 120);
  UTMN = c_fN0 + c_fK0 * c_fA * (s + v * tan(phi) * (pow(A, 2) / 2 + (5 - T + 9 * C + 4 * pow(C, 2)) * pow(A, 4) / 24 + (61 - 58 * T + pow(T, 2)) * pow(A, 6) / 720.0));
  ret.x = UTME * 1000; //
  ret.y = UTMN * 1000; // point

  return ret;
}

static bool is_init = false;
constexpr double RAD_TO_DEG_LOCAL = 180.0 / M_PI;
static projPJ wgs84pj_target_;
static projPJ utm_source_;
static std::string UTM_ZONE = "50";
static void UtmTransformGps(double &utm_x, double &utm_y, double utm_z) {
  if (!is_init) {
    std::string str = "+proj=utm +zone=" + UTM_ZONE + " +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs";
    utm_source_ = pj_init_plus(str.c_str());
    wgs84pj_target_ = pj_init_plus("+proj=latlong +ellps=WGS84");
    is_init = true;
  }
  // utm坐标改写为gps坐标
  pj_transform(utm_source_, wgs84pj_target_, 1, 1, &utm_x, &utm_y, &utm_z);
  utm_x *= RAD_TO_DEG_LOCAL;
  utm_y *= RAD_TO_DEG_LOCAL;
}

static Eigen::Matrix4d poseToEigenMat(const Pose& pose);
static void transGlobal2VehicleMat(const localization::Localization & localization, 
                            Eigen::Matrix4d &transform_mat);
static void transVehicle2GlobalMat(const localization::Localization & localization,
                            Eigen::Matrix4d &transform_mat);

static void transGlobal2VehicleMat(const Eigen::VectorXd & localization_pose,
                            Eigen::Matrix4d &transform_mat);
static void transVehicle2GlobalMat(const Eigen::VectorXd & localization_pose, 
                            Eigen::Matrix4d &transform_mat);

//only global to vehicle   Transform_matrix
void transGlobal2VehicleMat(const localization::Localization & localization, 
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
void transGlobal2VehicleMat(const Eigen::VectorXd & localization_pose,
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

//only vehicle to global   Transform_matrix
void transVehicle2GlobalMat(const localization::Localization & localization, 
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

void transVehicle2GlobalMat(const Eigen::VectorXd & localization_pose,
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

Eigen::Matrix4d poseToEigenMat(const Pose& pose) {
    Eigen::AngleAxisd init_rotation_x(pose.roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd init_rotation_y(pose.pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd init_rotation_z(pose.yaw, Eigen::Vector3d::UnitZ());

    Eigen::Translation3d init_translation(pose.x, pose.y, pose.z);

    return (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
}

}  // namespace v2x
}  // namespace perception

