#pragma once
#include <common/proto/localization.pb.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

namespace perception {
namespace fusion {

void UtmTransformGps(double& utm_x, double& utm_y, double utm_z);

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
  point(const T& p) {
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
  bool operator==(const point& A);
  point& operator=(const point& A);
  point operator-();
  point operator-(const point& A);
  point operator+(const point& A);
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
  const float c_fA = 6378.137;   // km
  const float c_fE = 0.0818192;  //;sqrt(0.00669438)
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
  s = (1 - pow(c_fE, 2) / 4.0 - 3 * pow(c_fE, 4) / 64.0 - 5 * pow(c_fE, 6) / 256.0) * phi -
      (3 * pow(c_fE, 2) / 8.0 + 3 * pow(c_fE, 4) / 32.0 + 45 * pow(c_fE, 6) / 1024.0) *
          sin(2 * phi) +
      (15 * pow(c_fE, 4) / 256.0 + 45 * pow(c_fE, 6) / 1024.0) * sin(4 * phi) -
      35 * pow(c_fE, 6) / 3072.0 * sin(6 * phi);
  UTME = c_fE0 + c_fK0 * c_fA * v *
                     (A + (1 - T + C) * pow(A, 3) / 6 + (5 - 18 * T + pow(T, 2)) * pow(A, 5) / 120);
  UTMN = c_fN0 + c_fK0 * c_fA *
                     (s + v * tan(phi) *
                              (pow(A, 2) / 2 + (5 - T + 9 * C + 4 * pow(C, 2)) * pow(A, 4) / 24 +
                               (61 - 58 * T + pow(T, 2)) * pow(A, 6) / 720.0));
  ret.x = UTME * 1000;  //
  ret.y = UTMN * 1000;  // point

  return ret;
}

}  // namespace fusion
}  // namespace perception