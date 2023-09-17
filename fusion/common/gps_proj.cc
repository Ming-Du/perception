#include "gps_proj.h"
#include <proj_api.h>
#include <std_msgs/String.h>

namespace perception {
namespace fusion {

static bool is_init = false;
constexpr double RAD_TO_DEG_LOCAL = 180.0 / M_PI;
static projPJ wgs84pj_target_;
static projPJ utm_source_;
std::string UTM_ZONE = "50";
void UtmTransformGps(double& utm_x, double& utm_y, double utm_z) {
  if (!is_init) {
    std::string str =
        "+proj=utm +zone=" + UTM_ZONE + " +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs";
    utm_source_ = pj_init_plus(str.c_str());
    wgs84pj_target_ = pj_init_plus("+proj=latlong +ellps=WGS84");
    is_init = true;
  }
  // utm坐标改写为gps坐标
  pj_transform(utm_source_, wgs84pj_target_, 1, 1, &utm_x, &utm_y, &utm_z);
  utm_x *= RAD_TO_DEG_LOCAL;
  utm_y *= RAD_TO_DEG_LOCAL;
}

bool point::operator==(const point& A) {
  if (fabs(x - A.x) < 1e-6 && fabs(y - A.y) < 1e-6) {
    return true;
  } else {
    return false;
  }
}

point& point::operator=(const point& A) {
  if (this != &A) {
    x = A.x;
    y = A.y;
  }
  return *this;
}

point point::operator-() {
  point tmp;
  tmp.x = -x;
  tmp.y = -y;
  return tmp;
}
point point::operator-(const point& A) {
  point tmp;
  tmp.x = x - A.x;
  tmp.y = y - A.y;
  return tmp;
}

point point::operator+(const point& A) {
  point tmp;
  tmp.x = A.x + x;
  tmp.y = A.y + y;
  return tmp;
}

point dotMultiMat(const point A, const point B) {
  point tmp;
  tmp.x = A.x * B.x;
  tmp.y = A.y * B.y;
  return tmp;
}

point dotDevideMat(const point A, const point B) {
  point tmp;
  tmp.x = A.x / B.x;
  tmp.y = A.y / B.y;
  return tmp;
}

double dot(const point A, const point B) {
  return A.x * B.x + A.y * B.y;
}

double cross(const point A, const point B) {
  return A.x * B.y - A.y * B.x;
}

}  // namespace fusion
}  // namespace perception
