#ifndef MAP_SERVER_H_
#define MAP_SERVER_H_

#include <json/json.h>
#include <Eigen/Core>
#include <vector>
#include <map>
#include <string>

#include <ros/ros.h>

#include <common/proto/localization.pb.h>


namespace perception {
namespace fusion {
namespace map{

struct Point {
  Point() {}
  Point(double x, double y) : x(x), y(y) {}
  Point(double x, double y, double z) : x(x), y(y), z(z) {}

  bool operator ==(const Point & a) const {
    return (x == a.x &&	y == a.y && z == a.z );
  }

  void operator =(const Point & a) {
    x = a.x;
    y = a.y;
    z = a.z;
  }

  void operator +=(const Point & a) {
    x += a.x;
    y += a.y;
    z += a.z;
  }

  void operator /=(const int & t) {
    x /= t;
    y /= t;
    z /= t;
  }

  bool operator < (const Point & rhs) const { 
    if ((x < rhs.x) || (x == rhs.x && y < rhs.y)) { return true; }
    return false; 
  }

  double x{0.0};
  double y{0.0};
  double z{0.0};
};

using V_Points = std::vector<Point>;
using V_V_Points = std::vector<V_Points>;

void Init(std::string map_name);

V_V_Points GetCurrentVegPolygons();

V_V_Points GetCurrentBuildingPolygons();

Point computeCenter(const V_Points& polygon);

void UploadPolygons(const localization::Localization& localization);

void Destroy();

} // namespace map
} // namespace fusion
} // namespace perception

#endif
