#pragma once
#include <geos/geom/Coordinate.h>

#include <Eigen/Core>
#include <cmath>
#include <iomanip>

#include "common/proto/BSM_PB.pb.h"
#include "common/proto/BasicSafetyMessage_PB.pb.h"
#include "common/proto/RSI_PB.pb.h"
#include "common/proto/RSI_RTE_PB.pb.h"
#include "common/proto/RSM_PB.pb.h"
#include "common/proto/RSM_PNT_PB.pb.h"
#include "common/proto/SSM_PB.pb.h"
#include "common/proto/SSM_PNT_PB.pb.h"
#include "common/proto/localization.pb.h"
#include "common/proto/object.pb.h"
#include "geos/geom/CoordinateArraySequence.h"
#include "geos/geom/Geometry.h"
#include "geos/geom/GeometryFactory.h"
#include "geos/io/WKTReader.h"
#include "perception/fusion/base/point_cloud.h"
#include "perception/fusion/common/gps_proj.h"
#include "perception/v2x/common/config_manager/v2x_config_manager.h"

constexpr double RAD_TO_DEG = 180.0 / M_PI;
constexpr double DEG_TO_RAD = M_PI / 180.0;

namespace perception {
namespace v2x {
using geos::geom::Coordinate;
using geos::geom::Geometry;
using geos::geom::GeometryFactory;
using namespace geos::geom;

typedef geos::geom::LinearRing* LinearRingPtr;
class SelfVehIdentBase {
 public:
  SelfVehIdentBase() {}
  virtual ~SelfVehIdentBase() = default;
  void SetHost(localization::Localization& v) { localization = v; }
  void SetParam();
  void GetSsmObs(::common::SSM_PNT_PB& v);
  void GetRsmObs(::common::RSM_PNT_PB& v);
  virtual void LocateSelf() = 0;

 public:
  double estm_;
  bool is_filter_flag_;

 protected:
  /*weight struct of self identify*/
  struct WeightIDE {
    double lon;
    double lat;
    double heading;
    double speed;
    double width;
    double length;
    double eva;
    double host_length;
    double host_width;
    double host_high;
    double iou_thr;
    WeightIDE() {
      lon = 0.41;
      lat = 0.41;
      heading = 1.2;
      width = 0;
      length = 0;
      speed = 0;
      eva = 100;
      host_length = 5.995;
      host_width = 2.020;
      host_high = 2.560;
      iou_thr = 0.5;
    }
  };
  struct Candidate {
    double lon;
    double lat;
    double heading;
    double speed;
    double width;
    double length;
  };

  WeightIDE weight; /*weight object*/
  Candidate obs;
  localization::Localization localization;
};

class SelfVehIdentOri : public SelfVehIdentBase {
 public:
  SelfVehIdentOri() {}
  ~SelfVehIdentOri() = default;
  void LocateSelf();

 private:
  float ConvertAngle(float angle, double lon, double lat);
  double DegToRad(double deg);
  double RadToDeg(double rad);
};

class SelfVehIdentIOU : public SelfVehIdentBase {
 public:
  SelfVehIdentIOU() {}
  ~SelfVehIdentIOU() = default;
  void LocateSelf();

 private:
  void AssginContours(perception::fusion::point object, double yaw, double len, double width,
                      perception::Object* obj);
  void PointRotate(const double cosYaw, const double sinYaw, const double dcx, const double dcy,
                   geometry::Point& point);
  void SetPointLFD(const perception::fusion::point& object, double len, double width,
                   geometry::Point& point);
  void SetPointLBD(const perception::fusion::point& object, double len, double width,
                   geometry::Point& point);
  void SetPointRFD(const perception::fusion::point& object, double len, double width,
                   geometry::Point& point);
  void SetPointRBD(const perception::fusion::point& object, double len, double width,
                   geometry::Point& point);
  double ComputeIOU(perception::Object* A, perception::Object* B, double trans_x, double trans_y);
};

}  // namespace v2x
}  // namespace perception