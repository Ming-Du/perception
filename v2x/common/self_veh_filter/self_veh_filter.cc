#include "self_veh_filter.h"
namespace perception {
namespace v2x {
void SelfVehIdentBase::GetSsmObs(::common::SSM_PNT_PB& v) {
  obs.lon = v.participantdata().ptc().pos().offsetll().position_latlon().lon() * 1e-7;
  obs.lat = v.participantdata().ptc().pos().offsetll().position_latlon().lat() * 1e-7;
  obs.heading = v.participantdata().ptc().heading() * 0.0125;
  obs.speed = v.participantdata().ptc().speed() * 0.02;
  obs.width = v.participantdata().ptc().size().width() * 0.01;
  obs.length = v.participantdata().ptc().size().length() * 0.01;
}

void SelfVehIdentBase::GetRsmObs(::common::RSM_PNT_PB& v) {
  obs.lon = v.participantdata().pos().offsetll().position_latlon().lon() * 1e-7;
  obs.lat = v.participantdata().pos().offsetll().position_latlon().lat() * 1e-7;
  obs.heading = v.participantdata().heading() * 0.0125;
  obs.speed = v.participantdata().speed() * 0.02;
  obs.width = v.participantdata().size().width() * 0.01;
  obs.length = v.participantdata().size().length() * 0.01;
}

void SelfVehIdentBase::SetParam() {
  V2xConfigManager::Instance()->get_value("lon", &weight.lon);
  V2xConfigManager::Instance()->get_value("lat", &weight.lat);
  V2xConfigManager::Instance()->get_value("heading", &weight.heading);
  V2xConfigManager::Instance()->get_value("speed", &weight.speed);
  V2xConfigManager::Instance()->get_value("width", &weight.width);
  V2xConfigManager::Instance()->get_value("length", &weight.length);
  V2xConfigManager::Instance()->get_value("eva", &weight.eva);
  V2xConfigManager::Instance()->get_value("host_length", &weight.host_length);
  V2xConfigManager::Instance()->get_value("host_width", &weight.host_width);
  V2xConfigManager::Instance()->get_value("host_high", &weight.host_high);
  V2xConfigManager::Instance()->get_value("iou_thr", &weight.iou_thr);
  // std::cout << weight.lon << std::endl;
  // std::cout << weight.lat << std::endl;
  // std::cout << weight.heading << std::endl;
  // std::cout << weight.speed << std::endl;
  // std::cout << weight.width << std::endl;
  // std::cout << weight.length << std::endl;
  // std::cout << weight.eva << std::endl;
  // std::cout << weight.host_length << std::endl;
  // std::cout << weight.host_width << std::endl;
  // std::cout << weight.host_high << std::endl;
  // std::cout << weight.iou_thr << std::endl;
}

void SelfVehIdentOri::LocateSelf() {
  double delta_lon = 100;
  double delta_lat = 100;
  double delta_heading = 100;
  double delta_width = 100;
  double delta_length = 100;
  double delta_speed = 100;
  delta_lon = abs(localization.longitude() - obs.lon);
  delta_lat = abs(localization.latitude() - obs.lat);
  // UTM -> WGS84
  auto host_head = M_PI_2 - localization.yaw();
  if (host_head > 2 * M_PI) {
    host_head -= 2 * M_PI;
  } else if (host_head < 0) {
    host_head += 2 * M_PI;
  }
  host_head *= RAD_TO_DEG;
  float yaw_converted = ConvertAngle(host_head, localization.longitude(), localization.latitude());
  double tmp_h = abs(yaw_converted - obs.heading);
  delta_heading = tmp_h > 180 ? abs(tmp_h - 360) : tmp_h;

  delta_width = abs(weight.host_width - obs.width);
  delta_length = abs(weight.host_length - obs.length);
  double host_velocity =
      std::sqrt(std::pow(localization.longitudinal_v(), 2) + std::pow(localization.lateral_v(), 2));
  delta_speed = abs(host_velocity - obs.speed);
  estm_ = delta_lon * weight.lon + delta_lat * weight.lat + delta_heading * weight.heading +
          delta_width * weight.width + delta_length * weight.length + delta_speed * weight.speed;
  if (estm_ < weight.eva) {
    is_filter_flag_ = true;
  } else {
    is_filter_flag_ = false;
  }
}

float SelfVehIdentOri::ConvertAngle(float angle, double lon, double lat) {
  double angleOrigin = 450.0 - angle;
  double radianOrigin = DegToRad(angleOrigin);
  double radianLat = DegToRad(lat);
  double radianDeal = atan(tan(radianOrigin) * cos(radianLat));

  double ruler = 270.0;
  if (angle >= 0 && angle <= 180.0) {
    ruler = 90.0;
  } else {
    ruler = 270.0;
  }
  float result = (float)(ruler - RadToDeg(radianDeal));
  return result;
}

double SelfVehIdentOri::DegToRad(double deg) { return (deg / 180.0 * M_PI); }

double SelfVehIdentOri::RadToDeg(double rad) { return (rad / M_PI * 180.0); }

void SelfVehIdentIOU::LocateSelf() {
  perception::fusion::point object_gps;
  perception::fusion::point loc_utm;
  loc_utm.x = localization.position().x();
  loc_utm.y = localization.position().y();
  object_gps.x = obs.lon;
  object_gps.y = obs.lat;
  // utm convert to vehicle of obj
  perception::fusion::point object_utm = GPS2MCT(object_gps);
  // convert obu angle from gps to utm
  double ruler = 270.0;
  double radianLat = DEG_TO_RAD * localization.latitude();
  float angle = 450 - RAD_TO_DEG * atan(tan(DEG_TO_RAD * (ruler - obs.heading)) / cos(radianLat));
  if (angle >= 0 && angle <= 180.0) ruler = 90.0;
  angle = 450 - RAD_TO_DEG * atan(tan(DEG_TO_RAD * (ruler - obs.heading)) / cos(radianLat));
  auto yaw = M_PI_2 - angle * DEG_TO_RAD;
  if (yaw > M_PI)
    yaw -= 2 * M_PI;
  else if (yaw < -M_PI)
    yaw += 2 * M_PI;
  /*
  0 —— —— —— —— 1
  |             |
  |             |
  |             |
  |             |
  |             |
  3 —— —— —— —— 2
  */
  perception::Object* obs_contour = new perception::Object;
  perception::Object* loc_contour = new perception::Object;
  AssginContours(object_utm, yaw, obs.length, obs.width, obs_contour);
  AssginContours(loc_utm, localization.yaw(), weight.host_length, weight.host_width, loc_contour);
  double iou = ComputeIOU(obs_contour, loc_contour, 0, 0);
  // std::cout << "LocateSelf: " << iou << " " << weight.iou_thr << std::endl;
  if (iou > weight.iou_thr) {
    is_filter_flag_ = true;
  } else {
    is_filter_flag_ = false;
  }
}

void SelfVehIdentIOU::AssginContours(perception::fusion::point object, double yaw, double len,
                                     double width, perception::Object* obj) {
  geometry::Point point[4];
  SetPointLFD(object, len, width, point[0]);
  SetPointLBD(object, len, width, point[1]);
  SetPointRBD(object, len, width, point[2]);
  SetPointRFD(object, len, width, point[3]);
  if (abs(yaw) > 0.01) {
    double cx = object.x;
    double cy = object.y;

    double cosYaw = std::cos(yaw);
    double sinYaw = std::sin(yaw);
    double dcx = -cx * cosYaw + cy * sinYaw + cx;
    double dcy = -cx * sinYaw - cy * cosYaw + cy;
    for (size_t i = 0; i < 4; ++i) {
      PointRotate(cosYaw, sinYaw, dcx, dcy, point[i]);
    }
  }
  geometry::Point* contour_p;
  for (size_t i = 0; i < 4; ++i) {
    contour_p = obj->add_contour();
    contour_p->set_x(point[i].x());
    contour_p->set_y(point[i].y());
    contour_p->set_z(point[i].z());
  }
}

void SelfVehIdentIOU::PointRotate(const double cosYaw, const double sinYaw, const double dcx,
                                  const double dcy, geometry::Point& point) {
  double px = point.x();
  double py = point.y();
  point.set_x(px * cosYaw - py * sinYaw + dcx);
  point.set_y(px * sinYaw + py * cosYaw + dcy);
}

void SelfVehIdentIOU::SetPointLFD(const perception::fusion::point& object, double len, double width,
                                  geometry::Point& point) {
  point.set_x(object.x - len / 2.);
  point.set_y(object.y - width / 2.);
  point.set_z(0.);
}
void SelfVehIdentIOU::SetPointLBD(const perception::fusion::point& object, double len, double width,
                                  geometry::Point& point) {
  point.set_x(object.x - len / 2.);
  point.set_y(object.y + width / 2.);
  point.set_z(0.);
}
void SelfVehIdentIOU::SetPointRFD(const perception::fusion::point& object, double len, double width,
                                  geometry::Point& point) {
  point.set_x(object.x + len / 2.);
  point.set_y(object.y - width / 2.);
  point.set_z(0.);
}
void SelfVehIdentIOU::SetPointRBD(const perception::fusion::point& object, double len, double width,
                                  geometry::Point& point) {
  point.set_x(object.x + len / 2.);
  point.set_y(object.y + width / 2.);
  point.set_z(0.);
}

double SelfVehIdentIOU::ComputeIOU(perception::Object* A, perception::Object* B, double trans_x,
                                   double trans_y) {
  geos::geom::GeometryFactory::Ptr factory = geos::geom::GeometryFactory::create();
  CoordinateArraySequence* A_cas = new CoordinateArraySequence();
  for (int i = 0; i < A->contour_size(); i++) {
    ROS_DEBUG_STREAM(std::setprecision(8) << A->contour(i).x() << " " << A->contour(i).y());
    auto x = A->contour(i).x() + trans_x;
    auto y = A->contour(i).y() + trans_y;
    A_cas->add(Coordinate(x, y));
  }
  A_cas->add(Coordinate(A->contour(0).x() + trans_x, A->contour(0).y() + trans_y));  // make a ring
  LinearRing* A_lr = factory->createLinearRing(A_cas);
  CoordinateArraySequence* B_cas = new CoordinateArraySequence();
  ROS_DEBUG_STREAM("-----");
  for (int i = 0; i < B->contour_size(); i++) {
    ROS_DEBUG_STREAM(std::setprecision(8) << B->contour(i).x() << " " << B->contour(i).y());
    auto x = B->contour(i).x();
    auto y = B->contour(i).y();
    B_cas->add(Coordinate(x, y));
  }
  B_cas->add(Coordinate(B->contour(0).x(), B->contour(0).y()));
  LinearRing* B_lr = factory->createLinearRing(B_cas);
  Polygon* A_poly = factory->createPolygon(A_lr, NULL);
  Polygon* B_poly = factory->createPolygon(B_lr, NULL);
  std::unique_ptr<Geometry> inter = A_poly->intersection(B_poly);
  double area_total = A_poly->getArea() + B_poly->getArea();
  double iou = inter->getArea() / area_total;
  ROS_DEBUG_STREAM("*********");
  ROS_DEBUG_STREAM("ComputeIOU: area1 " << A_poly->getArea());
  ROS_DEBUG_STREAM("ComputeIOU: area2 " << B_poly->getArea());
  ROS_DEBUG_STREAM("ComputeIOU: iou " << iou);
  delete A_cas;
  delete B_cas;
  A_lr = nullptr;
  B_lr = nullptr;
  A_poly = nullptr;
  B_poly = nullptr;
  factory.release();
  return iou;
}

}  // namespace v2x
}  // namespace perception
