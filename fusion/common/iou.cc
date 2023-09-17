//
// Created by tiger on 23-2-10.
//
#include "iou.h"
namespace perception {
namespace fusion {
IOU::IOU() {
  heading_ = Eigen::Vector3f(0., 0., 0.);
}

IOU::IOU(const Eigen::Vector3d& center,
         const Eigen::Vector3f& size,
         const Eigen::Vector3f& dir) {
  this->center_ = center;
  this->size_ = size;
  this->heading_ = dir;
  this->heading_.normalize();
  this->angle = std::atan2(this->heading_.y(), this->heading_.x());
}


double IOU::Compute(int track1_id,
                    int track2_id,
                    const PointCloud<PointD>& fused_polygons,
                    const PointCloud<PointD>& measurement_polygons) {
  if (fused_polygons.size() < 3 || measurement_polygons.size() < 3) return 0;
  this->Compute(track1_id, track2_id, fused_polygons, measurement_polygons, 0, 0);
}

double IOU::Compute(int track_id,
                    int sensor_id,
                    const PointCloud<PointD> &fuse_polygons,
                    const PointCloud<PointD> &measurement_polygons,
                    double pos_x,
                    double pos_y) {
  geos::geom::GeometryFactory::Ptr factory = geos::geom::GeometryFactory::create();
  std::vector<Coordinate> coordinates;
  for (int i = 0; i < fuse_polygons.size(); i++) {
    auto x = fuse_polygons[i].x + pos_x;
    auto y = fuse_polygons[i].y + pos_y;
    coordinates.push_back(Coordinate(x, y));
  }
  coordinates.push_back(Coordinate(fuse_polygons[0].x + pos_x, fuse_polygons[0].y + pos_y));

  std::unique_ptr<LinearRing> fused_lr = factory->createLinearRing(std::move(coordinates));
  std::vector<Coordinate> measured_coordinates;
  for (int i = 0; i < measurement_polygons.size(); i++) {
    auto x = measurement_polygons[i].x;
    auto y = measurement_polygons[i].y;
    measured_coordinates.push_back(Coordinate(x, y));
  }
  measured_coordinates.push_back(Coordinate(measurement_polygons[0].x, measurement_polygons[0].y));
  std::unique_ptr<LinearRing> measurd_lr = factory->createLinearRing(std::move(measured_coordinates));
  if (!fused_lr->isValid() || !measurd_lr->isValid()) {
    ROS_ERROR_STREAM("fused polygon valid " << fused_lr->isValid() << ", measurement polygon valid " << measurd_lr->isValid());
    return 0.0;
  }
  std::unique_ptr<Polygon> fused_poly = factory->createPolygon(std::move(fused_lr));
  std::unique_ptr<Polygon> measurment_poly = factory->createPolygon(std::move(measurd_lr));
  std::unique_ptr<Geometry> inter = fused_poly->intersection(measurment_poly.get());
  double min_area = std::min(fused_poly->getArea(), measurment_poly->getArea());
  double area = inter->getArea() / min_area;
  return area;
}

double IOU::Compute(const PointCloud<PointD>& fuse_polygon,
                    const map::V_Points& map_polygon,
                    int iter) {
  
  if (fuse_polygon.size() < 3 || map_polygon.size() < 3) {
    return std::numeric_limits<double>::max();
  }

  std::vector<Coordinate> coordinates;
  for (const auto& p : fuse_polygon) {
    coordinates.push_back(Coordinate(p.x, p.y));
  }
  coordinates.push_back(Coordinate(fuse_polygon[0].x, fuse_polygon[0].y));

  
  std::vector<Coordinate> map_coordinates;
  for (const auto& p : map_polygon) {
    map_coordinates.push_back(Coordinate(p.x, p.y));
  }
  map_coordinates.push_back(Coordinate(map_polygon[0].x, map_polygon[0].y));

  geos::geom::GeometryFactory::Ptr factory = geos::geom::GeometryFactory::create();
  std::unique_ptr<LinearRing> fused_lr = factory->createLinearRing(std::move(coordinates));
  std::unique_ptr<LinearRing> measurd_lr = factory->createLinearRing(std::move(map_coordinates));
  std::unique_ptr<Polygon> fused_poly = factory->createPolygon(std::move(fused_lr));
  std::unique_ptr<Polygon> measurment_poly = factory->createPolygon(std::move(measurd_lr));
  std::unique_ptr<Geometry> inter = fused_poly->intersection(measurment_poly.get());
  double min_area = std::min(fused_poly->getArea(), measurment_poly->getArea());
  double area = .0;
  if (iter == 1) {//交/参数1
    area = inter->getArea() / fused_poly->getArea();
  } else if (iter == 2) {//交/参数2
    area = inter->getArea() / measurment_poly->getArea();
  } else if (iter == 3) {//交并比
    area = inter->getArea() / (measurment_poly->getArea() + fused_poly->getArea() - inter->getArea());
  } else if (iter == 4) {//交/min
    area = inter->getArea() / min_area;
  } else {
    area = inter->getArea() / fused_poly->getArea();
  }
   
  return area;
}

double IOU::Compute(int track_id,
                    int sensor_id,
                    double& fuse_area,
                    double& measurement_area,
                    const PointCloud<PointD> &fuse_polygons,
                    const PointCloud<PointD> &measurement_polygons) {
  geos::geom::GeometryFactory::Ptr factory = geos::geom::GeometryFactory::create();
  std::vector<Coordinate> coordinates;
  if (fuse_polygons.size() < 3 || measurement_polygons.size() < 3) {
      return 0.0;
  }
  for (int i = 0; i < fuse_polygons.size(); i++) {
    auto x = fuse_polygons[i].x;
    auto y = fuse_polygons[i].y;
    coordinates.push_back(Coordinate(x, y));
  }
  coordinates.push_back(Coordinate(fuse_polygons[0].x, fuse_polygons[0].y));

  std::unique_ptr<LinearRing> fused_lr = factory->createLinearRing(std::move(coordinates));
  std::vector<Coordinate> measured_coordinates;
  for (int i = 0; i < measurement_polygons.size(); i++) {
    auto x = measurement_polygons[i].x;
    auto y = measurement_polygons[i].y;
    measured_coordinates.push_back(Coordinate(x, y));
  }
  measured_coordinates.push_back(Coordinate(measurement_polygons[0].x, measurement_polygons[0].y));
  std::unique_ptr<LinearRing> measurd_lr = factory->createLinearRing(std::move(measured_coordinates));
  if (!fused_lr->isValid() || !measurd_lr->isValid()) {
    ROS_ERROR_STREAM("fused polygon valid " << fused_lr->isValid() << ", measurement polygon valid " << measurd_lr->isValid());
    return 0.0;
  }
  std::unique_ptr<Polygon> fused_poly = factory->createPolygon(std::move(fused_lr));
  std::unique_ptr<Polygon> measurment_poly = factory->createPolygon(std::move(measurd_lr));
  std::unique_ptr<Geometry> inter = fused_poly->intersection(measurment_poly.get());
  fuse_area = fused_poly->getArea();
  measurement_area = measurment_poly->getArea();
  double min_area = std::min(fuse_area, measurement_area);
  double area = inter->getArea() / min_area;
  return area;
}

}  // namespace fusion
}  // namespace perception

