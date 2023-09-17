//
// Created by tiger on 23-2-10.
//

#ifndef PERCEPTION_FUSION2_IOU_H
#define PERCEPTION_FUSION2_IOU_H


#include <geos/geom/Coordinate.h>
#include <Eigen/Core>
#include <cmath>
#include <memory>
#include <vector>
#include "base/object.h"
#include "fstream"
#include "geos/geom/CoordinateArraySequence.h"
#include "geos/geom/Geometry.h"
#include "geos/geom/GeometryFactory.h"
#include "lib/map_server/map_server.h"
namespace perception {
namespace fusion {
using geos::geom::Coordinate;
using geos::geom::CoordinateSequence;
using geos::geom::Geometry;
using geos::geom::GeometryFactory;
using namespace geos::geom;

//typedef geos::geom::LinearRing* LinearRingPtr;

class IOU {
public:
  using ptr = std::shared_ptr<IOU>;

  IOU();

  IOU(const Eigen::Vector3d& center, const Eigen::Vector3f& size, const Eigen::Vector3f& dir);

  double Compute(Eigen::MatrixXd fused_corners,
                 Eigen::MatrixXd measurement_corners,
                 double pos_x,
                 double pos_y);
  double Compute(int track_id,
                 int sensor_id,
                 const PointCloud<PointD>& fuse_polygons,
                 const PointCloud<PointD>& measurement_polygons,
                 double pos_x,
                 double pos_y);
  double Compute(int track1_id,
                 int track2_id,
                 const PointCloud<PointD>& fused_polygons,
                 const PointCloud<PointD>& measurement_polygons);

  double Compute(const PointCloud<PointD>& fuse_polygon,
                 const map::V_Points& map_polygon,
                 int iter = 1);

  double Compute(int track_id,
                 int sensor_id,
                 double& fuse_area,
                 double& measurement_area,
                 const PointCloud<PointD> &fuse_polygons,
                 const PointCloud<PointD> &measurement_polygons);
//  Eigen::MatrixXd ObbCorners(PointCloud<PointD> polygons);

private:
  Eigen::Vector3d center_;
  Eigen::Vector3f size_, heading_;
  double angle = 0.f;
};
}  // namespace fusion
}  // namespace perception



#endif //PERCEPTION_FUSION2_IOU_H
