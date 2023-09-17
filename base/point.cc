#include "perception/base/point.h"
#include "common/include/log.h"

namespace perception {
namespace base {

PointFCloud 
ProtoPointsToPolygon(const google::protobuf::RepeatedPtrField<Point2D>& points, float height) {
  PointFCloud point_f;
  point_f.resize(points.size());
  for (int32_t idx = 0; idx < points.size(); idx++) {
    point_f.points[idx].x = points.Get(idx).x();
    point_f.points[idx].y = points.Get(idx).y();
    point_f.points[idx].z = height;
  }
  return point_f;
}

PointFCloud 
ProtoArrayToPointCloud(const google::protobuf::RepeatedField<double>& points) {
  PointFCloud cloud_f;
  if (points.size() % 3 == 0) {
    for (int32_t idx = 0; idx < points.size(); ) {
      PointF point_f;
      point_f.x = points.Get(idx++);
      point_f.y = points.Get(idx++);
      point_f.z = points.Get(idx++);
      cloud_f.push_back(point_f);
    }
  } else {
    AWARN << "point cloud data parsing not complete!!";
  }
  return cloud_f;
}

bool PointCloudToProtoArray(const PointFCloud& cloud,
                         google::protobuf::RepeatedField<double>* points) {
  for (size_t n = 0; n < cloud.size(); n++) {
    points->Add(cloud.points[n].x);
    points->Add(cloud.points[n].y);
    points->Add(cloud.points[n].z);
  }
  return true;
}

} // namespace base
} // namespace perception
