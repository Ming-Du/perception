#pragma once

#define PCL_NO_PRECOMPILE
#include <pcl_ros/point_cloud.h>

#include "common/proto/geometry.pb.h"

namespace perception {
namespace base {

// struct PointXYZIR
// {
//   PCL_ADD_POINT4D;                    // quad-word XYZ
//   uint16_t intensity;                 ///< laser intensity reading
//   uint16_t ring;                      ///< laser ring number
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
// } EIGEN_ALIGN16;

template<typename T> using PointCloud = ::pcl::PointCloud<T>;
// typedef pcl::PointXYZI PointD;
// typedef PointXYZIR PointF;
typedef pcl::PointXYZ PointF;
typedef ::geometry::Point2D Point2D;
typedef ::geometry::Point3D Point3D;

typedef PointCloud<PointF> PolygonFType;
typedef PointCloud<PointF> PointFCloud;
typedef std::shared_ptr<PointFCloud> PointFCloudPtr;
typedef std::shared_ptr<const PointFCloud> PointFCloudConstPtr;

// 从proto Point2D类拆包组成pointcloud
PointFCloud ProtoPointsToPolygon(const google::protobuf::RepeatedPtrField<Point2D>& points, float height = 0.f);
// 从proto数组拆包组pointcloud
PointFCloud ProtoArrayToPointCloud(const google::protobuf::RepeatedField<double>& points); 
// 转成proto数组
bool PointCloudToProtoArray(const PointFCloud& cloud,
                         google::protobuf::RepeatedField<double>* points);

} // namespace base
} // namespace perception

// POINT_CLOUD_REGISTER_POINT_STRUCT(perception::base::PointXYZIR,
//                   (float, x, x)(float, y, y)(float, z, z)
//                   (uint16_t, intensity, intensity)
//                   (uint16_t, ring, ring));
