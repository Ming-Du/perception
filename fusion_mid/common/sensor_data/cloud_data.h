/*
 * @Description:
 * @Author: lijun
 * @Date: 2022-08-04
 */
#ifndef MID_FUSION_SENSOR_DATA_CLOUD_DATA_HPP_
#define MID_FUSION_SENSOR_DATA_CLOUD_DATA_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace perception {
namespace mid_fusion {
class CloudData {
 public:
  using POINT = pcl::PointXYZI;
  using CLOUD = pcl::PointCloud<POINT>;
  using CLOUD_PTR = CLOUD::Ptr;

 public:
  CloudData() : cloud_ptr(new CLOUD()) {}
  // CloudData& operator=(const CloudData& b)
  // {
  // this->time = b.time;
  // *this->cloud_ptr =*b.cloud_ptr;
  // return *this;
  // }
 public:
  double time = 0.0;
  CLOUD_PTR cloud_ptr;
};
}  // namespace mid_fusion
}  // namespace perception

#endif