#ifndef CLUSTER_INTERFACE_HEAD_
#define CLUSTER_INTERFACE_HEAD_

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/shared_ptr.hpp>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

namespace perception {
namespace mid_fusion {

class ClusterInterface {
  using PointT = pcl::PointXYZ;

 public:
  virtual ~ClusterInterface() = default;

  virtual void cluster(const pcl::PointCloud<PointT>& in_cloud,
                       vector<pcl::PointCloud<PointT>::Ptr>* cluster_cloud_ptr) = 0;
};

typedef boost::shared_ptr<ClusterInterface> ClusterInterfacePtr;
}  // namespace mid_fusion
}  // namespace perception

#endif