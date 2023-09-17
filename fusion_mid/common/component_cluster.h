#ifndef COMPONENT_CLUSTER_H
#define COMPONENT_CLUSTER_H
#include <pcl/io/pcd_io.h>

#include <array>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <string>
#include <vector>

#include "cluster_interface.h"

using namespace std;

namespace perception {
namespace mid_fusion {

#define X_GRID_SIZE 200
#define Y_GRID_SIZE 200

class ComponentCluster : public ClusterInterface {
  using PointT = pcl::PointXYZ;

 public:
  ComponentCluster();
  ~ComponentCluster() = default;
  void cluster(const pcl::PointCloud<PointT>& in_cloud,
               vector<pcl::PointCloud<PointT>::Ptr>* cluster_cloud_ptr) override;

 private:
  float min_x_dist_;  //横向检测距离,-25~25
  float max_x_dist_;
  float min_y_dist_;  //纵向检测距离,-20~80
  float max_y_dist_;
  float x_grid_res_;
  float y_grid_res_;
  float inv_x_grid_res_;
  float inv_y_grid_res_;

  int min_cluster_points_;

  float cluster_thr_[10][2];  // 10m一个阈值
  int cluster_thr_num_;
  array<array<int, X_GRID_SIZE>, Y_GRID_SIZE> cartesian_data_;

  void mapCartesianGrid(const pcl::PointCloud<PointT>& no_ground_cloud,
                        array<array<int, X_GRID_SIZE>, Y_GRID_SIZE>& cartesian_data);
  //  对m×n网格中的每个x，y重复此过程，直到为所有非空cluster分配了ID。
  void findComponent(array<array<int, X_GRID_SIZE>, Y_GRID_SIZE>& cartesian_data, int& cluster_num);

  void getClusterCloud(const pcl::PointCloud<PointT>& no_ground_cloud,
                       const array<array<int, X_GRID_SIZE>, Y_GRID_SIZE>& cartesian_data,
                       const int cluster_num,
                       vector<pcl::PointCloud<PointT>::Ptr>* cluster_cloud_ptr);
  int getKernelSize(int x_id, int y_id);
};

typedef boost::shared_ptr<ComponentCluster> ComponentClusterPtr;
}  // namespace mid_fusion
}  // namespace perception
#endif  // COMPONENT_CLUSTER_H