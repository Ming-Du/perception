#include "component_cluster.h"

namespace perception {
namespace mid_fusion {

ComponentCluster::ComponentCluster() {
  min_y_dist_ = -25;  //横向检测距离,-25~25
  max_y_dist_ = 25;
  min_x_dist_ = -40;  //纵向检测距离,-20~80
  max_x_dist_ = 60;

  min_cluster_points_ = 3;

  //分段聚类参数
  cluster_thr_num_ = 10;
  float dist_step = (max_y_dist_ + max_x_dist_) / cluster_thr_num_;
  float base_thr = 0.5;
  float thr_step = 0.2;
  // for(int i=0;i<cluster_thr_num_;i++)
  // {
  //   cluster_thr_[i][0] = dist_step*(i+1);
  //   cluster_thr_[i][1] = base_thr + fabs(cluster_thr_[i][0])*thr_step -
  //   fabs(cluster_thr_[0][0])*thr_step; cluster_thr_[i][1] =
  //   cluster_thr_[i][1] > 2.0 ? 1.5:cluster_thr_[i][1];
  // }

  for (int i = 0; i < 5; i++) {
    cluster_thr_[i][0] = 5 + 5 * i;
    cluster_thr_[i][1] = 0.4 + 0.2 * i;
  }

  for (int i = 5; i < 10; i++) {
    cluster_thr_[i][0] = 25 + (i - 5) * 5;
    cluster_thr_[i][1] = 2;
  }

  x_grid_res_ = (max_x_dist_ - min_x_dist_) / X_GRID_SIZE;
  y_grid_res_ = (max_y_dist_ - min_y_dist_) / Y_GRID_SIZE;
  inv_x_grid_res_ = 1 / x_grid_res_;
  inv_y_grid_res_ = 1 / y_grid_res_;

  for (int i = 0; i < X_GRID_SIZE; i++) {
    for (int j = 0; j < Y_GRID_SIZE; j++) {
      cartesian_data_[i][j] = 0;  // 全部填充初始化为0
    }
  }
}

void ComponentCluster::cluster(const pcl::PointCloud<PointT>& in_cloud,
                               vector<pcl::PointCloud<PointT>::Ptr>* cluster_cloud_ptr) {
  mapCartesianGrid(in_cloud, cartesian_data_);

  int cluster_num;
  findComponent(cartesian_data_, cluster_num);
  getClusterCloud(in_cloud, cartesian_data_, cluster_num, cluster_cloud_ptr);
}

void ComponentCluster::mapCartesianGrid(
    const pcl::PointCloud<PointT>& no_ground_cloud,
    array<array<int, X_GRID_SIZE>, Y_GRID_SIZE>& cartesian_data) {
  // 全部填充初始化为0
  for (int i = 0; i < X_GRID_SIZE; i++) {
    for (int j = 0; j < Y_GRID_SIZE; j++) {
      cartesian_data[i][j] = 0;
    }
  }

  // no_ground_cloud 映射到笛卡尔坐标系, 统计落在这个grid的有多少个点
  for (int i = 0; i < no_ground_cloud.size(); i++) {
    float x = no_ground_cloud[i].x;
    float y = no_ground_cloud[i].y;

    int x_id = floor((x - min_x_dist_) * inv_x_grid_res_);
    int y_id = floor((y - min_y_dist_) * inv_y_grid_res_);
    if (x_id < 0 || x_id >= X_GRID_SIZE || y_id < 0 || y_id >= Y_GRID_SIZE) {
      continue;
    }

    cartesian_data[x_id][y_id] = -1;
  }
}

//  对m×n网格中的每个x，y重复此过程，直到为所有非空cluster分配了ID。
void ComponentCluster::findComponent(array<array<int, X_GRID_SIZE>, Y_GRID_SIZE>& cartesian_data,
                                     int& cluster_num) {
  cluster_num = 0;

  for (int i = 0; i < X_GRID_SIZE; i++) {
    for (int j = 0; j < Y_GRID_SIZE; j++) {
      if (-1 != cartesian_data[i][j])  //有点云
      {
        continue;
      }
      vector<int> cluster_maps;
      int cnt = 0;
      cluster_maps.push_back(i * Y_GRID_SIZE + j);
      cartesian_data[i][j] = cluster_num + 1;

      while (cnt < cluster_maps.size()) {
        int x_start = cluster_maps[cnt] / Y_GRID_SIZE;
        int y_start = cluster_maps[cnt] % Y_GRID_SIZE;

        int kernel_size = getKernelSize(x_start, y_start);
        int kernel_mean = kernel_size / 2;
        for (int m = -kernel_mean; m <= kernel_mean; m++) {
          for (int n = -kernel_mean; n <= kernel_mean; n++) {
            int x_id = x_start + m;
            int y_id = y_start + n;

            if (x_id < 0 || x_id >= X_GRID_SIZE || y_id < 0 || y_id >= Y_GRID_SIZE) {
              continue;
            }

            if (-1 != cartesian_data[x_id][y_id]) {
              continue;
            }

            cartesian_data[x_id][y_id] = cluster_num + 1;
            cluster_maps.push_back(x_id * Y_GRID_SIZE + y_id);
          }
        }

        cnt++;
      }

      cluster_num++;
    }
  }
}

void ComponentCluster::getClusterCloud(
    const pcl::PointCloud<PointT>& no_ground_cloud,
    const array<array<int, X_GRID_SIZE>, Y_GRID_SIZE>& cartesian_data, const int cluster_num,
    vector<pcl::PointCloud<PointT>::Ptr>* cluster_cloud_ptr) {
  cluster_cloud_ptr->clear();

  if (cluster_num <= 0) {
    return;
  }

  vector<pcl::PointCloud<PointT>::Ptr> tmp_cluster_cloud;
  tmp_cluster_cloud.resize(cluster_num);
  for (int i = 0; i < cluster_num; i++) {
    tmp_cluster_cloud[i].reset(new pcl::PointCloud<PointT>);
  }

  for (int i = 0; i < no_ground_cloud.size(); i++) {
    float x = no_ground_cloud[i].x;
    float y = no_ground_cloud[i].y;
    float z = no_ground_cloud[i].z;

    int x_id = floor((x - min_x_dist_) * inv_x_grid_res_);
    int y_id = floor((y - min_y_dist_) * inv_y_grid_res_);

    if (x_id < 0 || x_id >= X_GRID_SIZE || y_id < 0 || y_id >= Y_GRID_SIZE) {
      continue;
    }

    int cluster_id = cartesian_data[x_id][y_id];

    if (cluster_id <= 0 || cluster_id > cluster_num) {
      continue;
    }

    tmp_cluster_cloud[cluster_id - 1]->push_back(no_ground_cloud[i]);
  }

  // cluster_cloud_ptr->resize(cluster_num);
  // for(int i=0;i<cluster_num;i++)
  // {
  //   cluster_cloud_ptr->at(i).reset(new pcl::PointCloud<PointT>);
  // }

  for (int i = 0; i < cluster_num; i++) {
    if (tmp_cluster_cloud[i]->size() > min_cluster_points_) {
      cluster_cloud_ptr->push_back(tmp_cluster_cloud[i]);
    }
  }
}

int ComponentCluster::getKernelSize(int x_id, int y_id) {
  float range = fabs(x_id * x_grid_res_ + min_x_dist_) + fabs(y_id * y_grid_res_ + min_y_dist_);
  float cluster_dist = -1;
  int kernel = 0;

  for (int i = 0; i < cluster_thr_num_; i++) {
    if (range < cluster_thr_[i][0]) {
      cluster_dist = cluster_thr_[i][1];
      break;
    }
  }

  cluster_dist = cluster_dist > 0 ? cluster_dist : cluster_thr_[cluster_thr_num_ - 1][1];
  kernel = 2 * std::ceil(cluster_dist / y_grid_res_) + 1;

  return kernel;
}

}  // namespace mid_fusion
}  // namespace perception