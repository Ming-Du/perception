#pragma once

// headers in STL
#include <malloc.h>
#include <stdint.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// headers in TensorRT
#include <cuda.h>
#include <cuda_runtime.h>

#include "../trt_include/trt_run.h"

// headers in local files
#include "common.h"

#include <pcl/PCLPointCloud2.h>
#include "common/proto/object.pb.h"

namespace frcnn {

#define THREADS 1024
#define BLOCKS(N) ((N + THREADS - 1) / THREADS)

// intial tensor
void set_val_cuda(float* data, int length, float val);

void project_2d_cuda(const float* pcd_xyzi, const float* cam_extr, const float* cam_intr,
                     const int32_t* cam_hw, float* pc_cam_uv, int N, int cam_num);

void get_frustum_cuda(const float* pcd_xyzi, const float* pc_cam_uv, const int32_t* shuffle_index,
                      const float* box_det2d, const float* cam_extr_inv, const float* cam_intr_inv,
                      float* box_info, float* frustum_pcd_xyzi, int N, int cam_num, int M,
                      float enlarge_size_w, float enlarge_size_h, int frustum_point_num,
                      const float* vegetation_candidate_curb_gpu,
                      int vegetation_candidate_cnt);

class FrustumRCNN {
 public:
  FrustumRCNN() = default;
  void Init(YAML::Node pre_cfg, int cam_num, std::string model_name, std::string model_dir,
            std::string shuffle_index_file);
  ~FrustumRCNN();

  std::vector<float> pc_cam_;
  std::vector<float> pred_box3d_;
  std::vector<float> lidar_object_pred_type_;
  std::vector<std::vector<float>> frustum_2d_points_;

  void save_data(const std::string& prefix);
  void fill_input(int batch_size);
  void print_output();
  std::vector<float> GetCandidatesCube(
      const std::vector<perception::TrackedObject*>& vegetation_candidates,
      int batch_size,
      int box_2d_size);
  void inference(const std::vector<float>& pcd_xyzi, const std::vector<float>& box_det2d,
                 const std::vector<float>& cam_extr, const std::vector<float>& cam_intr,
                 const std::vector<int32_t>& cam_hw, const std::vector<float>& cam_extr_inv,
                 const std::vector<float>& cam_intr_inv,
                 const std::vector<perception::TrackedObject*>& vegetation_candidates,
                 const bool is_pub_frustum_points);
  std::shared_ptr<TrtModel> model_ptr_;
  TrtCommon::SharedBufferManager sm_;

  // pcd_xyzi, N * 4, 4 -> (x, y, z, i)
  // box_det2d, M * 7, 7 -> (cam_id, cx, cy, w, h, score, cls)
  // cam_extr, cam_num * 4 * 4
  // cam_intr, cam_num * 3 * 3
  // cam_hw, cam_num * 2
  // pc_cam, N * cam_num * 2, 2 -> (u, v)
  // pred_box3d, M * 9, 9 -> (cx, cy, cz, sx, sy, sz, yaw, score, cls)
  // frustum_pcd_xyzi, (batch_size, 16, frustum_point_num, 1)
  // box_info, (M, 2)

 private:
  float* pcd_xyzi_gpu_;
  float* pc_cam_uv_gpu_;
  int32_t* shuffle_index_gpu_;
  float* cam_extr_gpu_;
  float* cam_intr_gpu_;
  int32_t* cam_hw_gpu_;
  float* cam_extr_inv_gpu_;
  float* cam_intr_inv_gpu_;
  float* frustum_pcd_xyzi_gpu_;
  float* candidate_cube_gpu_;

  float margin_;
  float enlarge_size_w_;
  float enlarge_size_h_;
  int batch_size_;
  int frustum_point_num_;
  int cam_num_;
  int type_num_ = 13;

  int point_num_max_;
};

}  // namespace frcnn
