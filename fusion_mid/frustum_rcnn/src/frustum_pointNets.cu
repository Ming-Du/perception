#include "frustum_pointNets.h"

namespace frcnn {

// pcd_xyzi, N * 4, 4 -> (x, y, z, i)
// box_det2d, M * 7, 7 -> (cam_id, cx, cy, w, h, score, cls)
// cam_extr, cam_num * 4 * 4
// cam_intr, cam_num * 3 * 3
// cam_hw, cam_num * 2
// pc_cam, N * cam_num * 2, 2 -> (u, v)
// pred_box3d, M * 9, 9 -> (cx, cy, cz, sx, sy, sz, yaw, score, cls)
// box_info, (M, 2)

//====================== set_val_cuda start ================================
template <typename T>
__global__ void set_val_cuda_kernel(T* data, int length, T val) {
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < length; i = i + blockDim.x * gridDim.x) {
    data[i] = val;
  }
}

void set_val_cuda(float* data, int length, float val) {
  set_val_cuda_kernel<float><<<BLOCKS(length), THREADS>>>(data, length, val);
}
//====================== set_val_cuda end ================================

//====================== project_2d_cuda start ================================
__global__ void project_2d_cuda_kernel(const float* pcd_xyzi, const float* cam_extr,
                                       const float* cam_intr, const int32_t* cam_hw,
                                       float* pc_cam_uv, int N, int cam_num, int loop) {
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < loop; i = i + blockDim.x * gridDim.x) {
    int n = i / cam_num;
    int cam_id = i - n * cam_num;

    int pc_idx0 = n * 4;
    int cam_extr_idx0 = cam_id * 4 * 4;
    int cam_intr_idx0 = cam_id * 3 * 3;
    int pc_cam_uv_idx0 = n * cam_num * 2 + cam_id * 2;

    float x = pcd_xyzi[pc_idx0];
    float y = pcd_xyzi[pc_idx0 + 1];
    float z = pcd_xyzi[pc_idx0 + 2];
    float intensity = pcd_xyzi[pc_idx0 + 3];

    int32_t cam_h = cam_hw[cam_id * 2];
    int32_t cam_w = cam_hw[cam_id * 2 + 1];

    pc_cam_uv[pc_cam_uv_idx0] = -1;
    pc_cam_uv[pc_cam_uv_idx0 + 1] = -1;
    if (intensity >= 0) {
      float x_cam = cam_extr[cam_extr_idx0] * x + cam_extr[cam_extr_idx0 + 1] * y +
                    cam_extr[cam_extr_idx0 + 2] * z + cam_extr[cam_extr_idx0 + 3];
      float y_cam = cam_extr[cam_extr_idx0 + 4] * x + cam_extr[cam_extr_idx0 + 5] * y +
                    cam_extr[cam_extr_idx0 + 6] * z + cam_extr[cam_extr_idx0 + 7];
      float z_cam = cam_extr[cam_extr_idx0 + 8] * x + cam_extr[cam_extr_idx0 + 9] * y +
                    cam_extr[cam_extr_idx0 + 10] * z + cam_extr[cam_extr_idx0 + 11];
      if (z_cam > 0.1) {
        float x_2d = cam_intr[cam_intr_idx0] * x_cam + cam_intr[cam_intr_idx0 + 1] * y_cam +
                     cam_intr[cam_intr_idx0 + 2] * z_cam;
        float y_2d = cam_intr[cam_intr_idx0 + 3] * x_cam + cam_intr[cam_intr_idx0 + 4] * y_cam +
                     cam_intr[cam_intr_idx0 + 5] * z_cam;
        float z_2d = cam_intr[cam_intr_idx0 + 6] * x_cam + cam_intr[cam_intr_idx0 + 7] * y_cam +
                     cam_intr[cam_intr_idx0 + 8] * z_cam;
        if (z_2d != 0) {
          x_2d /= z_2d;
          y_2d /= z_2d;
          if ((x_2d >= 0) && (x_2d < cam_w) && (y_2d >= 0) && (y_2d < cam_h)) {
            pc_cam_uv[pc_cam_uv_idx0] = x_2d;
            pc_cam_uv[pc_cam_uv_idx0 + 1] = y_2d;
          }
        }
      }
    }
  }
}

void project_2d_cuda(const float* pcd_xyzi, const float* cam_extr, const float* cam_intr,
                     const int32_t* cam_hw, float* pc_cam_uv, int N, int cam_num) {
  int loop = N * cam_num;
  project_2d_cuda_kernel<<<BLOCKS(loop), THREADS>>>(pcd_xyzi, cam_extr, cam_intr, cam_hw, pc_cam_uv,
                                                    N, cam_num, loop);
}
//====================== project_2d_cuda end ================================

//====================== get_frustum_cuda start ================================
// __global__ void get_frustum_cuda_kernel(const float* pcd_xyzi, const float* pc_cam_uv,
__global__ void get_frustum_cuda_kernel(const float* pcd_xyzi, const float* pc_cam_uv,
                                        const int32_t* shuffle_index, const float* box_det2d,
                                        const float* cam_extr_inv, const float* cam_intr_inv,
                                        float* box_info, float* frustum_pcd_xyzi, int N,
                                        int cam_num, int M, float enlarge_size_w,
                                        float enlarge_size_h, int frustum_point_num_lg2) {
  int m = blockIdx.x;
  int frustum_point_num = 1<<frustum_point_num_lg2;
  __shared__ float count[1];
  __shared__ float frustum_angle[1];
  __shared__ float box_det2d_tmp[7];
  __shared__ float box_corners_enlarge[4];
  __shared__ float frustum_angle_cs[2];

  // box level
  if (threadIdx.x == 0) {
    count[0] = 0;
    box_info[m * 2] = -1.0;

    box_det2d_tmp[0] = box_det2d[m * 7];      // cam_id
    box_det2d_tmp[1] = box_det2d[m * 7 + 1];  // cx
    box_det2d_tmp[2] = box_det2d[m * 7 + 2];  // cy
    box_det2d_tmp[3] = box_det2d[m * 7 + 3];  // w
    box_det2d_tmp[4] = box_det2d[m * 7 + 4];  // h
    box_det2d_tmp[5] = box_det2d[m * 7 + 5];  // score
    box_det2d_tmp[6] = box_det2d[m * 7 + 6];  // cls

    box_corners_enlarge[0] = box_det2d_tmp[1] - (box_det2d_tmp[3] * enlarge_size_w) / 2.0f;
    box_corners_enlarge[2] = box_det2d_tmp[1] + (box_det2d_tmp[3] * enlarge_size_w) / 2.0f;
    box_corners_enlarge[1] = box_det2d_tmp[2] - (box_det2d_tmp[4] * enlarge_size_h) / 2.0f;
    box_corners_enlarge[3] = box_det2d_tmp[2] + (box_det2d_tmp[4] * enlarge_size_h) / 2.0f;

    // image space
    float box_center_z = 500;
    float box_center_x = box_center_z * box_det2d_tmp[1];
    float box_center_y = box_center_z * box_det2d_tmp[2];
    int cam_id = static_cast<int>(box_det2d_tmp[0] + 0.001f);
    int cam_intr_inv_idx0 = cam_id * 3 * 3;
    int cam_extr_inv_idx0 = cam_id * 4 * 4;

    // camera space
    float box_center_x_cam = cam_intr_inv[cam_intr_inv_idx0] * box_center_x +
                             cam_intr_inv[cam_intr_inv_idx0 + 1] * box_center_y +
                             cam_intr_inv[cam_intr_inv_idx0 + 2] * box_center_z;

    float box_center_y_cam = cam_intr_inv[cam_intr_inv_idx0 + 3] * box_center_x +
                             cam_intr_inv[cam_intr_inv_idx0 + 4] * box_center_y +
                             cam_intr_inv[cam_intr_inv_idx0 + 5] * box_center_z;

    float box_center_z_cam = cam_intr_inv[cam_intr_inv_idx0 + 6] * box_center_x +
                             cam_intr_inv[cam_intr_inv_idx0 + 7] * box_center_y +
                             cam_intr_inv[cam_intr_inv_idx0 + 8] * box_center_z;

    // imu space
    float box_center_x_imu = cam_extr_inv[cam_extr_inv_idx0] * box_center_x_cam +
                             cam_extr_inv[cam_extr_inv_idx0 + 1] * box_center_y_cam +
                             cam_extr_inv[cam_extr_inv_idx0 + 2] * box_center_z_cam +
                             cam_extr_inv[cam_extr_inv_idx0 + 3];

    float box_center_y_imu = cam_extr_inv[cam_extr_inv_idx0 + 4] * box_center_x_cam +
                             cam_extr_inv[cam_extr_inv_idx0 + 5] * box_center_y_cam +
                             cam_extr_inv[cam_extr_inv_idx0 + 6] * box_center_z_cam +
                             cam_extr_inv[cam_extr_inv_idx0 + 7];

    frustum_angle[0] = atan2f(box_center_y_imu, box_center_x_imu);
    frustum_angle_cs[0] = cos(frustum_angle[0]);
    frustum_angle_cs[1] = sin(frustum_angle[0]);
  }
  __syncthreads();

  float* frustum_pcd_xyzi_base = frustum_pcd_xyzi + m * 16 * frustum_point_num;
  // point level
  for (int n = threadIdx.x; n < N; n = n + blockDim.x) {
    int pc_idx0 = shuffle_index[n] * 4;
    int cam_id = static_cast<int>(box_det2d_tmp[0] + 0.001f);
    int pc_cam_uv_idx0 = shuffle_index[n] * cam_num * 2 + cam_id * 2;

    int box_cls = static_cast<int>(box_det2d_tmp[6] + 0.001f);
    float intensity = pcd_xyzi[pc_idx0 + 3];

    float x_2d = pc_cam_uv[pc_cam_uv_idx0];
    float y_2d = pc_cam_uv[pc_cam_uv_idx0 + 1];
    if ((intensity >= 0) && (x_2d >= 0) && (y_2d >= 0)) {
      if ((x_2d >= box_corners_enlarge[0]) && (x_2d <= box_corners_enlarge[2]) &&
          (y_2d >= box_corners_enlarge[1]) && (y_2d <= box_corners_enlarge[3])) {
        float old = atomicAdd(count, 1.0f);
        int valid_point_num = static_cast<int>(old + 0.001f);
        if (valid_point_num < frustum_point_num) {
          float x = pcd_xyzi[pc_idx0];
          float y = pcd_xyzi[pc_idx0 + 1];
          float z = pcd_xyzi[pc_idx0 + 2];
          float* frustum_pcd_xyzi_cur = frustum_pcd_xyzi_base + valid_point_num;
          frustum_pcd_xyzi_cur[0] =
              x * frustum_angle_cs[0] + y * frustum_angle_cs[1];
          frustum_pcd_xyzi_cur[1 << frustum_point_num_lg2] =
              y * frustum_angle_cs[0] - x * frustum_angle_cs[1];
          frustum_pcd_xyzi_cur[2 << frustum_point_num_lg2] = z;
          frustum_pcd_xyzi_cur[3 << frustum_point_num_lg2] = intensity;
          // class info
          frustum_pcd_xyzi_cur[15 << frustum_point_num_lg2] = 1;
        }
      }
    }
  }
  __syncthreads();
  if (threadIdx.x == 0) {
    box_info[m * 2] = count[0];
    box_info[m * 2 + 1] = frustum_angle[0];
  }
  __syncthreads();
  // padding
  int count_num = static_cast<int>(count[0] + 0.001f);
  int padding_length = frustum_point_num - count_num;
  if (padding_length > 0 && count_num > 0) {
    for (int n = threadIdx.x; n < padding_length; n = n + blockDim.x) {
      int frustum_pcd_xyzi_idx0 = m * 16 * frustum_point_num + n + count_num;
      // If count_num = 0.it will meet errors..
      int frustum_pcd_xyzi_copy_idx0 = m * 16 * frustum_point_num + (n % count_num);
      for (int64_t c = 0; c < 16; c++) {
        frustum_pcd_xyzi[frustum_pcd_xyzi_idx0 + c * frustum_point_num] =
            frustum_pcd_xyzi[frustum_pcd_xyzi_copy_idx0 + c * frustum_point_num];
      }
    }
  }
}

//====================== get_vegetation_frustum_cuda start ================================

__global__ void get_vegetation_frustum_cuda_kernal(const float* pcd_xyzi,
                                                   const float* vegetation_candidate_cube_gpu,
                                                   const int32_t* shuffle_index,
                                                   float* frustum_pcd_xyzi,
                                                   float* box_info,
                                                   int N,
                                                   int M,
                                                   int vegetation_candidate_cnt,
                                                   int frustum_point_num) {
  int candidate_index = blockIdx.x + M;
  int vegetation_index = blockIdx.x;
  __shared__ float count[1];
  __shared__ float frustum_angle[1];
  __shared__ float box_area_3d[6];
  __shared__ float frustum_angle_cs[2];

  if (threadIdx.x == 0) {
    box_area_3d[0] = vegetation_candidate_cube_gpu[vegetation_index * 6];
    box_area_3d[1] = vegetation_candidate_cube_gpu[vegetation_index * 6 + 1];
    box_area_3d[2] = vegetation_candidate_cube_gpu[vegetation_index * 6 + 2];
    box_area_3d[3] = vegetation_candidate_cube_gpu[vegetation_index * 6 + 3];
    box_area_3d[4] = vegetation_candidate_cube_gpu[vegetation_index * 6 + 4];
    box_area_3d[5] = vegetation_candidate_cube_gpu[vegetation_index * 6 + 5];

    frustum_angle[0] = atan2f((box_area_3d[2] + box_area_3d[3]) / 2.0,
                              (box_area_3d[0] + box_area_3d[1]) / 2.0);
    frustum_angle_cs[0] = cos(frustum_angle[0]);
    frustum_angle_cs[1] = sin(frustum_angle[0]);

    count[0] = 0;
    box_info[candidate_index * 2] = -1.0;
  }
  __syncthreads();
  for (int n = threadIdx.x; n < N; n = n + blockDim.x) {
    int pc_idx0 = shuffle_index[n] * 4;
    float x = pcd_xyzi[pc_idx0];
    float y = pcd_xyzi[pc_idx0 + 1];
    float z = pcd_xyzi[pc_idx0 + 2];
    float intensity = pcd_xyzi[pc_idx0 + 3];
    bool is_in_3d_box = (x >= box_area_3d[0]) && (x <= box_area_3d[1]) &&
                        (y >= box_area_3d[2]) && (y <= box_area_3d[3]) &&
                        (z >= box_area_3d[4]) && (z <= box_area_3d[5]);
    if (is_in_3d_box && intensity > 0.0) {
      float old = atomicAdd(count, 1.0f);
      int valid_point_num = static_cast<int>(old + 0.001f);
      if (valid_point_num < frustum_point_num) {
        int frustum_pcd_xyzi_idx0 = candidate_index * 16 * frustum_point_num + valid_point_num;
        frustum_pcd_xyzi[frustum_pcd_xyzi_idx0 + 0 * frustum_point_num] =
            x * frustum_angle_cs[0] + y * frustum_angle_cs[1];
        frustum_pcd_xyzi[frustum_pcd_xyzi_idx0 + 1 * frustum_point_num] =
            x * (-1 * frustum_angle_cs[1]) + y * frustum_angle_cs[0];
        frustum_pcd_xyzi[frustum_pcd_xyzi_idx0 + 2 * frustum_point_num] = z;
        frustum_pcd_xyzi[frustum_pcd_xyzi_idx0 + 3 * frustum_point_num] = intensity;

        // class info
        frustum_pcd_xyzi[frustum_pcd_xyzi_idx0 + 15 * frustum_point_num] = 1;
      }
    }
  }

  __syncthreads();
  if (threadIdx.x == 0) {
    box_info[candidate_index * 2] = count[0];
    box_info[candidate_index * 2 + 1] = frustum_angle[0];
  }

  __syncthreads();
  // padding
  int count_num = static_cast<int>(count[0] + 0.001f);
  int padding_length = frustum_point_num - count_num;
  if (padding_length > 0 && count_num > 0) {
    for (int n = threadIdx.x; n < padding_length; n = n + blockDim.x) {
      int frustum_pcd_xyzi_idx0 = candidate_index * 16 * frustum_point_num + n + count_num;
      // If count_num = 0.it will meet errors..
      int frustum_pcd_xyzi_copy_idx0 = candidate_index * 16 * frustum_point_num + (n % count_num);
      for (int64_t c = 0; c < 16; c++) {
        frustum_pcd_xyzi[frustum_pcd_xyzi_idx0 + c * frustum_point_num] =
            frustum_pcd_xyzi[frustum_pcd_xyzi_copy_idx0 + c * frustum_point_num];
      }
    }
  }
}

void get_frustum_cuda(const float* pcd_xyzi,
                      const float* pc_cam_uv,
                      const int32_t* shuffle_index,
                      const float* box_det2d,
                      const float* cam_extr_inv,
                      const float* cam_intr_inv,
                      float* box_info,
                      float* frustum_pcd_xyzi,
                      int N,
                      int cam_num,
                      int M,
                      float enlarge_size_w,
                      float enlarge_size_h,
                      int frustum_point_num,
                      const float* vegetation_candidate_cube_gpu,
                      int vegetation_candidate_cnt) {
  if (M > 0) {
    int frustum_point_num_lg2 = __builtin_ctz(frustum_point_num);
    get_frustum_cuda_kernel<<<M, 512>>>(pcd_xyzi, pc_cam_uv, shuffle_index, box_det2d, cam_extr_inv,
                                        cam_intr_inv, box_info, frustum_pcd_xyzi, N, cam_num, M,
                                        enlarge_size_w, enlarge_size_h, frustum_point_num_lg2);
  }

  if (vegetation_candidate_cnt > 0) {
    get_vegetation_frustum_cuda_kernal<<<vegetation_candidate_cnt, 512>>>(
        pcd_xyzi, vegetation_candidate_cube_gpu, shuffle_index, frustum_pcd_xyzi, box_info, N, M,
        vegetation_candidate_cnt, frustum_point_num);
  }
}
  //====================== get_frustum_cuda end ================================

}  // namespace frcnn