#include "frustum_pointNets.h"

namespace frcnn {
constexpr int kBackgroundPointLimit = 20;
constexpr int kBackgroundIndex = 0;
constexpr double kVegeEnlargeSize = 0.5;
constexpr double kVegeBaseSize = 0.5;
void FrustumRCNN::Init(YAML::Node pre_cfg, int cam_num, std::string model_name,
                       std::string model_dir, std::string shuffle_index_file) {
  ROS_INFO_STREAM("Init: FrustumRCNN model init~!");
  // parse config
  this->margin_ = pre_cfg["margin"].as<float>();
  this->enlarge_size_w_ = pre_cfg["enlarge_size_w"].as<float>();
  this->enlarge_size_h_ = pre_cfg["enlarge_size_h"].as<float>();
  this->frustum_point_num_ = pre_cfg["frustum_point_num"].as<int>();
  this->cam_num_ = cam_num;

  // load model
  ModelParams model_args;
  model_args.modelName = model_name;
  model_args.modelDirs.push_back(model_dir);

  this->model_ptr_ = std::make_shared<TrtModel>(model_args);
  this->model_ptr_->CudaSetDeviceFlags();
  if (!this->model_ptr_->BuildTrt()) {
    ROS_WARN_STREAM("Init: build model faild");
  }
  this->model_ptr_->WarnUp();

  // load shuffle file
  std::vector<int32_t> shuffle_index;

  std::ifstream inFile;
  inFile.open(shuffle_index_file);
  if (!inFile) {
    ROS_WARN_STREAM("Init: build model faild");
  }

  int x;
  while (inFile >> x) {
    shuffle_index.push_back(static_cast<int32_t>(x));
  }
  inFile.close();

  this->point_num_max_ = shuffle_index.size();
  this->batch_size_ = this->model_ptr_->mMaxBatch;

  // allocate gpu memory
  GPU_CHECK(cudaMalloc((void**)&pcd_xyzi_gpu_, sizeof(float) * this->point_num_max_ * 4));
  GPU_CHECK(cudaMalloc((void**)&pc_cam_uv_gpu_,
                       sizeof(float) * this->point_num_max_ * this->cam_num_ * 2));
  GPU_CHECK(cudaMalloc((void**)&shuffle_index_gpu_, sizeof(int32_t) * this->point_num_max_));
  GPU_CHECK(cudaMalloc((void**)&cam_extr_gpu_, sizeof(float) * this->cam_num_ * 4 * 4));
  GPU_CHECK(cudaMalloc((void**)&cam_intr_gpu_, sizeof(float) * this->cam_num_ * 3 * 3));
  GPU_CHECK(cudaMalloc((void**)&cam_hw_gpu_, sizeof(int32_t) * this->cam_num_ * 2));
  GPU_CHECK(cudaMalloc((void**)&cam_extr_inv_gpu_, sizeof(float) * this->cam_num_ * 4 * 4));
  GPU_CHECK(cudaMalloc((void**)&cam_intr_inv_gpu_, sizeof(float) * this->cam_num_ * 3 * 3));
  GPU_CHECK(cudaMalloc((void**)&frustum_pcd_xyzi_gpu_,
                       sizeof(float) * this->batch_size_ * 16 * this->frustum_point_num_));
  GPU_CHECK(
      cudaMalloc((void**)&candidate_cube_gpu_, sizeof(float) * this->batch_size_ * 6));

  GPU_CHECK(cudaMemcpy(shuffle_index_gpu_, shuffle_index.data(),
                       sizeof(int32_t) * this->point_num_max_, cudaMemcpyHostToDevice));

  // allocate cpu memory
  pc_cam_.reserve(this->point_num_max_ * this->cam_num_ * 2);
  pred_box3d_.reserve(this->batch_size_ * 9);
}

FrustumRCNN::~FrustumRCNN() {
  // release gpu memory
  GPU_CHECK(cudaFree(pcd_xyzi_gpu_));
  GPU_CHECK(cudaFree(pc_cam_uv_gpu_));
  GPU_CHECK(cudaFree(shuffle_index_gpu_));
  GPU_CHECK(cudaFree(cam_extr_gpu_));
  GPU_CHECK(cudaFree(cam_intr_gpu_));
  GPU_CHECK(cudaFree(cam_hw_gpu_));
  GPU_CHECK(cudaFree(cam_extr_inv_gpu_));
  GPU_CHECK(cudaFree(cam_intr_inv_gpu_));
  GPU_CHECK(cudaFree(frustum_pcd_xyzi_gpu_));
  GPU_CHECK(cudaFree(candidate_cube_gpu_));
}

inline void FrustumRCNN::fill_input(int batch_size) {
  this->sm_.InitBuffer(this->model_ptr_->GetCudaEngine().get(), batch_size);
  float* input_0 = static_cast<float*>(this->sm_.getHostBuffer("input_0"));
  GPU_CHECK(cudaMemcpy(input_0, this->frustum_pcd_xyzi_gpu_,
                       sizeof(float) * batch_size * 16 * this->frustum_point_num_,
                       cudaMemcpyDeviceToHost));
}

inline void FrustumRCNN::print_output() {
  for (auto name : this->model_ptr_->GetOutputNames()) {
    auto shape = this->sm_.GetOutputDims()[name];
    std::cout << "name:" << name << ",dims:(";
    for (auto j = 0; j < shape.size(); j++) {
      if (j != 0) {
        std::cout << ",";
      }
      std::cout << shape[j];
    }
    std::cout << ")" << std::endl;
  }
}

std::vector<float> FrustumRCNN::GetCandidatesCube(
    const std::vector<perception::TrackedObject*>& candidates,
    int batch_size,
    int box_2d_size) {
  std::vector<float> candidate_cube;
  int candidate_cnt = 0;
  double enlarge_size = 0.0;
  if (box_2d_size < batch_size) {
    candidate_cnt =
        std::min(batch_size - box_2d_size, int(candidates.size()));
    for (int i = 0; i < candidate_cnt; i++) {
      const perception::Object& lidar_object = candidates[i]->obj();
      const double z_min = lidar_object.center().z() - lidar_object.size().z() * 0.5f;
      const double z_max = lidar_object.center().z() + lidar_object.size().z() * 0.5f;
      const double x_min = lidar_object.center().x() - lidar_object.size().x() * 0.5f;
      const double x_max = lidar_object.center().x() + lidar_object.size().x() * 0.5f;
      const double y_min = lidar_object.center().y() - lidar_object.size().y() * 0.5f;
      const double y_max = lidar_object.center().y() + lidar_object.size().y() * 0.5f;
      // If unknown object have small polygon, we enlarge the polygon to get enough lidar points.
      if (std::min({lidar_object.size().x(), lidar_object.size().y(), lidar_object.size().z()}) <=
          kVegeBaseSize) {
        enlarge_size = kVegeEnlargeSize;
      }

      candidate_cube.push_back(x_min - enlarge_size);
      candidate_cube.push_back(x_max + enlarge_size);
      candidate_cube.push_back(y_min - enlarge_size);
      candidate_cube.push_back(y_max + enlarge_size);
      candidate_cube.push_back(z_min - enlarge_size);
      candidate_cube.push_back(z_max + enlarge_size);

      // int point_cnt = 0;
      // for (int j = 0; j <= 200000; j++) {
      //   float x = pcd_xyzi[j*4];
      //   float y = pcd_xyzi[j*4 + 1];
      //   float z = pcd_xyzi[j*4 + 2];
      //   const float expand = 0.2;
      //   bool is_in_3d_box = (x >= (x_min - expand)) && (x <= (x_max + expand)) &&
      //                       (y >= (y_min - expand)) && (y <= (y_max + expand)) &&
      //                       (z >= (z_min - expand)) && (z <= (z_max + expand));
      //   if (is_in_3d_box) {
      //     point_cnt ++;
      //   }
      // }
      // std::cout << "id is: " << lidar_object.id() << "\n";
      // std::cout << "box points: " << point_cnt << "\n";
    }
  }
  return candidate_cube;
}

void FrustumRCNN::inference(const std::vector<float>& pcd_xyzi, const std::vector<float>& box_det2d,
                            const std::vector<float>& cam_extr, const std::vector<float>& cam_intr,
                            const std::vector<int32_t>& cam_hw,
                            const std::vector<float>& cam_extr_inv,
                            const std::vector<float>& cam_intr_inv,
                            const std::vector<perception::TrackedObject*>& candidates,
                            const bool is_pub_frustum_points)
// pcd_xyzi, N * 4, 4 -> (x, y, z, i)
// box_det2d, M * 7, 7 -> (cam_id, cx, cy, w, h, score, cls)
// cam_extr, cam_num * 4 * 4
// cam_intr, cam_num * 3 * 3
// cam_hw, cam_num * 2
// pc_cam, N * cam_num * 2, 2 -> (u, v)
// pred_box3d, M * 9, 9 -> (cx, cy, cz, sx, sy, sz, yaw, score, cls)
// frustum_pcd_xyzi, (batch_size, 16, frustum_point_num, 1)
// box_info, (M, 2)
{
  pc_cam_.clear();
  pred_box3d_.clear();
  lidar_object_pred_type_.clear();
  frustum_2d_points_.clear();

  int N = pcd_xyzi.size() / 4;
  int box_2d_size = box_det2d.size() / 7;
  ROS_DEBUG_STREAM("FrustumRCNN::inference: "
                  << "pcd xyzi N is: " << N);
  ROS_DEBUG_STREAM("FrustumRCNN::inference: "
                  << "pcd box_det2d M is: " << box_2d_size);
  // if (box_det2d.size() == 0) {
  //   return;
  // }
  if (N == 0) {
    return;
  }
  if (box_2d_size > this->batch_size_) {
    box_2d_size = this->batch_size_;
    ROS_DEBUG_STREAM("FrustumRCNN::inference: "
                     << "The number of 2D Box exceeds the max batch size: " << box_2d_size << '\t'
                     << this->batch_size_);
  }
  // Add obstacle candidate
  std::vector<float> candidate_cube =
      GetCandidatesCube(candidates, this->batch_size_, box_2d_size);
  const int candidate_cnt = candidate_cube.size() / 6;
  ROS_DEBUG_STREAM(
      "FrustumRCNN::inference: candidate cnt: " << candidate_cnt);
  auto candidate_end = std::chrono::system_clock::now();

  // copy data from cpu to gpu memory
  // debug info
  set_val_cuda(pcd_xyzi_gpu_, point_num_max_ * 4, -10.0f);
  ROS_DEBUG_STREAM("FrustumRCNN::inference: "
                  << "point_num_max_ is: " << point_num_max_);
  GPU_CHECK(
      cudaMemcpy(pcd_xyzi_gpu_, pcd_xyzi.data(), sizeof(float) * N * 4, cudaMemcpyHostToDevice));

  float* box_det2d_gpu;
  float* box_info_gpu;
  GPU_CHECK(cudaMalloc((void**)&box_det2d_gpu, sizeof(float) * box_2d_size * 7));
  GPU_CHECK(
      cudaMemcpy(box_det2d_gpu, box_det2d.data(), sizeof(float) * box_2d_size * 7, cudaMemcpyHostToDevice));

  GPU_CHECK(cudaMalloc((void**)&box_info_gpu, sizeof(float) * batch_size_ * 2));

  GPU_CHECK(cudaMemcpy(cam_extr_gpu_, cam_extr.data(), sizeof(float) * cam_num_ * 4 * 4,
                       cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(cam_intr_gpu_, cam_intr.data(), sizeof(float) * cam_num_ * 3 * 3,
                       cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(cam_hw_gpu_, cam_hw.data(), sizeof(int32_t) * cam_num_ * 2,
                       cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(cam_extr_inv_gpu_, cam_extr_inv.data(), sizeof(float) * cam_num_ * 4 * 4,
                       cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(cam_intr_inv_gpu_, cam_intr_inv.data(), sizeof(float) * cam_num_ * 3 * 3,
                       cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(candidate_cube_gpu_, candidate_cube.data(),
                       sizeof(float) * candidate_cnt * 6, cudaMemcpyHostToDevice));

  auto data_memcpy = std::chrono::system_clock::now();
  auto duration1 =
      std::chrono::duration_cast<std::chrono::microseconds>(data_memcpy - candidate_end);
  ROS_DEBUG_STREAM("FrustumRCNN::inference: data memcpy cost "
                   << double(duration1.count()) * 1000 * std::chrono::microseconds::period::num /
                          std::chrono::microseconds::period::den
                   << " ms");

  // project 2D
  cudaEvent_t start, stop, frustum, infer;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);
  cudaEventCreate(&frustum);
  cudaEventCreate(&infer);

  cudaEventRecord(start, 0);
  project_2d_cuda(pcd_xyzi_gpu_, cam_extr_gpu_, cam_intr_gpu_, cam_hw_gpu_, pc_cam_uv_gpu_,
                  point_num_max_, cam_num_);
  cudaEventRecord(stop, 0);
  cudaEventSynchronize(stop);
  float time1;
  cudaEventElapsedTime(&time1, start, stop);
  ROS_DEBUG_STREAM("FrustumRCNN::inference: project points cost time is : " << time1);
  // get frustum
  GPU_CHECK(cudaMemset(box_info_gpu, 0, sizeof(float) * batch_size_ * 2));
  GPU_CHECK(
      cudaMemset(frustum_pcd_xyzi_gpu_, 0, sizeof(float) * batch_size_ * 16 * frustum_point_num_));

  get_frustum_cuda(pcd_xyzi_gpu_, pc_cam_uv_gpu_, shuffle_index_gpu_, box_det2d_gpu,
                   cam_extr_inv_gpu_, cam_intr_inv_gpu_, box_info_gpu, frustum_pcd_xyzi_gpu_,
                   point_num_max_, cam_num_, box_2d_size, enlarge_size_w_, enlarge_size_h_,
                   frustum_point_num_, candidate_cube_gpu_, candidate_cnt);

  cudaEventRecord(frustum, 0);
  cudaEventSynchronize(frustum);
  float time2;
  cudaEventElapsedTime(&time2, stop, frustum);
  ROS_DEBUG_STREAM("FrustumRCNN::inference: get frustum time is: " << time2);
  // model inference
  ROS_DEBUG_STREAM("FrustumRCNN::inference: model inference real batchsize: "
                   << box_2d_size + candidate_cnt);
  fill_input(box_2d_size + candidate_cnt);
  if (!this->model_ptr_->infer_v2(this->sm_)) {
    ROS_ERROR_STREAM("inference: "
                     << "infer model faild");
    return;
  }
  cudaEventRecord(infer, 0);
  cudaEventSynchronize(infer);
  float time3;
  cudaEventElapsedTime(&time3, frustum, infer);
  ROS_DEBUG_STREAM("FrustumRCNN::inference: infer time is: " << time3);
  // post-process
  auto start1 = std::chrono::system_clock::now();
  float* pred_cls = static_cast<float*>(this->sm_.getHostBuffer("pred_cls"));
  float* pred_center = static_cast<float*>(this->sm_.getHostBuffer("pred_center"));
  float* pred_size = static_cast<float*>(this->sm_.getHostBuffer("pred_size"));
  float* pred_angle = static_cast<float*>(this->sm_.getHostBuffer("pred_angle"));

  std::vector<float> box_info(batch_size_ * 2, 0);
  GPU_CHECK(
      cudaMemcpy(box_info.data(), box_info_gpu, sizeof(float) * batch_size_ * 2, cudaMemcpyDeviceToHost));

  // pred_cls, (M, 13)
  // pred_center, (M, 3)
  // pred_size, (M, 13, 3)
  // pred_angle, (M, 1)
  for (int i = 0; i < box_2d_size; i++) {
    // score and label
    int label = -1;
    float max_score = -10.0f;
    if (box_info[2 * i] > 0.0) {
      for (int c = 0; c < type_num_; c++) {
        if (pred_cls[i * type_num_ + c] > max_score) {
          max_score = pred_cls[i * type_num_ + c];
          label = c;
        }
      }
    }

    // center
    float cz = pred_center[i * 3 + 2];
    float frustum_angle_c = std::cos(box_info[i * 2 + 1]);
    float frustum_angle_s = std::sin(box_info[i * 2 + 1]);
    float cx = pred_center[i * 3] * frustum_angle_c - pred_center[i * 3 + 1] * frustum_angle_s;
    float cy = pred_center[i * 3] * frustum_angle_s + pred_center[i * 3 + 1] * frustum_angle_c;

    // size
    int pred_size_idx0 = i * 12 * 3 + label * 3;
    float sx = pred_size[pred_size_idx0];
    float sy = pred_size[pred_size_idx0 + 1];
    float sz = pred_size[pred_size_idx0 + 2];

    // angle
    float yaw = pred_angle[i] + box_info[i * 2 + 1];
    this->pred_box3d_.push_back(cx);
    this->pred_box3d_.push_back(cy);
    this->pred_box3d_.push_back(cz);
    this->pred_box3d_.push_back(sx);
    this->pred_box3d_.push_back(sy);
    this->pred_box3d_.push_back(sz);
    this->pred_box3d_.push_back(yaw);
    this->pred_box3d_.push_back(max_score);
    this->pred_box3d_.push_back(label);
  }
  // get type and scores
  for (int i = box_2d_size; i < box_2d_size + candidate_cnt; i++) {
    // score and label
    int label = -1;
    float max_score = -10.0f;
    for (int c = 0; c < type_num_; c++) {
      if (pred_cls[i * type_num_ + c] > max_score) {
        max_score = pred_cls[i * type_num_ + c];
        label = c;
      }
    }
    // Obstacle points >= 20 but classified as background, invalid! Set label as -1.
    if (box_info[2 * i] > kBackgroundPointLimit && label == kBackgroundIndex) {
      label = -1;
    }
    lidar_object_pred_type_.push_back(max_score);
    lidar_object_pred_type_.push_back(label);
  }
  // get frustum points
  if (is_pub_frustum_points) {
    std::vector<float> frustum_pcd_xyzi_cpu(batch_size_ * 16 * frustum_point_num_);
    GPU_CHECK(cudaMemcpy(frustum_pcd_xyzi_cpu.data(), frustum_pcd_xyzi_gpu_,
                         sizeof(float) * batch_size_ * 16 * frustum_point_num_,
                         cudaMemcpyDeviceToHost));
    frustum_2d_points_.resize(box_2d_size);
    for (int sample_cnt = 0; sample_cnt < box_2d_size; sample_cnt++) {
      const int st = sample_cnt * 1024 * 16;
      float frustum_angle_c = std::cos(box_info[sample_cnt * 2 + 1]);
      float frustum_angle_s = std::sin(box_info[sample_cnt * 2 + 1]);

      for (int i = st; i < st + std::min(1024, int(box_info[sample_cnt * 2])); i++) {
        float pt_x = frustum_pcd_xyzi_cpu[i + 1024 * 0];
        float pt_y = frustum_pcd_xyzi_cpu[i + 1024 * 1];
        float pt_z = frustum_pcd_xyzi_cpu[i + 1024 * 2];

        float cx = pt_x * frustum_angle_c - pt_y * frustum_angle_s;
        float cy = pt_x * frustum_angle_s + pt_y * frustum_angle_c;

        pt_x = cx;
        pt_y = cy;
        frustum_2d_points_[sample_cnt].push_back(pt_x);
        frustum_2d_points_[sample_cnt].push_back(pt_y);
        frustum_2d_points_[sample_cnt].push_back(pt_z);
      }
    }
  }

  auto end1 = std::chrono::system_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end1 - start1);
  ROS_DEBUG_STREAM("FrustumRCNN::inference: postprocessing cost "
                   << double(duration.count()) * std::chrono::microseconds::period::num /
                          std::chrono::microseconds::period::den);

  // get output
  // this->pc_cam_.resize(N * this->cam_num_ * 2);
  // GPU_CHECK(cudaMemcpy(this->pc_cam_.data(), this->pc_cam_uv_gpu_,
  //                      sizeof(float) * N * this->cam_num_ * 2, cudaMemcpyDeviceToHost));

  // release gpu momery
  GPU_CHECK(cudaFree(box_det2d_gpu));
  GPU_CHECK(cudaFree(box_info_gpu));
}

void FrustumRCNN::save_data(const std::string& prefix) {
  // copy pc_cam
  std::vector<float> pc_cam_tmp(this->point_num_max_ * this->cam_num_ * 2, 0);
  GPU_CHECK(cudaMemcpy(pc_cam_tmp.data(), this->pc_cam_uv_gpu_,
                       sizeof(float) * this->point_num_max_ * this->cam_num_ * 2,
                       cudaMemcpyDeviceToHost));
  write_data<float>(pc_cam_tmp, prefix + "_pc_cam.bin");

  // copy pcds
  std::vector<float> pc_tmp(this->point_num_max_ * 4, 0);
  GPU_CHECK(cudaMemcpy(pc_tmp.data(), this->pcd_xyzi_gpu_, sizeof(float) * this->point_num_max_ * 4,
                       cudaMemcpyDeviceToHost));
  write_data<float>(pc_tmp, prefix + "_pc.bin");

  // copy model input
  std::vector<float> frustum_pcd_xyzi(this->batch_size_ * 16 * this->frustum_point_num_, 0);
  GPU_CHECK(cudaMemcpy(frustum_pcd_xyzi.data(), this->frustum_pcd_xyzi_gpu_,
                       sizeof(float) * this->batch_size_ * 16 * this->frustum_point_num_,
                       cudaMemcpyDeviceToHost));
  write_data<float>(frustum_pcd_xyzi, prefix + "_frustum_pcd_xyzi.bin");

  write_data<float>(this->pc_cam_, prefix + "_pc_cam.bin");
  write_data<float>(this->pred_box3d_, prefix + "_pred_box3d.bin");
}

}  // namespace frcnn