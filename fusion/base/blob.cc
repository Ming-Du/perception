#include "blob.h"
#include <limits>

namespace perception {
namespace fusion {

template <typename Dtype>
void Blob<Dtype>::Reshape(const int num, const int channels, const int height, const int width) {
  std::vector<int> shape(4);
  shape[0] = num;
  shape[1] = channels;
  shape[2] = height;
  shape[3] = width;
  Reshape(shape);
}

template <typename Dtype>
void Blob<Dtype>::Reshape(const std::vector<int>& shape) {
  CHECK_LE(shape.size(), kMaxBlobAxes);
  count_ = 1;
  shape_.resize(shape.size());
  if (!shape_data_ || shape_data_->size() < shape.size() * sizeof(int)) {
    shape_data_.reset(new SyncedMemory(shape.size() * sizeof(int), use_cuda_host_malloc_));
  }
  int* shape_data = static_cast<int*>(shape_data_->mutable_cpu_data());
  for (size_t i = 0; i < shape.size(); ++i) {
    CHECK_GE(shape[i], 0);
    if (count_ != 0) {
      CHECK_LE(shape[i], std::numeric_limits<int>::max() / count_)
          << "blob size exceeds std::numeric_limits<int>::max()";
    }
    count_ *= shape[i];
    shape_[i] = shape[i];
    shape_data[i] = shape[i];
  }
  if (count_ > capacity_) {
    capacity_ = count_;
    data_.reset(new SyncedMemory(capacity_ * sizeof(Dtype), use_cuda_host_malloc_));
  }
}

template <typename Dtype>
void Blob<Dtype>::ReshapeLike(const Blob<Dtype>& other) {
  Reshape(other.shape());
}

template <typename Dtype>
Blob<Dtype>::Blob(const int num,
                  const int channels,
                  const int height,
                  const int width,
                  const bool use_cuda_host_malloc)
    // capacity_ must be initialized before calling Reshape
    : capacity_(0), use_cuda_host_malloc_(use_cuda_host_malloc) {
  Reshape(num, channels, height, width);
}

template <typename Dtype>
Blob<Dtype>::Blob(const std::vector<int>& shape, const bool use_cuda_host_malloc)
    // capacity_ must be initialized before calling Reshape
    : capacity_(0), use_cuda_host_malloc_(use_cuda_host_malloc) {
  Reshape(shape);
}

template <typename Dtype>
const int* Blob<Dtype>::gpu_shape() const {
  if (!shape_data_) {
    ROS_ERROR("gpu_shape: gpu shape data is empty.");
  }
  return (const int*)shape_data_->gpu_data();
}

template <typename Dtype>
const Dtype* Blob<Dtype>::cpu_data() const {
  if (!data_) {
    ROS_ERROR("cpu_data: cpu data is empty.");
  }
  return (const Dtype*)data_->cpu_data();
}

template <typename Dtype>
void Blob<Dtype>::set_cpu_data(Dtype* data) {
  if (!data) {
    ROS_ERROR("set_cpu_data: set cpu data is empty.");
  }
  // Make sure CPU and GPU sizes remain equal
  size_t size = count_ * sizeof(Dtype);
  if (data_->size() != size) {
    data_.reset(new SyncedMemory(size, use_cuda_host_malloc_));
  }
  data_->set_cpu_data(data);
}

template <typename Dtype>
const Dtype* Blob<Dtype>::gpu_data() const {
  if (!data_) {
    ROS_ERROR("gpu_data: gpu data is empty.");
  }
  return (const Dtype*)data_->gpu_data();
}

template <typename Dtype>
void Blob<Dtype>::set_gpu_data(Dtype* data) {
  if (!data) {
    ROS_ERROR("set_gpu_data: set gpu data is empty.");
  }
  // Make sure CPU and GPU sizes remain equal
  size_t size = count_ * sizeof(Dtype);
  if (data_->size() != size) {
    data_.reset(new SyncedMemory(size, use_cuda_host_malloc_));
  }
  data_->set_gpu_data(data);
}

template <typename Dtype>
Dtype* Blob<Dtype>::mutable_cpu_data() {
  if (!data_) {
    ROS_ERROR("mutable_cpu_data: mutable_cpu_data is empty.");
  }
  return static_cast<Dtype*>(data_->mutable_cpu_data());
}

template <typename Dtype>
Dtype* Blob<Dtype>::mutable_gpu_data() {
  if (!data_) {
    ROS_ERROR("mutable_gpu_data: mutable_gpu_data is empty.");
  }
  return static_cast<Dtype*>(data_->mutable_gpu_data());
}

template <typename Dtype>
void Blob<Dtype>::ShareData(const Blob& other) {
  CHECK_EQ(count_, other.count());
  data_ = other.data();
}

template class Blob<bool>;
template class Blob<uint8_t>;
template class Blob<int>;
template class Blob<unsigned int>;
template class Blob<float>;
template class Blob<double>;

}  // namespace fusion
}  // namespace perception
