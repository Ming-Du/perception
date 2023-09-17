#pragma once

#include <ros/ros.h>
#include "common.h"
#include "common/include/log.h"

namespace perception {
namespace fusion {

inline void PerceptionMallocHost(void** ptr, size_t size, bool use_cuda) {
#if USE_GPU == 1
  if (use_cuda) {
    BASE_CUDA_CHECK(cudaMallocHost(ptr, size));
    return;
  }
#endif
  *ptr = malloc(size);
  if (!*ptr) {
    ROS_ERROR_STREAM("PerceptionMallocHost: host allocation of size " << size << " failed");
  }
}

inline void PerceptionFreeHost(void* ptr, bool use_cuda) {
#if USE_GPU == 1
  if (use_cuda) {
    BASE_CUDA_CHECK(cudaFreeHost(ptr));
    return;
  }
#endif
  free(ptr);
}

/**
 * @brief Manages memory allocation and synchronization between the host (CPU)
 *        and device (GPU).
 *
 * TODO(dox): more thorough description.
 */
class SyncedMemory {
 public:
  enum SyncedHead { UNINITIALIZED, HEAD_AT_CPU, HEAD_AT_GPU, SYNCED };

  explicit SyncedMemory(bool use_cuda);
  SyncedMemory(size_t size, bool use_cuda);
  SyncedMemory(const SyncedMemory&) = delete;
  void operator=(const SyncedMemory&) = delete;
  ~SyncedMemory();

  const void* cpu_data();
  void set_cpu_data(void* data);
  const void* gpu_data();
  void set_gpu_data(void* data);
  void* mutable_cpu_data();
  void* mutable_gpu_data();

  SyncedHead head() const { return head_; }
  void set_head(SyncedHead head) { head_ = head; }
  void set_head_gpu() { set_head(HEAD_AT_GPU); }
  void set_head_cpu() { set_head(HEAD_AT_CPU); }
  size_t size() { return size_; }

#if USE_GPU == 1
  void async_gpu_push(const cudaStream_t& stream);
#endif

 private:
  void check_device();
  void to_cpu();
  void to_gpu();

 private:
  void* cpu_ptr_;
  void* gpu_ptr_;
  size_t size_;
  SyncedHead head_;
  bool own_cpu_data_;
  bool cpu_malloc_use_cuda_;
  bool own_gpu_data_;
  int device_;
};  // class SyncedMemory

}  // namespace fusion
}  // namespace perception
