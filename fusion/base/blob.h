#pragma once

#include <memory>
#include <string>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"

#include <ros/ros.h>
#include "syncedmem.h"
#include "common/include/log.h"

namespace perception {
namespace fusion {

constexpr size_t kMaxBlobAxes = 32;

/**
 * @brief A wrapper around SyncedMemory holders serving as the basic
 *        computational unit for images, feature maps, etc.
 *
 * TODO(dox): more thorough description.
 */
template <typename Dtype>
class Blob {
 public:
  Blob() : data_(), count_(0), capacity_(0), use_cuda_host_malloc_(false) {}
  explicit Blob(bool use_cuda_host_malloc)
      : data_(), count_(0), capacity_(0), use_cuda_host_malloc_(use_cuda_host_malloc) {}

  /// @brief Deprecated; use <code>Blob(const std::vector<int>& shape)</code>.
  Blob(const int num,
       const int channels,
       const int height,
       const int width,
       const bool use_cuda_host_malloc = false);
  explicit Blob(const std::vector<int>& shape, const bool use_cuda_host_malloc = false);

  Blob(const Blob&) = delete;
  void operator=(const Blob&) = delete;

  /// @brief Deprecated; use `Reshape(const std::vector<int>& shape)`.
  void Reshape(const int num, const int channels, const int height, const int width);
  /**
   * @brief Change the dimensions of the blob, allocating new memory if
   *        necessary.
   *
   * This function can be called both to create an initial allocation
   * of memory, and to adjust the dimensions of a top blob during Layer::Reshape
   * or Layer::Forward. When changing the size of blob, memory will only be
   * reallocated if sufficient memory does not already exist, and excess memory
   * will never be freed.
   *
   * Note that reshaping an input blob and immediately calling Net::Backward is
   * an error; either Net::Forward or Net::Reshape need to be called to
   * propagate the new input shape to higher layers.
   */
  void Reshape(const std::vector<int>& shape);
  void ReshapeLike(const Blob& other);
  inline std::string shape_string() const {
    // return shape_.empty()
    //            ? absl::StrCat("(", count_, ")")
    //            : absl::StrCat(absl::StrJoin(shape_, " "), " (", count_, ")");
    return "";
  }
  inline const std::vector<int>& shape() const { return shape_; }
  /**
   * @brief Returns the dimension of the index-th axis (or the negative index-th
   *        axis from the end, if index is negative).
   *
   * @param index the axis index, which may be negative as it will be
   *        "canonicalized" using CanonicalAxisIndex.
   *        Dies on out of range index.
   */
  inline int shape(int index) const { return shape_[CanonicalAxisIndex(index)]; }
  inline int num_axes() const { return static_cast<int>(shape_.size()); }
  inline int count() const { return count_; }

  /**
   * @brief Compute the volume of a slice; i.e., the product of dimensions
   *        among a range of axes.
   *
   * @param start_axis The first axis to include in the slice.
   *
   * @param end_axis The first axis to exclude from the slice.
   */
  inline int count(int start_axis, int end_axis) const {
    CHECK_LE(start_axis, end_axis);
    CHECK_GE(start_axis, 0);
    CHECK_GE(end_axis, 0);
    CHECK_LE(start_axis, num_axes());
    CHECK_LE(end_axis, num_axes());
    int count = 1;
    for (int i = start_axis; i < end_axis; ++i) {
      count *= shape(i);
    }
    return count;
  }
  /**
   * @brief Compute the volume of a slice spanning from a particular first
   *        axis to the final axis.
   *
   * @param start_axis The first axis to include in the slice.
   */
  inline int count(int start_axis) const { return count(start_axis, num_axes()); }

  /**
   * @brief create RoI Blob.
   *
   * @param roi_begin begin of roi
   * @param roi_end end of roi
   */
  // const Blob<Dtype> operator ()(const std::vector<int> &roi_start,
  //                               const std::vector<int> &roi_end) const {
  // }

  /**
   * @brief Returns the 'canonical' version of a (usually) user-specified axis,
   *        allowing for negative indexing (e.g., -1 for the last axis).
   *
   * @param axis_index the axis index.
   *        If 0 <= index < num_axes(), return index.
   *        If -num_axes <= index <= -1, return (num_axes() - (-index)),
   *        e.g., the last axis index (num_axes() - 1) if index == -1,
   *        the second to last if index == -2, etc.
   *        Dies on out of range index.
   */
  inline int CanonicalAxisIndex(int axis_index) const {
    CHECK_GE(axis_index, -num_axes()) << "axis " << axis_index << " out of range for " << num_axes()
                                      << "-D Blob with shape " << shape_string();
    CHECK_LT(axis_index, num_axes()) << "axis " << axis_index << " out of range for " << num_axes()
                                     << "-D Blob with shape " << shape_string();
    if (axis_index < 0) {
      return axis_index + num_axes();
    }
    return axis_index;
  }

  /// @brief Deprecated legacy shape accessor num: use shape(0) instead.
  inline int num() const { return LegacyShape(0); }
  /// @brief Deprecated legacy shape accessor channels: use shape(1) instead.
  inline int channels() const { return LegacyShape(1); }
  /// @brief Deprecated legacy shape accessor height: use shape(2) instead.
  inline int height() const { return LegacyShape(2); }
  /// @brief Deprecated legacy shape accessor width: use shape(3) instead.
  inline int width() const { return LegacyShape(3); }
  inline int LegacyShape(int index) const {
    CHECK_LE(num_axes(), 4) << "Cannot use legacy accessors on Blobs with > 4 axes.";
    CHECK_LT(index, 4);
    CHECK_GE(index, -4);
    if (index >= num_axes() || index < -num_axes()) {
      // Axis is out of range, but still in [0, 3] (or [-4, -1] for reverse
      // indexing) -- this special case simulates the one-padding used to fill
      // extraneous axes of legacy blobs.
      return 1;
    }
    return shape(index);
  }

  inline int offset(const int n, const int c = 0, const int h = 0, const int w = 0) const {
    CHECK_GE(n, 0);
    CHECK_LE(n, num());
    CHECK_GE(channels(), 0);
    CHECK_LE(c, channels());
    CHECK_GE(height(), 0);
    CHECK_LE(h, height());
    CHECK_GE(width(), 0);
    CHECK_LE(w, width());
    return ((n * channels() + c) * height() + h) * width() + w;
  }

  inline int offset(const std::vector<int>& indices) const {
    CHECK_LE(indices.size(), static_cast<size_t>(num_axes()));
    int offset = 0;
    for (int i = 0; i < num_axes(); ++i) {
      offset *= shape(i);
      if (static_cast<int>(indices.size()) > i) {
        CHECK_GE(indices[i], 0);
        CHECK_LT(indices[i], shape(i));
        offset += indices[i];
      }
    }
    return offset;
  }
  /**
   * @brief Copy from a source Blob.
   *
   * @param source the Blob to copy from
   * @param reshape if false, require this Blob to be pre-shaped to the shape
   *        of other (and die otherwise); if true, Reshape this Blob to other's
   *        shape if necessary
   */
  void CopyFrom(const Blob<Dtype>& source, bool reshape = false);

  inline Dtype data_at(const int n, const int c, const int h, const int w) const {
    return cpu_data()[offset(n, c, h, w)];
  }

  inline Dtype data_at(const std::vector<int>& index) const { return cpu_data()[offset(index)]; }

  inline const std::shared_ptr<SyncedMemory>& data() const {
    if (!data_) {
      ROS_ERROR("data: SyncedMemory data is empty.");
    }
    return data_;
  }

  const Dtype* cpu_data() const;
  void set_cpu_data(Dtype* data);
  const int* gpu_shape() const;
  const Dtype* gpu_data() const;
  void set_gpu_data(Dtype* data);
  Dtype* mutable_cpu_data();
  Dtype* mutable_gpu_data();
  void set_head_gpu() { data_->set_head_gpu(); }
  void set_head_cpu() { data_->set_head_cpu(); }
  SyncedMemory::SyncedHead head() const { return data_->head(); }

  /**
   * @brief Set the data_ std::shared_ptr to point to the SyncedMemory holding
   * the
   *        data_ of Blob other -- useful in Layer%s which simply perform a copy
   *        in their Forward pass.
   *
   * This deallocates the SyncedMemory holding this Blob's data_, as
   * std::shared_ptr calls its destructor when reset with the "=" operator.
   */
  void ShareData(const Blob& other);

 protected:
  std::shared_ptr<SyncedMemory> data_;
  std::shared_ptr<SyncedMemory> shape_data_;
  std::vector<int> shape_;
  int count_;
  int capacity_;
  bool use_cuda_host_malloc_;
};  // class Blob

template <typename Dtype>
using BlobPtr = std::shared_ptr<Blob<Dtype>>;
template <typename Dtype>
using BlobConstPtr = std::shared_ptr<const Blob<Dtype>>;

}  // namespace fusion
}  // namespace perception
