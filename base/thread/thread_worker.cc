#include "perception/base/thread/thread_worker.h"

namespace perception {
namespace lib {

void ThreadWorker::Bind(const std::function<bool()> &func) {
  work_func_ = func;
}

void ThreadWorker::Start() {
  if (thread_ptr_ == nullptr) {
    thread_ptr_.reset(new std::thread(&ThreadWorker::Core, this));
  }
  std::lock_guard<std::mutex> lock(mutex_);
  work_flag_ = false;
  exit_flag_ = false;
}

void ThreadWorker::WakeUp() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    work_flag_ = true;
  }
  condition_.notify_one();
}

void ThreadWorker::Join() {
  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait(lock, [&]() { return !work_flag_; });
}

void ThreadWorker::Release() {
  if (thread_ptr_ == nullptr) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    work_flag_ = true;
    exit_flag_ = true;
  }
  condition_.notify_one();
  thread_ptr_->join();
  thread_ptr_.reset(nullptr);
}

void ThreadWorker::Core() {
  while (true) {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [&]() { return work_flag_; });
    }
    if (exit_flag_) {
      break;
    }
    work_func_();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      work_flag_ = false;
    }
    condition_.notify_one();
  }
}

}  // namespace lib
}  // namespace perception
