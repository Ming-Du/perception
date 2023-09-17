#pragma once

#include <vector>

#include "google/protobuf/stubs/callback.h"
#include "google/protobuf/stubs/common.h"

#include "perception/base/thread/concurrent_queue.h"
#include "perception/base/thread/mutex.h"
#include "perception/base/thread/thread.h"

namespace perception {
namespace lib {

class ThreadPoolWorker;

class ThreadPool {
 public:
  explicit ThreadPool(int num_workers);

  ~ThreadPool();

  void Start();

  void Add(google::protobuf::Closure *closure);
  void Add(const std::vector<google::protobuf::Closure *> &closures);

  int num_workers() const { return num_workers_; }

  int num_available_workers() const { return num_available_workers_; }

  ThreadPool(const ThreadPool &) = delete;
  ThreadPool &operator=(const ThreadPool &) = delete;

 private:
  friend class ThreadPoolWorker;

  std::vector<ThreadPoolWorker *> workers_;
  int num_workers_;
  int num_available_workers_;
  Mutex mutex_;

  FixedSizeConQueue<google::protobuf::Closure *> task_queue_;
  bool started_;
};

class ThreadPoolWorker : public Thread {
 public:
  explicit ThreadPoolWorker(ThreadPool *thread_pool)
      : Thread(true, "ThreadPoolWorker"), thread_pool_(thread_pool) {}

  virtual ~ThreadPoolWorker() {}

  ThreadPoolWorker(const ThreadPoolWorker &) = delete;
  ThreadPoolWorker &operator=(const ThreadPoolWorker &) = delete;

 protected:
  void Run() override;

 private:
  ThreadPool *thread_pool_;
};

}  // namespace lib
}  // namespace perception
