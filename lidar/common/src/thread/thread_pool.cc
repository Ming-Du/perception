#include "thread/thread_pool.h"

namespace robosense {

ThreadPool::ThreadPool(int num_workers)
    : num_workers_(num_workers)
    , num_available_workers_(num_workers)
    , task_queue_(num_workers)
    , started_(false) 
{
    workers_.reserve(num_workers_);
    for (int idx = 0; idx < num_workers_; ++idx) {
        ThreadPoolWorker *worker = new ThreadPoolWorker(this);
        workers_.push_back(worker);
    }
}

ThreadPool::~ThreadPool() {
    if (!started_) {
        return;
    }

    for (int idx = 0; idx < num_workers_; ++idx) {
        Add(NULL);
    }

    for (int idx = 0; idx < num_workers_; ++idx) {
        workers_[idx]->Join();
        delete workers_[idx];
    }
}

void ThreadPool::Start() {
    for (int idx = 0; idx < num_workers_; ++idx) {
        workers_[idx]->Start();
    }
    started_ = true;
}

void ThreadPool::Add(Closure *closure) { 
	task_queue_.Push(closure); 
}

void ThreadPool::Add(const std::vector<Closure *> &closures) {
    for (size_t idx = 0; idx < closures.size(); ++idx) {
        Add(closures[idx]);
    }
}

void ThreadPoolWorker::Run() {
    while (true) {
        Closure *closure = nullptr;
        thread_pool_->task_queue_.Pop(&closure);
        if (closure == nullptr) {
            break;
        }

        {
            MutexLock lock(&(thread_pool_->mutex_));
            --(thread_pool_->num_available_workers_);
        }

        closure->Run();

        {
            MutexLock lock(&(thread_pool_->mutex_));
            ++(thread_pool_->num_available_workers_);
        }
    }
}

} // namespace robosense
