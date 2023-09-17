#pragma once

#include <condition_variable>
#include <functional>
#include <memory>
#include <thread>

namespace robosense {

class ThreadWorker {
public:
    ThreadWorker() = default;
    ~ThreadWorker() { Release(); }

    // bind a bool returned function, should be called before start
    void Bind(const std::function<bool()> &func);

    // start the thread loopy running
    void Start();

    // wake up the thread to do something
    void WakeUp();

    // wait util the one time execution is done
    void Join();

    // release the thread resources
    void Release();

private:
    // the main function of thread
    void Core();

private:
    std::unique_ptr<std::thread> thread_ptr_;
    std::mutex mutex_;
    std::condition_variable condition_;

    bool work_flag_ = false;
    bool exit_flag_ = true;

    std::function<bool()> work_func_;
};

} // namespace perception
