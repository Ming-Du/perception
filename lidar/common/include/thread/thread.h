#pragma once

#include <ros/ros.h>
#include <glog/logging.h>
#include <string>

namespace robosense {

class Thread {
public:
    explicit Thread(bool joinable = false, const std::string &name = "Thread")
        : tid_(0), started_(false), joinable_(joinable), thread_name_(name) {}

    pthread_t tid() const { return tid_; }

    void set_joinable(bool joinable) {
        if (!started_) {
            joinable_ = joinable;
        }
    }

    void Start();

    void Join();

    bool IsAlive();

    std::string get_thread_name() const { return thread_name_; }
    void set_thread_name(const std::string &name) { thread_name_ = name; }

    Thread(const Thread &) = delete;
    Thread &operator=(const Thread &) = delete;

protected:
    virtual void Run() = 0;

    static void *ThreadRunner(void *arg) {
        Thread *t = reinterpret_cast<Thread *>(arg);
        t->Run();
        return nullptr;
    }

    pthread_t tid_;
    bool started_;
    bool joinable_;
    std::string thread_name_;

private:
};

} // namespace perception
