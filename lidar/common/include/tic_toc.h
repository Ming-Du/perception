#ifndef RS_UTILS_TIC_TOC_H_
#define RS_UTILS_TIC_TOC_H_

#include <ctime>
#include <cstdlib>
#include <chrono>
#include <string>
#include <iostream>
#include <iomanip>
#include <ros/ros.h>

namespace robosense {
class TicToc {
  public:
    TicToc(std::string name="") : name_(name), printed(false) {
        start = std::chrono::steady_clock::now();
    }

    ~TicToc() {
        if (!printed) {
            elapsed_time();
        }
    }
    void print() {
        elapsed_time();
        printed = true;
    }

  private:
    void elapsed_time() {
        if (!enable_timer_)
            return;
        end = std::chrono::steady_clock::now();
        std::chrono::duration<double> diff = end - start;
        if (!name_.empty()) {
            ROS_INFO("%s cost time: %.2fms", name_.c_str(), diff.count() * 1000);
        }
        else {
            ROS_INFO("cost time: %.2fms", diff.count() * 1000);
        }
    }
    std::chrono::steady_clock::time_point start, end;
    std::string name_;
    bool printed;
  public:
    static bool enable_timer_;

};

} // namespace perception

#endif // RS_UTILS_TIC_TOC_H_
