//
// Created by moriarty on 4/15/21.
//

#ifndef SRC_TIMER_H
#define SRC_TIMER_H
#include <chrono>
#include <string>
#include <ros/ros.h>

namespace perception {
namespace base {
class MicrosecondTimer {
public:
  // for timestamp, use GetTimeStampNow
  using Time = std::chrono::time_point<std::chrono::high_resolution_clock>;

  static Time now() {
    return std::chrono::high_resolution_clock::now();
  }

  static long long to_microsecond(const Time &begin, const Time &end) {
    return std::chrono::duration_cast<std::chrono::microseconds>(
      end - begin).count();
  }

  static const long long SecondToMicro = 1000000L;

  MicrosecondTimer() :
    ended_(0), label_("Timer") {
  }

  MicrosecondTimer(const std::string &label) : ended_(0), label_(label) {}

  MicrosecondTimer(const char *label) : ended_(0), label_(label) {}

  ~MicrosecondTimer() {}

  Time begin() {
    ended_ = 0;
    start_time_ = now();
    return start_time_;
  }

  Time begin(const char *label) {
    label_ = label;
    return begin();
  }

  long long end(bool display = true) {
    return end(label_.c_str(), display);
  }

  long long end(const char *label, bool display = true) {
    if (ended_ == 1) {
      ROS_INFO_STREAM( label << " has ended");
      return 0;
    }
    ended_ = 1;
    long long micro_sec_passed = to_microsecond(start_time_, now());
    if (display) {
      ROS_INFO_STREAM( "Timer " << label << "=[" 
        << micro_sec_passed << "] microseconds");
    }
    return micro_sec_passed;
  }

  std::string &label() { return label_; }

  const std::string &label() const { return label_; }

private:
  int ended_;
  std::string label_;
  Time start_time_;
};
}
}
#endif //SRC_TIMER_H
