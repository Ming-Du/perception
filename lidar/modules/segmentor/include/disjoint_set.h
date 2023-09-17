#pragma once

#include <vector>
#include <map>
#include "common/include/msg/lidar_frame_msg.h"

namespace robosense {
class Universe {
  struct Element {
    int rank = 0;
    int p = 0;
    int size = 1;
  };
  
  public:
  Universe() : elts_(), sets_num_(0) {}
  ~Universe() {
    elts_.clear();
  }
  explicit Universe(const int elements_num);
  void Reset(const int elements_num);
  int Find(const int x);
  void Join(const int x, const int y);
  void JoinWithoutRank(const int x, const int y);
  int GetSize(const int x) const {
    return elts_[x].size;
  }
  int GetSetsNum() const {
    return sets_num_;
  }
  
  private:
  std::vector<Element> elts_;
  int sets_num_;
};

}
