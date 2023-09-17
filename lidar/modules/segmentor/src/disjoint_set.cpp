#include "disjoint_set.h"
#include <iostream>

namespace robosense {

Universe::Universe(const int elements_num) : elts_(elements_num),
                                             sets_num_(elements_num) {
  for (int i = 0; i < elements_num; ++i) {
    elts_[i].rank = 0;
    elts_[i].size = 1;
    elts_[i].p = i;
  }
}

void Universe::Reset(const int elements_num) {
  sets_num_ = elements_num;
  elts_.resize(elements_num);
  for (int i = 0; i < elements_num; ++i) {
    elts_[i].rank = 0;
    elts_[i].size = 1;
    elts_[i].p = i;
  }
}

int Universe::Find(const int x) {
  int y = x;
  
  while (y != elts_[y].p) {
    y = elts_[y].p;
  }
  int w = x;
  while (true) {
    const int z = elts_[w].p;
    if (z == w) {
      break;
    }
    elts_[w].p = y;
    w = z;
  }
  return y;
}

void Universe::Join(const int x, const int y) {
  if (elts_[x].rank > elts_[y].rank) {
    elts_[y].p = x;
    elts_[x].size += elts_[y].size;
  } else {
    elts_[x].p = y;
    elts_[y].size += elts_[x].size;
    if (elts_[x].rank == elts_[y].rank) {
      ++elts_[y].rank;
    }
  }
  --sets_num_;
}

void Universe::JoinWithoutRank(const int x, const int y) {
  elts_[y].p = x;
  elts_[x].size += elts_[y].size;
  --sets_num_;
}

}