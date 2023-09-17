#pragma once

#include <iostream>
#include <map>
#include <vector>

namespace perception {
namespace base {

class Polynomial {
 public:
  Polynomial();
  ~Polynomial();
  double& operator[](const uint32_t& order);
  // sum(coeff_[i] * x^i)
  double operator()(const double& x);

  const std::map<uint32_t, double>& getCoeff() const;

 private:
  std::map<uint32_t, double> coeff_;
  std::vector<uint32_t> index_gap_;
  std::map<uint32_t, double> power_cache_;
  bool initialized_ = false;
};

std::ostream& operator<<(std::ostream& o, const Polynomial& p);

}  // namespace base
}  // namespace perception
