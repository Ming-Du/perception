
#pragma once

#include <cmath>
#include <limits>

namespace perception {
namespace fusion {

// Integral type equal
template <typename T>
typename std::enable_if<std::is_integral<T>::value, bool>::type Equal(const T& lhs, const T& rhs) {
  return lhs == rhs;
}

// Floating point type equal
template <typename T>
typename std::enable_if<std::is_floating_point<T>::value, bool>::type Equal(const T& lhs,
                                                                            const T& rhs) {
  return std::fabs(lhs - rhs) < std::numeric_limits<T>::epsilon();
}

}  // namespace fusion
}  // namespace perception
