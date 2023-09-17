#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

namespace perception {
namespace mid_fusion {

template <typename T>
struct Vec2 {
  T x, y;
  Vec2() : x(T(0)), y(T(0)){};
  Vec2(T _x, T _y) : x(_x), y(_y){};

  Vec2 operator+(const Vec2 &other) const { return Vec2(this->x + other.x, this->y + other.y); }
  Vec2 operator-(const Vec2 &other) const { return Vec2(this->x - other.x, this->y - other.y); }
  Vec2 &operator+=(const Vec2 &other) {
    this->x += other.x;
    this->y += other.y;
    return *this;
  }
  Vec2 &operator-=(const Vec2 &other) {
    this->x -= other.x;
    this->y -= other.y;
    return *this;
  }
  Vec2 operator*(T factor) const { return Vec2(this->x * factor, this->y * factor); }
  void normalize() {
    double norm = std::sqrt(x * x + y * y);
    x /= norm;
    y /= norm;
  }
  double norm() {
    double norm = std::sqrt(x * x + y * y);
    return norm;
  }
};

typedef Vec2<double> Vec2d;
typedef Vec2d Point2d;
typedef Vec2<float> Vec2f;
typedef Vec2f Point2f;
typedef Vec2<int> Vec2i;

}  // namespace mid_fusion
}  // namespace perception
