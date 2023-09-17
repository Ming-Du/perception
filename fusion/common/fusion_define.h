#pragma once

#include <limits>
#include <string>
#include <cmath>

#define H_DEBUG_R "\033[31m[Debug] " << __FUNCTION__ << __LINE__ << "\033[0m"
#define H_DEBUG_G "\033[32m[Debug] " << __FUNCTION__ << __LINE__ << "\033[0m"
#define H_DEBUG_B "\033[34m[Debug] " << __FUNCTION__ << __LINE__ << "\033[0m"
#define H_DEBUG_Y "\033[33m[Debug] " << __FUNCTION__ << __LINE__ << "\033[0m"

#define HDEBUG_R "\033[31m[Debug] \033[0m"
#define HDEBUG_G "\033[32m[Debug] \033[0m"
#define HDEBUG_B "\033[34m[Debug] \033[0m"
#define HDEBUG_Y "\033[33m[Debug] \033[0m"

#define EPSILON 0.000001
#define MIN_TRACKING_TIME 0.0
#define MIN_VALUE 1e-6
#define IS_DOUBLE_ZERO(d) (fabs(d) < MIN_VALUE)

#define DEBUG_OBU 0
#define DEBUG_ROS 0
#define DURATION_DEFAULT 65535

constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / M_PI;
