#pragma once

#include <limits>
#include <string>
#include <cmath>

namespace perception {
namespace mid_fusion {

#define TEST_TIME

#define H_DEBUG_R "\033[31m[Debug] " << __FUNCTION__ << __LINE__ << "\033[0m"
#define H_DEBUG_G "\033[32m[Debug] " << __FUNCTION__ << __LINE__ << "\033[0m"
#define H_DEBUG_B "\033[34m[Debug] " << __FUNCTION__ << __LINE__ << "\033[0m"
#define H_DEBUG_Y "\033[33m[Debug] " << __FUNCTION__ << __LINE__ << "\033[0m"

#define HDEBUG_R "\033[31m[Debug] \033[0m"
#define HDEBUG_G "\033[32m[Debug] \033[0m"
#define HDEBUG_B "\033[34m[Debug] \033[0m"
#define HDEBUG_Y "\033[33m[Debug] \033[0m"

#define PRECISION_NUMERIC 0.00001
#define MATCH_LIMIT_THR 5

#define MODEL_INFER_HISTORY_BUFF_SIZE 3

#define PED_SPEED_THRESHOLD 12 // 5 min/km
#define BIC_SPEED_THRESHOLD 20 // 3 min/km

#define EXTRA_VEG 0

}
}  // namespace perception
