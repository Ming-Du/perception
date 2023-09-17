#pragma once

#include <limits>
#include <string>

#define H_DEBUG_R "\033[31m[Debug] " << __FUNCTION__ << __LINE__ << "\033[0m"
#define H_DEBUG_G "\033[32m[Debug] " << __FUNCTION__ << __LINE__ << "\033[0m"
#define H_DEBUG_B "\033[34m[Debug] " << __FUNCTION__ << __LINE__ << "\033[0m"
#define H_DEBUG_Y "\033[33m[Debug] " << __FUNCTION__ << __LINE__ << "\033[0m"

#define HDEBUG_R "\033[31m[Debug] \033[0m"
#define HDEBUG_G "\033[32m[Debug] \033[0m"
#define HDEBUG_B "\033[34m[Debug] \033[0m"
#define HDEBUG_Y "\033[33m[Debug] \033[0m"