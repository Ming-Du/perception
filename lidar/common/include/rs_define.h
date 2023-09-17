/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#ifndef RS_COMMON_RS_DEFINE_H_
#define RS_COMMON_RS_DEFINE_H_

#include <limits>
#include <string>

#if 1
#define HCOUT std::cout
#else
#define HCOUT //std::cout
#endif
#define H_DEBUG_R "\033[31m[Debug] " << __FUNCTION__ << __LINE__ << "\033[0m"
#define H_DEBUG_G "\033[32m[Debug] " << __FUNCTION__ << __LINE__ << "\033[0m"
#define H_DEBUG_B "\033[34m[Debug] " << __FUNCTION__ << __LINE__ << "\033[0m"
#define H_DEBUG_Y "\033[33m[Debug] " << __FUNCTION__ << __LINE__ << "\033[0m"

#define HDEBUG_R "\033[31m[Debug] \033[0m"
#define HDEBUG_G "\033[32m[Debug] \033[0m"
#define HDEBUG_B "\033[34m[Debug] \033[0m"
#define HDEBUG_Y "\033[33m[Debug] \033[0m"

#define HDENOISE_G "\033[32m[Denoise] \033[0m"

#define H_R "\033[31m"
#define H_G "\033[32m"
#define H_B "\033[34m"
#define H_Y "\033[33m"
#define H_P "\033[35m"
#define H_C "\033[36m"
#define H_0 "\033[0m"


#define RS_VERSION_MAJOR 1
#define RS_VERSION_MINOR 0
#define RS_VERSION_PATCH 0
namespace robosense {

#define RSAUX_STR_EXP(__A) #__A
#define RSAUX_STR(__A) RSAUX_STR_EXP(__A)
#define RS_VERSION              \
    RSAUX_STR(RS_VERSION_MAJOR) \
    "." RSAUX_STR(RS_VERSION_MINOR) "." RSAUX_STR(RS_VERSION_PATCH)

const double RS_EPS = std::numeric_limits<double>::epsilon();
const double RS_MAX = std::numeric_limits<double>::max();
const double RS_MIN = std::numeric_limits<double>::min();
#define MIN_VALUE 1e-15
#define RS_DBL_EPSILON 1e-8

#define RS_M_PI 3.14159265358979323846   /* pi */
#define RS_M_PI_2 1.57079632679489661923 /* pi/2 */
#define RS_M_PI_4 0.78539816339744830962 /* pi/4 */
#define RS_M_1_PI 0.31830988618379067154 /* 1/pi */
#define RS_M_2_PI 0.63661977236758134308 /* 2/pi */

const char g_frame_id_[] = "_frame_id_";
const char g_lidar_type_[] = "_lidar_type_";
const char g_base_dir_[] = "_base_dir_";

// 1:open try-catch . 0:close try-catch 
#define ENABLE_EXCEPTION_PROTECTION 1

#if ENABLE_EXCEPTION_PROTECTION
#define TRY_CATCH try {
#define END_TRY_CATCH \
    } catch (const std::exception &e) {                                                                 \
        ROS_ERROR_STREAM("catched exception: (" << __FILE__ << ":" << __LINE__ << ") " << e.what());    \
    } catch (const std::runtime_error &e) {                                                             \
        ROS_ERROR_STREAM("catched runtime_error: (" << __FILE__ << ":" << __LINE__ << ") " << e.what());\
    } catch (...) {                                                                                     \
        ROS_ERROR_STREAM("catched unknow exception. (" << __FILE__ << ":" << __LINE__ << ") ");         \
    }

#else
#define TRY_CATCH
#define END_TRY_CATCH
#endif
} // namespace robosense

#endif // RS_COMMON_RS_DEFINE_H_
