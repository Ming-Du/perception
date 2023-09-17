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

#ifndef RS_SDK_BASE_CUDA_CHECK_H
#define RS_SDK_BASE_CUDA_CHECK_H

#include <stdio.h>
#include <cuda_runtime_api.h>
#include <iostream>
#include <ros/ros.h>
namespace robosense {

#if __CUDACC_VER_MAJOR__ >= 9
#undef __CUDACC_VER__
#define __CUDACC_VER__ \
  ((__CUDACC_VER_MAJOR__ * 10000) + (__CUDACC_VER_MINOR__ * 100))
#endif

#define BASE_CUDA_CHECK(condition) { GPUAssert((condition), __FILE__, __LINE__); }

inline void GPUAssert(cudaError_t code, const char *file, int line, bool abort = true) {
    if (code != cudaSuccess) {
        fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
        if (abort) {
            exit(code);
        }
    }
}

#define TV_CHECK_CUDA_ERR_V3(__macro_err)                                                                                 \
   {                                                                                                                      \                                                                                                 
        if (__macro_err != cudaSuccess) {                                                                                 \             
            std::stringstream __macro_s;                                                                                  \             
            __macro_s << __FILE__ << ":" << __LINE__ << "\n";                                                             \             
            __macro_s << "cuda execution failed with error " << __macro_err;                                              \             
            __macro_s << " " << cudaGetErrorString(__macro_err) << "\n";                                                  \             
            ROS_WARN_STREAM("\033[31m[" << __FUNCTION__ << __LINE__ << " ]:\033[0m" << __macro_s.str() );                 \
            throw std::runtime_error(__macro_s.str());                                                                    \            
        }                                                                                                                 \             
    }

    
}  // namespace robosense

#endif  // RS_SDK_BASE_CUDA_CHECK_H
