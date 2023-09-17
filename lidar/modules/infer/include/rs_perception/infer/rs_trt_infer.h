/******************************************************************************
 * Copyright 2021 RoboSense All rights reserved.
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

#ifndef RS_TRT_INFER_H_
#define RS_TRT_INFER_H_

#include <iostream>
#include <sstream>
#include <cudnn.h>
#include "NvInfer.h"
#include "NvOnnxConfig.h"
#include "NvOnnxParser.h"

#if defined(RSPERCEPTION_TENSORRT_V7)
#include "NvInferPlugin.h"
#else
#include "NvOnnxParserRuntime.h"
#endif
#include "common/include/common.h"
#include "rs_perception/infer/infer.h"
#include <ros/ros.h>

namespace robosense {

#define BASE_CUDA_CHECK(condition) { GPUAssert((condition), __FILE__, __LINE__); }

inline void GPUAssert(cudaError_t code, const char *file, int line, bool abort = true) {
    if (code != cudaSuccess) {
        fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
        // if (abort) {
        //     exit(code);
        // }
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

// Logger for TensorRT
class Logger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char *msg) override {
        // suppress info-level message
        if (severity == Severity::kERROR || severity == Severity::kINTERNAL_ERROR) {
            std::cerr << "rs_trt_infer: " << msg << std::endl;
            std::cerr << "rs_trt_infer:" << " --- detected system info ---" << std::endl;
            std::cerr << "rs_trt_infer:" << "- cuda version: " << CUDART_VERSION << std::endl;
            std::cerr << "rs_trt_infer:" << "- cudnn version: " << CUDNN_VERSION << std::endl;
            std::cerr << "rs_trt_infer:" << "- tensorRT version: " << NV_TENSORRT_VERSION << std::endl;
            exit(-1);
        }
    }
};

class  RsTrtInferImpl {
public:
    using Ptr = std::shared_ptr<RsTrtInferImpl>;

    RsTrtInferImpl() {}

    void init(const RsInferConfig& init_configs);

    const std::vector<void*> getInputs() const {
        return inputs_;
    }
    const std::vector<void*> getOutputs() const {
        return outputs_;
    }
    const std::vector<size_t > getInputSizes() const {
        return input_sizes_;
    }
    const std::vector<size_t > getOutputSizes() const {
        return output_sizes_;
    }

    ~RsTrtInferImpl();

    void infer();
    void inferWithCpus(const std::vector<float*>& inputs, const std::vector<float*>& outputs);
    void inferWithCpuInputs(const std::vector<float*>& inputs);
    void inferWithCpuOutputs(const std::vector<float*>& outputs);

private:
    void buildEngine() {
        loadTrtEngine();
        buildWithEngine();
    }
    std::string name() {
        return "RsTrtInfer";
    }
    void loadTrtEngine();
    std::string getEngineInfo();
    bool loadFile(const std::string& in_file, std::vector<char>& out_infos);
    void buildWithEngine();
    void *safeCudaMalloc(size_t memSize);

    RsInferConfig init_configs_;
    nvinfer1::ICudaEngine *engine_ = nullptr;
    nvinfer1::IExecutionContext *context_;
#if defined(RSPERCEPTION_TENSORRT_V5) || defined(RSPERCEPTION_TENSORRT_V6)
    nvinfer1::IPluginFactory *plugin_;
#endif
    Logger gLogger_;
    cudaStream_t stream_;
    // parameters
    std::vector<void *> buffers_, inputs_, outputs_;
    std::vector<int> binding_idx_inputs_, binding_idx_outputs_;
    std::vector<size_t> buffer_sizes_, input_sizes_, output_sizes_;
};

}  // namespace robosense

#endif  // RS_TRT_INFER_H_
