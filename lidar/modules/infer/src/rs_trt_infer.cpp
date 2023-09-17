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
#include <iostream>
#include <sstream>
#include <unistd.h>
#include "rs_perception/infer/infer.h"
#include "rs_perception/infer/rs_trt_infer.h"
#include <chrono>

namespace robosense {

template<typename DataType>
inline unsigned int elementSize(DataType t) {
    switch (t) {
        case nvinfer1::DataType::kINT32:
            // Fallthrough, same as kFLOAT
        case nvinfer1::DataType::kFLOAT:
            return 4;
        case nvinfer1::DataType::kHALF:
            return 2;
        case nvinfer1::DataType::kINT8:
            return 1;
        default:
            break;
            std::cerr << "elementSize: invalid data type" << std::endl;
            abort();
    }
}

void RsTrtInferImpl::init(const RsInferConfig& init_configs) {
    init_configs_ = init_configs;
    buildEngine();
    std::stringstream ss;
    ss << name() << ": get params as follow" << std::endl;
    ss << getEngineInfo() ;
}

std::string RsTrtInferImpl::getEngineInfo() {
    std::ostringstream os;
    os << name() << ": max_workspace "<< init_configs_.max_workspace << std::endl;
    os << name() << ": enable_fp16 "<< init_configs_.enable_fp16 << std::endl;
    os << name() << ": height "<< init_configs_.height << std::endl;
    os << name() << ": width "<< init_configs_.width << std::endl;
    os << name() << ": device_id " << init_configs_.device_id << std::endl;
    return os.str();
}

RsTrtInferImpl::~RsTrtInferImpl() {
    cudaError_t err = cudaFree(0);
    if (err != cudaErrorCudartUnloading) {
        for (auto &buffer : buffers_) {
            if (buffer == nullptr) {
                continue;
            }
            TV_CHECK_CUDA_ERR_V3(cudaFree(buffer));
        }
        context_->destroy();
        engine_->destroy();
    }
    cudaStreamDestroy(stream_);
}

void RsTrtInferImpl::infer() {
    try {
        // execute inference
        context_->enqueue(1, &buffers_[0], stream_, nullptr);
    }
    catch (const char *msg) {
        remove(init_configs_.gie_path.c_str());
        std::cerr << name() << ": infer failed, please try again!" << std::endl;
        // exit(-1);
        std::ostringstream oss;
        oss << name() << ": infer failed, please try again!";
        throw std::runtime_error(oss.str());
    }
    cudaStreamSynchronize(stream_);
}

void RsTrtInferImpl::inferWithCpus(const std::vector<float*>& inputs, const std::vector<float*>& outputs) {
    if (inputs.size() != input_sizes_.size()) {
        std::cerr << name() << ": invalid inputs!" << std::endl;
        exit(-1);
    }
    if (outputs.size() != output_sizes_.size()) {
        std::cerr << name() << ": invalid outputs!" << std::endl;
        exit(-1);
    }
    // copy input features to GPU device
    for (size_t i = 0; i < inputs.size(); i++) {
        TV_CHECK_CUDA_ERR_V3(cudaMemcpyAsync(buffers_[binding_idx_inputs_[i]], inputs[i], input_sizes_[i],
                                        cudaMemcpyHostToDevice, stream_));
    }
    infer();
    for (size_t i = 0; i < outputs.size(); ++i) {
        TV_CHECK_CUDA_ERR_V3(cudaMemcpyAsync(outputs[i], buffers_[binding_idx_outputs_[i]], output_sizes_[i],
                                        cudaMemcpyDeviceToHost, stream_));
    }
}

void RsTrtInferImpl::inferWithCpuInputs(const std::vector<float*>& inputs) {
    if (inputs.size() != input_sizes_.size()) {
        std::cerr << name() << ": invalid inputs!" << std::endl;
        exit(-1);
    }
    // copy input features to GPU device
    for (size_t i = 0; i < inputs.size(); i++) {
        TV_CHECK_CUDA_ERR_V3(cudaMemcpyAsync(buffers_[binding_idx_inputs_[i]], inputs[i], input_sizes_[i],
                                        cudaMemcpyHostToDevice, stream_));
    }
    infer();
}

void RsTrtInferImpl::inferWithCpuOutputs(const std::vector<float*>& outputs) {
    if (outputs.size() != output_sizes_.size()) {
        std::cerr << name() << ": invalid outputs!" << std::endl;
        exit(-1);
    }
    infer();
    for (size_t i = 0; i < outputs.size(); ++i) {
        TV_CHECK_CUDA_ERR_V3(cudaMemcpyAsync(outputs[i], buffers_[binding_idx_outputs_[i]], output_sizes_[i],
                                        cudaMemcpyDeviceToHost, stream_));
    }
}

void RsTrtInferImpl::loadTrtEngine() {

    engine_ = nullptr;
    initLibNvInferPlugins(&gLogger_, "");

    if (access(init_configs_.trt_path.c_str(), F_OK) == 0) {
        
        std::vector<char> trtModelStream;

        if (loadFile(init_configs_.trt_path, trtModelStream)) {
            try {

                nvinfer1::IRuntime* runtime = nvinfer1::createInferRuntime(gLogger_);
                if (runtime == nullptr) {
                    throw "runtime_error";
                }

                auto start_time = std::chrono::high_resolution_clock::now();

          #if defined(RSPERCEPTION_TENSORRT_V5) || defined(RSPERCEPTION_TENSORRT_V6)
                plugin_ = nvonnxparser::createPluginFactory(gLogger_);
                engine_ = runtime->deserializeCudaEngine(trtModelStream.data(), trtModelStream.size(), plugin_);
          #endif
          #if defined(RSPERCEPTION_TENSORRT_V7)
                engine_ = runtime->deserializeCudaEngine(trtModelStream.data(), trtModelStream.size(), nullptr);
          #endif
                auto end_time = std::chrono::high_resolution_clock::now();
                auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                std::cout << "Engine loading time: " << elapsed_time.count() << " ms." << std::endl;


                runtime->destroy();
            }
            catch (...) {

                engine_ = nullptr;
            }
        }
    }

    if (engine_ == nullptr) {
        throw "model_loading_error";
    }
}

bool RsTrtInferImpl::loadFile(const std::string& in_file, std::vector<char>& out_infos) {
    std::ifstream cache_infos(in_file.c_str(), std::ios::binary);
    if (cache_infos.good()) {
        // get size of file
        cache_infos.seekg(0, cache_infos.end);
        int size = static_cast<int>(cache_infos.tellg());
        out_infos.clear();
        out_infos.resize(size);
        cache_infos.seekg(0);
        // read content of infile
        cache_infos.read(out_infos.data(), size);
        cache_infos.close();
        return true;
    } else {
        return false;
    }
}

void RsTrtInferImpl::buildWithEngine() {
    if (engine_ == nullptr) {
        std::cerr << name() << ": failed to build inference engine" << std::endl;
        exit(-1);
    }
    // create context
    context_ = engine_->createExecutionContext();
    const auto& num_bindings = engine_->getNbBindings();
    binding_idx_inputs_.clear();
    binding_idx_outputs_.clear();
    buffer_sizes_.clear();

    binding_idx_inputs_.reserve(num_bindings);
    binding_idx_outputs_.reserve(num_bindings);
    buffer_sizes_.reserve(num_bindings);
    buffers_.resize(static_cast<unsigned int>(num_bindings));

    std::vector<int64_t> buffer_size(num_bindings);
    std::vector<nvinfer1::DataType> buffer_type(num_bindings);
    for (auto i = 0; i < num_bindings; ++i) {
        nvinfer1::Dims dims = engine_->getBindingDimensions(i);
        auto dtype = engine_->getBindingDataType(i);
        int64_t buffer_size = 1;
        for (int j = 0; j < dims.nbDims; ++j) {
            buffer_size *= dims.d[j];
        }
        const auto& tmp_buffer_size = static_cast<size_t>(buffer_size * elementSize(dtype));
        buffer_sizes_.emplace_back(tmp_buffer_size);

        if (engine_->bindingIsInput(i)) {
            binding_idx_inputs_.emplace_back(i);
            buffers_[i] = safeCudaMalloc(tmp_buffer_size);
        } else {
            binding_idx_outputs_.emplace_back(i);
            buffers_[i] = safeCudaMalloc(tmp_buffer_size);
        }
    }
    inputs_.resize(binding_idx_inputs_.size());
    input_sizes_.resize(binding_idx_inputs_.size());
    outputs_.resize(binding_idx_outputs_.size());
    output_sizes_.resize(binding_idx_outputs_.size());
    for (size_t i = 0; i < binding_idx_inputs_.size(); ++i) {
        const auto& bind_idx = binding_idx_inputs_[i];
        inputs_[i] = buffers_[bind_idx];
        input_sizes_[i] = buffer_sizes_[bind_idx];
    }
    for (size_t i = 0; i < binding_idx_outputs_.size(); ++i) {
        const auto& bind_idx = binding_idx_outputs_[i];
        outputs_[i] = buffers_[bind_idx];
        output_sizes_[i] = buffer_sizes_[bind_idx];
    }
    // define cuda stream
    TV_CHECK_CUDA_ERR_V3(cudaStreamCreate(&stream_));
}

void* RsTrtInferImpl::safeCudaMalloc(size_t memSize) {
    void *deviceMem;
    TV_CHECK_CUDA_ERR_V3(cudaMalloc(&deviceMem, memSize));
    if (deviceMem == nullptr) {
        std::cerr << name() << ": CUDA out of memory!" << std::endl;
        abort();
    }
    return deviceMem;
}

}  // namespace robosense
