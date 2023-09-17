/*
 * SPDX-FileCopyrightText: Copyright (c) 1993-2022 NVIDIA CORPORATION & AFFILIATES. All rights
 * reserved. SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once
#include "NvOnnxConfig.h"
#include "NvOnnxParser.h"
#include "buffers.h"
#include "logger.h"
#include "trt_common.h"

#include <cuda_runtime_api.h>
#include "NvInfer.h"
#include "NvInferPlugin.h"
#include "NvInferRuntime.h"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include "utils.h"

using namespace Trt;
struct ModelParams {
    std::vector<std::string> modelDirs;
    std::string modelName;
};

typedef std::unordered_map<std::string, std::vector<int>> Shapebing;

template <typename T>
struct TrtDestroyer {
    void operator()(T* t) { t->destroy(); }
};

template <typename T>
using TrtUniquePtr = std::unique_ptr<T, TrtDestroyer<T>>;

class TrtModel {
   public:
    TrtModel(const ModelParams& params) : mParams(params), mEngine(nullptr) {
        Trt::gLogger.setReportableSeverity(Logger::Severity::kVERBOSE);
    }
    bool BuildTrt();
    bool infer(TrtCommon::BufferManager& buffers);
    // zero-copy infer
    bool infer_v2(TrtCommon::SharedBufferManager& sm);

    TrtUniquePtr<nvinfer1::ICudaEngine>& GetCudaEngine() { return mEngine; }
    bool WarnUp();
    //获取指定name输入输出tensor的shape (batchsize=1)
    std::vector<int> GetEngineIOputDimsOneBatch(std::string name) {
        std::vector<int> ret;
        if ((mEngineInputDims.find(name)) == mEngineInputDims.end()) {
            if ((mEngineOutputDims.find(name)) == mEngineOutputDims.end()) {
            } else {
                ret = mEngineOutputDims.find(name)->second;
            }
        } else {
            ret = mEngineInputDims.find(name)->second;
        }
        return ret;
    }
    //获取指定name的输入输出tensor在batchsize=1时，tensor所占空间大小(bytes)
    int GetIOputSizeInBytesOneBatch(std::string name) {
        int size = 0;
        if (mEngineDimSize.find(name) != mEngineDimSize.end()) {
            size = mEngineDimSize.find(name)->second;
        }
        return size;
    }
    const std::vector<std::string>& GetOutputNames();
    const std::vector<std::string>& GetInputNames();
    const nvinfer1::DataType GetEngineDataType(std::string name) { return mEngineDataType[name]; }
    void CudaSetDeviceFlags() { cudaSetDeviceFlags(cudaDeviceMapHost); }

    int mMaxBatch{1};

   private:
    ModelParams mParams;  //!< The parameters
    std::unordered_map<std::string, std::vector<int>> mEngineInputDims;
    std::unordered_map<std::string, std::vector<int>> mEngineOutputDims;
    std::unordered_map<std::string, nvinfer1::DataType> mEngineDataType;
    std::vector<std::string> mInputNames;
    std::vector<std::string> mOutputNames;
    std::unordered_map<std::string, int> mEngineDimSize;
    TrtUniquePtr<nvinfer1::ICudaEngine> mEngine{
        nullptr};  //!< The TensorRT engine used to run the network
    TrtUniquePtr<nvinfer1::IExecutionContext> mContex{nullptr};
    bool setProfile(TrtUniquePtr<nvinfer1::ICudaEngine>& engine,
                    TrtUniquePtr<nvinfer1::IExecutionContext>& context,
                    Shapebing& io_shape);
    void Init();
};
