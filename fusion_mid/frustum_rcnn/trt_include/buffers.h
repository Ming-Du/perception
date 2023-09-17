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
#ifndef TENSORRT_BUFFERS_H
#define TENSORRT_BUFFERS_H

#include <cuda_runtime_api.h>
#include <cassert>
#include <iostream>
#include <iterator>
#include <memory>
#include <new>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>
#include "NvInfer.h"
#include "half.h"
#include "trt_common.h"

namespace TrtCommon {

template <typename AllocFunc, typename FreeFunc>
class GenericBuffer {
   public:
    //!
    //! \brief Construct an empty buffer.
    //!
    GenericBuffer(nvinfer1::DataType type = nvinfer1::DataType::kFLOAT)
        : mSize(0), mCapacity(0), mType(type), mBuffer(nullptr) {}

    //!
    //! \brief Construct a buffer with the specified allocation size in bytes.
    //!
    GenericBuffer(size_t size, nvinfer1::DataType type)
        : mSize(size), mCapacity(size), mType(type) {
        if (!allocFn(&mBuffer, this->nbBytes())) {
            throw std::bad_alloc();
        }
    }
    GenericBuffer(void* buf, size_t size, nvinfer1::DataType type)
        : mSize(size), mCapacity(size), mType(type), mBuffer(buf) {}

    GenericBuffer(GenericBuffer&& buf)
        : mSize(buf.mSize), mCapacity(buf.mCapacity), mType(buf.mType), mBuffer(buf.mBuffer) {
        buf.mSize = 0;
        buf.mCapacity = 0;
        buf.mType = nvinfer1::DataType::kFLOAT;
        buf.mBuffer = nullptr;
    }

    GenericBuffer& operator=(GenericBuffer&& buf) {
        if (this != &buf) {
            freeFn(mBuffer);
            mSize = buf.mSize;
            mCapacity = buf.mCapacity;
            mType = buf.mType;
            mBuffer = buf.mBuffer;
            // Reset buf.
            buf.mSize = 0;
            buf.mCapacity = 0;
            buf.mBuffer = nullptr;
        }
        return *this;
    }

    //!
    //! \brief Returns pointer to underlying array.
    //!
    void* data() { return mBuffer; }

    //!
    //! \brief Returns pointer to underlying array.
    //!
    const void* data() const { return mBuffer; }

    //!
    //! \brief Returns the size (in number of elements) of the buffer.
    //!
    size_t size() const { return mSize; }

    //!
    //! \brief Returns the size (in bytes) of the buffer.
    //!
    size_t nbBytes() const { return this->size() * TrtCommon::getElementSize(mType); }

    //!
    //! \brief Resizes the buffer. This is a no-op if the new size is smaller than or equal to the
    //! current capacity.
    //!
    void resize(size_t newSize) {
        mSize = newSize;
        if (mCapacity < newSize) {
            freeFn(mBuffer);
            if (!allocFn(&mBuffer, this->nbBytes())) {
                throw std::bad_alloc{};
            }
            mCapacity = newSize;
        }
    }

    //!
    //! \brief Overload of resize that accepts Dims
    //!
    void resize(const nvinfer1::Dims& dims) { return this->resize(TrtCommon::volume(dims)); }

    ~GenericBuffer() { freeFn(mBuffer); }

   private:
    size_t mSize{0}, mCapacity{0};
    nvinfer1::DataType mType;
    void* mBuffer;
    AllocFunc allocFn;
    FreeFunc freeFn;
};

class DeviceAllocator {
   public:
    bool operator()(void** ptr, size_t size) const { return cudaMalloc(ptr, size) == cudaSuccess; }
};

class DeviceFree {
   public:
    void operator()(void* ptr) const { cudaFree(ptr); }
};

class HostAllocator {
   public:
    bool operator()(void** ptr, size_t size) const {
        *ptr = malloc(size);
        return *ptr != nullptr;
    }
};
class NoAllocator {
   public:
    bool operator()(void** ptr, size_t size) const {
        // *ptr = malloc(size);
        return *ptr != nullptr;
    }
};

class HostFree {
   public:
    void operator()(void* ptr) const { free(ptr); }
};
class NoFree {
   public:
    void operator()(void* ptr) const { delete (ptr); }
};

using DeviceBuffer = GenericBuffer<DeviceAllocator, DeviceFree>;
using HostBuffer = GenericBuffer<HostAllocator, HostFree>;
using StacBuffer = GenericBuffer<NoAllocator, NoFree>;

class ManagedBuffer {
   public:
    DeviceBuffer deviceBuffer;
    HostBuffer hostBuffer;
    StacBuffer stackBuffer;
};
typedef struct InputsBuffer {
    void* mdata;
    size_t msize;
    std::vector<int> mShape;
    InputsBuffer(void* data, size_t size) : mdata(data), msize(size) {}
} InputsBuffer;

class BufferManager {
   public:
    static const size_t kINVALID_SIZE_VALUE = ~size_t(0);

    BufferManager() { mMaxtBatchSize = 0; }
    //初始化函数，batchSize 表示实际的数据batch大小，会自动分配好固定内存和显存，推荐使用该函数
    //此种初始化方式，可以服用buffer对象
    void InitBuffer(nvinfer1::ICudaEngine* engine, int batchSize);
    //初始化函数，batchSize 表示实际的数据batch大小，会自动按需分配好内存和显存，属于自适应动态batch
    void InitBufferDynamic(nvinfer1::ICudaEngine* engine, int64_t batchSize);
    //不推荐使用
    void InitDeviceBuffer(nvinfer1::ICudaEngine* engine, const std::vector<InputsBuffer>& inputs);

    //获取gpu内存地址
    std::vector<void*>& getDeviceBindings() { return mDeviceBindings; }

    const std::vector<void*>& getDeviceBindings() const { return mDeviceBindings; }

    //获取指定name的模型输入输出的显存地址
    void* getDeviceBuffer(const std::string& tensorName) const;

    //获取指定name的模型输入输出的内存地址
    void* getHostBuffer(const std::string& tensorName) const;

    void copyInputToDevice();

    void copyOutputToHost();

    void copyInputToDeviceAsync(const cudaStream_t& stream = 0);

    void copyOutputToHostAsync(const cudaStream_t& stream = 0);

    ~BufferManager() = default;

    //获取本次推理的实际batch大小，等同InitBuffer中batchsize
    int64_t BatchSize() { return mBatchSize; }
    //获取输入为name的数据实际shape
    std::unordered_map<std::string, std::vector<int>>& GetInputDims() { return mInputDims; }
    //获取输出为name的数据实际shape
    std::unordered_map<std::string, std::vector<int>>& GetOutputDims() { return mOutputDims; }
    int64_t GetIONbytesByName(std::string& name);

   private:
    void* getBuffer(const bool isHost, const std::string& tensorName) const;

    void memcpyBuffers(const bool copyInput,
                       const bool deviceToHost,
                       const bool async,
                       const cudaStream_t& stream = 0);

    nvinfer1::ICudaEngine* mEngine;  //!< The pointer to the engine
    int mBatchSize;
    int mMaxtBatchSize;  //!< The batch size for legacy networks, 0 otherwise.
    std::vector<std::unique_ptr<ManagedBuffer>>
        mManagedBuffers;  //!< The vector of pointers to managed buffers
    std::vector<void*>
        mDeviceBindings;  //!< The vector of device buffers needed for engine execution
    std::unordered_map<std::string, std::vector<int>> mInputDims;
    std::unordered_map<std::string, std::vector<int>> mOutputDims;
    std::unordered_map<std::string, nvinfer1::DataType> mIODtypes;
};
struct SharedBuffer {
    void* mHostPtr{nullptr};
    void* mDevicePtr{nullptr};
    size_t mSize{0};
    nvinfer1::DataType mType;
    bool mStatus{false};
    SharedBuffer(size_t vol, nvinfer1::DataType type);
    size_t GetNBytes();
    ~SharedBuffer();
};
// SharedBufferManager 适合xviar社保，属于zero-copy 用法
class SharedBufferManager {
   private:
    nvinfer1::ICudaEngine* mEngine;
    int mBatchSize;
    int mMaxBatchSize;
    std::vector<std::unique_ptr<SharedBuffer>> mManagedBuffers;
    std::vector<void*> mDeviceBindings;
    std::unordered_map<std::string, std::vector<int>> mInputDims;
    std::unordered_map<std::string, std::vector<int>> mOutputDims;
    std::unordered_map<std::string, nvinfer1::DataType> mIODtypes;

   public:
    SharedBufferManager() { mMaxBatchSize = 0; }
    //初始化函数，batchSize 表示实际的数据batch大小，会自动按需分配好内存和显存，属于自适应动态batch
    bool InitBufferDynamic(nvinfer1::ICudaEngine* engine, int batch = 1);
    //初始化函数，batchSize 表示实际的数据batch大小，会自动分配好固定内存和显存，推荐使用该函数
    //此种初始化方式，可以服用buffer对象
    void InitBuffer(nvinfer1::ICudaEngine* engine, int batchSize);
    //获取显存社保地址
    std::vector<void*>& getDeviceBindings() { return mDeviceBindings; }
    const std::vector<void*>& getDeviceBindings() const { return mDeviceBindings; }
    //获取指定输入输出为name的显存地址
    void* getDeviceBuffer(const std::string& tensorName) const;

    //获取指定输入输出为name的内存地址
    void* getHostBuffer(const std::string& tensorName) const;

    // 获取实现推理的batchsize，同InitBuffer 中batchsize
    int64_t BatchSize() { return mBatchSize; }
    //获取输入为name的数据实际shape
    std::unordered_map<std::string, std::vector<int>>& GetInputDims() { return mInputDims; }
    //获取输出为name的数据实际shape
    std::unordered_map<std::string, std::vector<int>>& GetOutputDims() { return mOutputDims; }
    //获取指定name的输入输出大小
    int64_t GetIONbytesByName(std::string& name);

   private:
    void* getBuffer(const bool isHost, const std::string& tensorName) const;
};

}  // namespace TrtCommon

#endif  // TENSORRT_BUFFERS_H
