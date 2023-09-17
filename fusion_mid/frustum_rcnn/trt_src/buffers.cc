#include "buffers.h"

namespace TrtCommon {

void BufferManager::InitBuffer(nvinfer1::ICudaEngine* engine, int batchSize) {
  if (mMaxtBatchSize > 0) {
    mBatchSize = batchSize;
    return;
  }
  mEngine = engine;
  mBatchSize = batchSize;
  int n = mEngine->getNbOptimizationProfiles();
  int max_idx = n - 1;
  for (int i = 0; i < mEngine->getNbBindings(); i++) {
    std::vector<int> real_dims;
    nvinfer1::DataType type = mEngine->getBindingDataType(i);
    int64_t vol = 1;
    if (engine->bindingIsInput(i)) {
      nvinfer1::Dims max_profile_dims =
          mEngine->getProfileDimensions(i, max_idx, nvinfer1::OptProfileSelector::kMAX);
      vol = TrtCommon::volume(max_profile_dims);

      mMaxtBatchSize = max_profile_dims.d[0];
      real_dims.push_back(mBatchSize);
      for (int k = 1; k < max_profile_dims.nbDims; k++) {
        real_dims.push_back(max_profile_dims.d[k]);
      }

    } else {
      auto dims = mEngine->getBindingDimensions(i);
      int vecDim = mEngine->getBindingVectorizedDim(i);
      if (-1 != vecDim)  // i.e., 0 != lgScalarsPerVector
      {
        int scalarsPerVec = mEngine->getBindingComponentsPerElement(i);
        dims.d[vecDim] = divUp(dims.d[vecDim], scalarsPerVec);
        vol *= scalarsPerVec;
      }
      vol *= TrtCommon::volume(dims);
      if (vol < 0) {
        vol *= (-1 * mMaxtBatchSize);
      }
      real_dims.push_back(mBatchSize);
      for (auto j = 1; j < dims.nbDims; j++) {
        if (dims.d[j] < 0) {
          real_dims.push_back(mBatchSize);
        } else {
          real_dims.push_back(dims.d[j]);
        }
      }
      mOutputDims[mEngine->getBindingName(i)] = real_dims;
    }
    mIODtypes[mEngine->getBindingName(i)] = type;
    std::unique_ptr<ManagedBuffer> manBuf{new ManagedBuffer()};
    manBuf->deviceBuffer = DeviceBuffer(vol, type);
    manBuf->hostBuffer = HostBuffer(vol, type);
    mDeviceBindings.emplace_back(manBuf->deviceBuffer.data());
    mManagedBuffers.emplace_back(std::move(manBuf));
  }
}
void BufferManager::InitBufferDynamic(nvinfer1::ICudaEngine* engine, int64_t batchSize) {
  mBatchSize = batchSize;
  mEngine = engine;
  for (int i = 0; i < mEngine->getNbBindings(); i++) {
    auto dims = mEngine->getBindingDimensions(i);
    int64_t vol = 1;
    nvinfer1::DataType type = mEngine->getBindingDataType(i);
    int vecDim = mEngine->getBindingVectorizedDim(i);
    if (-1 != vecDim)  // i.e., 0 != lgScalarsPerVector
    {
      int scalarsPerVec = mEngine->getBindingComponentsPerElement(i);
      dims.d[vecDim] = divUp(dims.d[vecDim], scalarsPerVec);
      vol *= scalarsPerVec;
    }
    vol *= TrtCommon::volume(dims);
    if (vol < 0) {
      vol *= (-1 * mBatchSize);
    }
    std::vector<int> real_dims;
    for (auto j = 0; j < dims.nbDims; j++) {
      if (dims.d[j] < 0) {
        real_dims.push_back(mBatchSize);
      } else {
        real_dims.push_back(dims.d[j]);
      }
    }
    if (engine->bindingIsInput(i)) {
      mInputDims[mEngine->getBindingName(i)] = real_dims;
    } else {
      mOutputDims[mEngine->getBindingName(i)] = real_dims;
    }
    mIODtypes[mEngine->getBindingName(i)] = type;
    std::unique_ptr<ManagedBuffer> manBuf{new ManagedBuffer()};
    manBuf->deviceBuffer = DeviceBuffer(vol, type);
    manBuf->hostBuffer = HostBuffer(vol, type);
    mDeviceBindings.emplace_back(manBuf->deviceBuffer.data());
    mManagedBuffers.emplace_back(std::move(manBuf));
  }
}
void BufferManager::InitDeviceBuffer(nvinfer1::ICudaEngine* engine,
                                     const std::vector<InputsBuffer>& inputs) {
  mEngine = engine;
  for (int i = 0; i < mEngine->getNbBindings(); i++) {
    // std::cout << "tensor name,idx=" << i << ",name=" <<  mEngine->getBindingName(i);
    auto dims = mEngine->getBindingDimensions(i);
    int64_t vol = 1;
    nvinfer1::DataType type = mEngine->getBindingDataType(i);
    int vecDim = mEngine->getBindingVectorizedDim(i);
    if (-1 != vecDim)  // i.e., 0 != lgScalarsPerVector
    {
      int scalarsPerVec = mEngine->getBindingComponentsPerElement(i);
      dims.d[vecDim] = divUp(dims.d[vecDim], scalarsPerVec);
      vol *= scalarsPerVec;
    }
    vol *= TrtCommon::volume(dims);

    if (vol < 0) {
      vol = (-1 * vol);
    }
    std::vector<int> real_dims;
    std::unique_ptr<ManagedBuffer> manBuf{new ManagedBuffer()};
    if (engine->bindingIsInput(i)) {
      mBatchSize = inputs[i].msize / (TrtCommon::getElementSize(type) * vol);
      manBuf->stackBuffer = StacBuffer(inputs[i].mdata, inputs[i].msize, type);
      manBuf->deviceBuffer = DeviceBuffer(inputs[i].msize, type);
      std::cout << "batch size:" << mBatchSize << std::endl;
      for (auto j = 0; j < dims.nbDims; j++) {
        if (dims.d[j] < 0) {
          real_dims.push_back(mBatchSize);
        } else {
          real_dims.push_back(dims.d[j]);
        }
      }
      mInputDims[mEngine->getBindingName(i)] = real_dims;
    } else {
      manBuf->hostBuffer = HostBuffer(vol * mBatchSize, type);
      manBuf->deviceBuffer = DeviceBuffer(vol * mBatchSize, type);
      for (auto j = 0; j < dims.nbDims; j++) {
        if (dims.d[j] < 0) {
          real_dims.push_back(mBatchSize);
        } else {
          real_dims.push_back(dims.d[j]);
        }
      }
      mOutputDims[mEngine->getBindingName(i)] = real_dims;
    }
    mIODtypes[mEngine->getBindingName(i)] = type;
    mDeviceBindings.emplace_back(manBuf->deviceBuffer.data());
    mManagedBuffers.emplace_back(std::move(manBuf));
  }
}

//获取指定name的模型输入输出的显存地址
void* BufferManager::getDeviceBuffer(const std::string& tensorName) const {
  return getBuffer(false, tensorName);
}
//获取指定name的模型输入输出的内存地址
void* BufferManager::getHostBuffer(const std::string& tensorName) const {
  return getBuffer(true, tensorName);
}

void BufferManager::copyInputToDevice() { memcpyBuffers(true, false, false); }

void BufferManager::copyOutputToHost() { memcpyBuffers(false, true, false); }

void BufferManager::copyInputToDeviceAsync(const cudaStream_t& stream) {
  memcpyBuffers(true, false, true, stream);
}
void BufferManager::copyOutputToHostAsync(const cudaStream_t& stream) {
  memcpyBuffers(false, true, true, stream);
}

void* BufferManager::getBuffer(const bool isHost, const std::string& tensorName) const {
  int index = mEngine->getBindingIndex(tensorName.c_str());
  if (index == -1) return nullptr;
  if (isHost) {
    if (mManagedBuffers[index]->hostBuffer.size() == 0) {
      return mManagedBuffers[index]->stackBuffer.data();
    } else {
      return mManagedBuffers[index]->hostBuffer.data();
    }
  } else {
    return mManagedBuffers[index]->deviceBuffer.data();
  }
}

void BufferManager::memcpyBuffers(const bool copyInput, const bool deviceToHost, const bool async,
                                  const cudaStream_t& stream) {
  for (int i = 0; i < mEngine->getNbBindings(); i++) {
    void* host = mManagedBuffers[i]->stackBuffer.size() == 0
                     ? mManagedBuffers[i]->hostBuffer.data()
                     : mManagedBuffers[i]->stackBuffer.data();
    void* dstPtr = deviceToHost ? host : mManagedBuffers[i]->deviceBuffer.data();
    const void* srcPtr = deviceToHost ? mManagedBuffers[i]->deviceBuffer.data() : host;
    size_t byteSize = mManagedBuffers[i]->hostBuffer.nbBytes();
    if (byteSize == 0) {
      byteSize = mManagedBuffers[i]->stackBuffer.nbBytes();
    }
    const cudaMemcpyKind memcpyType =
        deviceToHost ? cudaMemcpyDeviceToHost : cudaMemcpyHostToDevice;
    if ((copyInput && mEngine->bindingIsInput(i)) || (!copyInput && !mEngine->bindingIsInput(i))) {
      if (async)
        CHECK(cudaMemcpyAsync(dstPtr, srcPtr, byteSize, memcpyType, stream));
      else
        CHECK(cudaMemcpy(dstPtr, srcPtr, byteSize, memcpyType));
    }
  }
}
int64_t BufferManager::GetIONbytesByName(std::string& name) {
  std::vector<int> shape;
  int64_t vol = 0;
  if (mInputDims.find(name) != mInputDims.end()) {
    shape = mInputDims[name];
  } else if (mOutputDims.find(name) != mOutputDims.end()) {
    shape = mOutputDims[name];
  }
  if (shape.size() > 0) {
    nvinfer1::DataType dtype = mIODtypes[name];
    vol = std::accumulate(shape.begin(), shape.end(), 1, std::multiplies<int64_t>());
    vol *= TrtCommon::getElementSize(dtype);
  }
  return vol;
}

SharedBuffer::SharedBuffer(size_t vol, nvinfer1::DataType type) : mSize(vol), mType(type) {
  size_t size = vol * TrtCommon::getElementSize(type);
  cudaHostAlloc((void**)&mHostPtr, size, cudaHostAllocMapped);
  if (!mHostPtr) {
    return;
  }
  cudaHostGetDevicePointer((void**)&mDevicePtr, (void*)mHostPtr, 0);
  if (!mDevicePtr) {
    return;
  }
  mStatus = true;
}
size_t SharedBuffer::GetNBytes() { return mSize * TrtCommon::getElementSize(mType); }
SharedBuffer::~SharedBuffer() {
  if (mHostPtr) {
    cudaFreeHost(mHostPtr);
    mHostPtr = nullptr;
  }
}

bool SharedBufferManager::InitBufferDynamic(nvinfer1::ICudaEngine* engine, int batch) {
  mBatchSize = batch;
  mEngine = engine;
  for (int i = 0; i < mEngine->getNbBindings(); i++) {
    auto dims = mEngine->getBindingDimensions(i);
    int64_t vol = 1;
    nvinfer1::DataType type = mEngine->getBindingDataType(i);
    int vecDim = mEngine->getBindingVectorizedDim(i);
    if (-1 != vecDim) {
      int scalarsPerVec = mEngine->getBindingComponentsPerElement(i);
      dims.d[vecDim] = divUp(dims.d[vecDim], scalarsPerVec);
      vol *= scalarsPerVec;
    }
    vol *= TrtCommon::volume(dims);
    if (vol < 0) {
      vol *= (-1 * mBatchSize);
    }
    std::vector<int> real_dims;
    for (auto j = 0; j < dims.nbDims; j++) {
      if (dims.d[j] < 0) {
        real_dims.push_back(mBatchSize);
      } else {
        real_dims.push_back(dims.d[j]);
      }
    }
    if (engine->bindingIsInput(i)) {
      mInputDims[mEngine->getBindingName(i)] = real_dims;
    } else {
      mOutputDims[mEngine->getBindingName(i)] = real_dims;
    }
    mIODtypes[mEngine->getBindingName(i)] = type;
    std::unique_ptr<SharedBuffer> manBuf{new SharedBuffer(vol, type)};
    mDeviceBindings.emplace_back(manBuf->mDevicePtr);
    mManagedBuffers.emplace_back(std::move(manBuf));
  }
}
void SharedBufferManager::InitBuffer(nvinfer1::ICudaEngine* engine, int batchSize) {
  if (mMaxBatchSize > 0) {
    mBatchSize = batchSize;
    return;
  }
  mEngine = engine;
  mBatchSize = batchSize;
  int n = mEngine->getNbOptimizationProfiles();
  int max_idx = n - 1;
  for (int i = 0; i < mEngine->getNbBindings(); i++) {
    std::vector<int> real_dims;
    nvinfer1::DataType type = mEngine->getBindingDataType(i);
    int64_t vol = 1;
    if (engine->bindingIsInput(i)) {
      nvinfer1::Dims max_profile_dims =
          mEngine->getProfileDimensions(i, max_idx, nvinfer1::OptProfileSelector::kMAX);
      vol = TrtCommon::volume(max_profile_dims);
      std::vector<int> real_dims;
      mMaxBatchSize = max_profile_dims.d[0];
      real_dims.push_back(batchSize);
      for (int k = 1; k < max_profile_dims.nbDims; k++) {
        real_dims.push_back(max_profile_dims.d[k]);
      }
      mInputDims[mEngine->getBindingName(i)] = real_dims;
    } else {
      auto dims = mEngine->getBindingDimensions(i);
      int vecDim = mEngine->getBindingVectorizedDim(i);
      if (-1 != vecDim) {
        int scalarsPerVec = mEngine->getBindingComponentsPerElement(i);
        dims.d[vecDim] = divUp(dims.d[vecDim], scalarsPerVec);
        vol *= scalarsPerVec;
      }
      vol *= TrtCommon::volume(dims);
      if (vol < 0) {
        vol *= (-1 * mMaxBatchSize);
      }
      real_dims.push_back(mBatchSize);
      for (auto j = 1; j < dims.nbDims; j++) {
        if (dims.d[j] < 0) {
          real_dims.push_back(mBatchSize);
        } else {
          real_dims.push_back(dims.d[j]);
        }
      }
      mOutputDims[mEngine->getBindingName(i)] = real_dims;
    }
    mIODtypes[mEngine->getBindingName(i)] = type;
    std::unique_ptr<SharedBuffer> manBuf{new SharedBuffer(vol, type)};
    mDeviceBindings.emplace_back(manBuf->mDevicePtr);
    mManagedBuffers.emplace_back(std::move(manBuf));
  }
}

int64_t SharedBufferManager::GetIONbytesByName(std::string& name) {
  std::vector<int> shape;
  int64_t vol = 0;
  if (mInputDims.find(name) != mInputDims.end()) {
    shape = mInputDims[name];
  } else if (mOutputDims.find(name) != mOutputDims.end()) {
    shape = mOutputDims[name];
  }
  if (shape.size() > 0) {
    nvinfer1::DataType dtype = mIODtypes[name];
    vol = std::accumulate(shape.begin(), shape.end(), 1, std::multiplies<int64_t>());
    vol *= TrtCommon::getElementSize(dtype);
  }
  return vol;
}

void* SharedBufferManager::getBuffer(const bool isHost, const std::string& tensorName) const {
  int index = mEngine->getBindingIndex(tensorName.c_str());
  if (index == -1) return nullptr;
  if (isHost) {
    return mManagedBuffers[index]->mHostPtr;
  } else {
    return mManagedBuffers[index]->mDevicePtr;
  }
}
//获取指定输入输出为name的显存地址
void* SharedBufferManager::getDeviceBuffer(const std::string& tensorName) const {
  return getBuffer(false, tensorName);
}
//获取指定输入输出为name的内存地址
void* SharedBufferManager::getHostBuffer(const std::string& tensorName) const {
  return getBuffer(true, tensorName);
}

}  // namespace TrtCommon