#include "trt_run.h"

using namespace Trt;

bool TrtModel::BuildTrt() {
  initLibNvInferPlugins(&Trt::gLogger.getTRTLogger(), "");
  // De-serialize engine from file
  std::ifstream engineFile(locateFile(mParams.modelName, mParams.modelDirs).c_str(),
                           std::ios::binary);
  if (engineFile.fail()) {
    return false;
  }
  engineFile.seekg(0, std::ifstream::end);
  auto fsize = engineFile.tellg();
  engineFile.seekg(0, std::ifstream::beg);

  std::vector<char> engineData(fsize);
  engineFile.read(engineData.data(), fsize);
  nvinfer1::IRuntime* runtime = nvinfer1::createInferRuntime(Trt::gLogger.getTRTLogger());
  // runtime->setDLACore(0);

  nvinfer1::ICudaEngine* engine = runtime->deserializeCudaEngine(engineData.data(), fsize);
  if (!engine) {
    std::cout << "deserilize failed,engine:" << engine << std::endl;
    return false;
  }
  mEngine.reset(engine);
  std::cout << "deserial ok!" << std::endl;
  Init();
  runtime->destroy();
  // engineFile.close();
  return true;
}

bool TrtModel::setProfile(TrtUniquePtr<nvinfer1::ICudaEngine>& engine,
                          TrtUniquePtr<nvinfer1::IExecutionContext>& context, Shapebing& io_shape) {
  if (io_shape.empty()) {
    std::cout << "using static shape?" << std::endl;
    return false;
  }
  for (int i = 0; i < engine->getNbBindings(); i++) {
    bool is_input = engine->bindingIsInput(i);
    auto dims = context->getBindingDimensions(i);
    if (is_input) {
      const bool isScalar = dims.nbDims == 0;
      const bool isDynamicInput =
          std::any_of(dims.d, dims.d + dims.nbDims, [](int dim) { return dim == -1; }) ||
          engine->isShapeBinding(i);
      if (isDynamicInput) {
        auto shape = io_shape.find(engine->getBindingName(i));
        std::vector<int> staticDims;
        if (shape == io_shape.end()) {
          constexpr int DEFAULT_DIMENSION = 1;
          if (engine->isShapeBinding(i)) {
            if (isScalar) {
              staticDims.push_back(1);
            } else {
              staticDims.resize(dims.d[0]);
              std::fill(staticDims.begin(), staticDims.end(), DEFAULT_DIMENSION);
            }
          } else {
            staticDims.resize(dims.nbDims);
            std::transform(dims.d, dims.d + dims.nbDims, staticDims.begin(),
                           [&](int dim) { return dim >= 0 ? dim : DEFAULT_DIMENSION; });
          }
          std::cout << "Dynamic dimensions required for input: " << engine->getBindingName(i)
                    << ", but no shapes were provided. Automatically overriding shape to: "
                    << staticDims.data() << std::endl;
        } else {
          staticDims = shape->second;
        }
        if (engine->isShapeBinding(i)) {
          if (!context->setInputShapeBinding(i, staticDims.data())) {
            return false;
          }
        } else {
          if (!context->setBindingDimensions(i, toDims(staticDims))) {
            return false;
          }
        }
      }
    }
  }

  return true;
}

bool TrtModel::infer(TrtCommon::BufferManager& buffers) {
  // Memcpy from host input buffers to device input buffers
  buffers.copyInputToDevice();
  setProfile(mEngine, mContex, buffers.GetInputDims());

  bool status = this->mContex->executeV2(buffers.getDeviceBindings().data());
  if (!status) {
    return false;
  }

  // Memcpy from device output buffers to host output buffers
  buffers.copyOutputToHost();

  return true;
}
bool TrtModel::infer_v2(TrtCommon::SharedBufferManager& sm) {
  bool status = this->mContex->executeV2(sm.getDeviceBindings().data());
  if (!status) {
    return false;
  }
  return true;
}

bool TrtModel::WarnUp() {
  TrtCommon::BufferManager buffers;
  buffers.InitBuffer(GetCudaEngine().get(), mMaxBatch);
  if (!infer(buffers)) {
    std::cout << "WarnUp model faild" << std::endl;
    return false;
  }
  // std::cout << "warm up succ!" << std::endl;
  return true;
}

void TrtModel::Init() {
  int n = mEngine->getNbOptimizationProfiles();
  // nvinfer1::Dims
  // std::cout << "num of profiles:" << n << std::endl;
  int max_idx = n - 1;
  for (int i = 0; i < mEngine->getNbBindings(); i++) {
    std::vector<int> real_dims;
    if (mEngine->bindingIsInput(i)) {
      nvinfer1::Dims max_profile_dims =
          mEngine->getProfileDimensions(i, max_idx, nvinfer1::OptProfileSelector::kMAX);
      mMaxBatch = max_profile_dims.d[0];
      nvinfer1::DataType type = mEngine->getBindingDataType(i);
      auto vol = TrtCommon::volume(max_profile_dims) * TrtCommon::getElementSize(type);
      mEngineDimSize[mEngine->getBindingName(i)] = vol / mMaxBatch;
      real_dims.push_back(1);
      for (int j = 1; j < max_profile_dims.nbDims; j++) {
        real_dims.push_back(max_profile_dims.d[j]);
      }
      mEngineInputDims[mEngine->getBindingName(i)] = real_dims;
      mInputNames.push_back(mEngine->getBindingName(i));
    } else {
      auto dims = mEngine->getBindingDimensions(i);
      int64_t vol = 1;
      nvinfer1::DataType type = mEngine->getBindingDataType(i);
      vol *= TrtCommon::volume(dims);
      vol *= TrtCommon::getElementSize(type);
      if (vol < 0) {
        vol *= (-1 * mMaxBatch);
      }
      for (auto j = 0; j < dims.nbDims; j++) {
        if (dims.d[j] < 0) {
          real_dims.push_back(1);
        } else {
          real_dims.push_back(dims.d[j]);
        }
      }
      mEngineDimSize[mEngine->getBindingName(i)] = vol / mMaxBatch;
      mEngineOutputDims[mEngine->getBindingName(i)] = real_dims;
      mOutputNames.push_back(mEngine->getBindingName(i));
    }
  }
  mContex.reset(mEngine->createExecutionContext());
  setProfile(mEngine, mContex, mEngineInputDims);
}

const std::vector<std::string>& TrtModel::GetOutputNames() { return mOutputNames; }
const std::vector<std::string>& TrtModel::GetInputNames() { return mInputNames; }
