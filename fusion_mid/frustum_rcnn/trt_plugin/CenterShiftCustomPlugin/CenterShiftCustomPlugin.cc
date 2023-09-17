#include "CenterShiftCustomPlugin.h"

#include <assert.h>

#include <chrono>

#include "../common/serialize.h"
#include "CenterShiftCustom.h"


namespace amirstan {
namespace plugin {

namespace {
static const char *PLUGIN_VERSION{"1"};
static const char *PLUGIN_NAME{"CenterShiftCustom"};
}  // namespace

void CenterShiftCustomPlugin::setPluginNamespace(const char* np)  PLUGIN_NOEXCEPT{
      this->mNamespace = np;
};
const char* CenterShiftCustomPlugin::getPluginNamespace() const PLUGIN_NOEXCEPT { 
    return this->mNamespace.c_str();
};

CenterShiftCustomPlugin::CenterShiftCustomPlugin(const std::string &name)
    : PluginDynamicBase(name){
}

CenterShiftCustomPlugin::CenterShiftCustomPlugin(const std::string name,
                                             const void *data, size_t length)
    : PluginDynamicBase(name) {
}


nvinfer1::IPluginV2DynamicExt *CenterShiftCustomPlugin::clone() const
    PLUGIN_NOEXCEPT {
  CenterShiftCustomPlugin *plugin = new CenterShiftCustomPlugin(mLayerName);
  plugin->setPluginNamespace(getPluginNamespace());

  return plugin;
}

nvinfer1::DimsExprs CenterShiftCustomPlugin::getOutputDimensions(
    int outputIndex, const nvinfer1::DimsExprs *inputs, int nbInputs,
    nvinfer1::IExprBuilder &exprBuilder) PLUGIN_NOEXCEPT {
  nvinfer1::DimsExprs ret = inputs[0];
  return ret;
}

bool CenterShiftCustomPlugin::supportsFormatCombination(
    int pos, const nvinfer1::PluginTensorDesc *inOut, int nbInputs,
    int nbOutputs) PLUGIN_NOEXCEPT {

  bool condition = inOut[pos].format == nvinfer1::TensorFormat::kLINEAR;
   condition &= (inOut[pos].type == nvinfer1::DataType::kFLOAT);  
   condition &= inOut[pos].type == inOut[0].type;
   return condition;
}

void CenterShiftCustomPlugin::configurePlugin(
    const nvinfer1::DynamicPluginTensorDesc *inputs, int nbInputs,
    const nvinfer1::DynamicPluginTensorDesc *outputs,
    int nbOutputs) PLUGIN_NOEXCEPT {
  // Validate input arguments
}

size_t CenterShiftCustomPlugin::getWorkspaceSize(
    const nvinfer1::PluginTensorDesc *inputs, int nbInputs,
    const nvinfer1::PluginTensorDesc *outputs,
    int nbOutputs) const PLUGIN_NOEXCEPT {
  return 0;
}

int CenterShiftCustomPlugin::enqueue(const nvinfer1::PluginTensorDesc *inputDesc,
                                   const nvinfer1::PluginTensorDesc *outputDesc,
                                   const void *const *inputs,
                                   void *const *outputs, void *workSpace,
                                   cudaStream_t stream) PLUGIN_NOEXCEPT {
  auto data_type = inputDesc[0].type;
  
  int64_t BS = inputDesc[0].dims.d[0];
  int64_t C = inputDesc[0].dims.d[1];
  int64_t N = inputDesc[0].dims.d[2];
  switch (data_type) {
    case nvinfer1::DataType::kFLOAT:
        center_shift_cuda((float*)inputs[0], (float*)inputs[1], (float*)outputs[0], BS, C, N);
        break;
    case nvinfer1::DataType::kHALF:
        center_shift_cuda_half((half*)inputs[0], (half*)inputs[1], (half*)outputs[0], BS, C, N);
        break;
    default:
      return 1;
  }

  return 0;
}

nvinfer1::DataType CenterShiftCustomPlugin::getOutputDataType(
    int index, const nvinfer1::DataType *inputTypes,
    int nbInputs) const PLUGIN_NOEXCEPT {
  return inputTypes[0];
}

// IPluginV2 Methods
const char *CenterShiftCustomPlugin::getPluginType() const PLUGIN_NOEXCEPT {
  return PLUGIN_NAME;
}

const char *CenterShiftCustomPlugin::getPluginVersion() const PLUGIN_NOEXCEPT {
  return PLUGIN_VERSION;
}

int CenterShiftCustomPlugin::getNbOutputs() const PLUGIN_NOEXCEPT {
   return 1; 
}

size_t CenterShiftCustomPlugin::getSerializationSize() const PLUGIN_NOEXCEPT {
  return 0;
}

void CenterShiftCustomPlugin::serialize(void *buffer) const PLUGIN_NOEXCEPT {
}


////////////////////// creator /////////////////////////////

CenterShiftCustomPluginCreator::CenterShiftCustomPluginCreator() {
  mFC.nbFields = mPluginAttributes.size();
  mFC.fields = mPluginAttributes.data();
}

const char *CenterShiftCustomPluginCreator::getPluginName() const
    PLUGIN_NOEXCEPT {
  return PLUGIN_NAME;
}

const char *CenterShiftCustomPluginCreator::getPluginVersion() const
    PLUGIN_NOEXCEPT {
  return PLUGIN_VERSION;
}

IPluginV2 *CenterShiftCustomPluginCreator::createPlugin(
    const char *name, const PluginFieldCollection *fc) PLUGIN_NOEXCEPT {

  CenterShiftCustomPlugin *plugin = new CenterShiftCustomPlugin(name);
  plugin->setPluginNamespace(getPluginNamespace());
  return plugin;
}

IPluginV2 *CenterShiftCustomPluginCreator::deserializePlugin(
    const char *name, const void *serialData,
    size_t serialLength) PLUGIN_NOEXCEPT {
  // This object will be deleted when the network is destroyed, which will
  auto plugin = new CenterShiftCustomPlugin(name, serialData, serialLength);
  plugin->setPluginNamespace(getPluginNamespace());
  return plugin;
}

}  // namespace plugin
}  