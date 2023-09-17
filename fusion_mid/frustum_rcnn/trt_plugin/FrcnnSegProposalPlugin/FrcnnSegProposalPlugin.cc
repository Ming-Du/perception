#include "FrcnnSegProposalPlugin.h"

#include <assert.h>

#include <chrono>

#include "../common/serialize.h"
#include "FrcnnSegProposal.h"


namespace amirstan {
namespace plugin {

namespace {
static const char *PLUGIN_VERSION{"1"};
static const char *PLUGIN_NAME{"FrcnnSegProposal"};
}  // namespace

void FrcnnSegProposalPlugin::setPluginNamespace(const char* np)  PLUGIN_NOEXCEPT{
      this->mNamespace = np;
};
const char* FrcnnSegProposalPlugin::getPluginNamespace() const PLUGIN_NOEXCEPT { 
    return this->mNamespace.c_str();
};

FrcnnSegProposalPlugin::FrcnnSegProposalPlugin(const std::string &name, const int32_t point_num)
    : PluginDynamicBase(name),mpoint_num(point_num){
}

FrcnnSegProposalPlugin::FrcnnSegProposalPlugin(const std::string name,
                                             const void *data, size_t length)
    : PluginDynamicBase(name) {
    deserialize_value(&data, &length, &mpoint_num);
}


nvinfer1::IPluginV2DynamicExt *FrcnnSegProposalPlugin::clone() const
    PLUGIN_NOEXCEPT {
  FrcnnSegProposalPlugin *plugin = new FrcnnSegProposalPlugin(mLayerName, mpoint_num);
  plugin->setPluginNamespace(getPluginNamespace());

  return plugin;
}

nvinfer1::DimsExprs FrcnnSegProposalPlugin::getOutputDimensions(
    int outputIndex, const nvinfer1::DimsExprs *inputs, int nbInputs,
    nvinfer1::IExprBuilder &exprBuilder) PLUGIN_NOEXCEPT {
  nvinfer1::DimsExprs ret;
  ret.d[0] = inputs[0].d[0];
  if(outputIndex == 0){
    ret.nbDims = 2;
    ret.d[1] = exprBuilder.constant(3);
  }
  else if(outputIndex == 1){
    ret.nbDims = 4;
    ret.d[1] = inputs[0].d[1];
    ret.d[2] = exprBuilder.constant(mpoint_num);
    ret.d[3] = exprBuilder.constant(1);
  }
  else{
    std::cout << outputIndex << "is invalid" << std::endl;
  }
  return ret;
}

bool FrcnnSegProposalPlugin::supportsFormatCombination(
    int pos, const nvinfer1::PluginTensorDesc *inOut, int nbInputs,
    int nbOutputs) PLUGIN_NOEXCEPT {
  // const auto *in = inOut;
  // const auto *out = inOut + nbInputs;
  bool condition = inOut[pos].format == nvinfer1::TensorFormat::kLINEAR;
   condition &= (inOut[pos].type == nvinfer1::DataType::kFLOAT);
   condition &= inOut[pos].type == inOut[0].type;
   return condition;
}

void FrcnnSegProposalPlugin::configurePlugin(
    const nvinfer1::DynamicPluginTensorDesc *inputs, int nbInputs,
    const nvinfer1::DynamicPluginTensorDesc *outputs,
    int nbOutputs) PLUGIN_NOEXCEPT {
  // Validate input arguments
}

size_t FrcnnSegProposalPlugin::getWorkspaceSize(
    const nvinfer1::PluginTensorDesc *inputs, int nbInputs,
    const nvinfer1::PluginTensorDesc *outputs,
    int nbOutputs) const PLUGIN_NOEXCEPT {
  return 0;
}

int FrcnnSegProposalPlugin::enqueue(const nvinfer1::PluginTensorDesc *inputDesc,
                                   const nvinfer1::PluginTensorDesc *outputDesc,
                                   const void *const *inputs,
                                   void *const *outputs, void *workSpace,
                                   cudaStream_t stream) PLUGIN_NOEXCEPT {
  nvinfer1::Dims pcd_xyzi_dims = inputDesc[0].dims;
  nvinfer1::Dims pred_seg_dims = inputDesc[1].dims;

  nvinfer1::Dims pcd_center_dims = outputDesc[0].dims;
  nvinfer1::Dims pcd_out_dims = outputDesc[1].dims;

  auto data_type = inputDesc[0].type;
  
  int64_t BS = pcd_xyzi_dims.d[0];
  int64_t C = pcd_xyzi_dims.d[1];
  int64_t N = pcd_xyzi_dims.d[2];
  int64_t N_out = pcd_out_dims.d[2];
  switch (data_type) {
    case nvinfer1::DataType::kFLOAT:
        frcnn_seg_proposal_cuda((float*)(inputs[0]), (float*)(inputs[1]), (float*)(outputs[0]), (float*)(outputs[1]),
        BS, C, N, N_out, stream);
        break;
    case nvinfer1::DataType::kHALF:
        frcnn_seg_proposal_cuda_half((half*)(inputs[0]), (half*)(inputs[1]), (half*)(outputs[0]), (half*)(outputs[1]),
        BS, C, N, N_out, stream);
        break;
    default:
      return 1;
  }

  return 0;
}

nvinfer1::DataType FrcnnSegProposalPlugin::getOutputDataType(
    int index, const nvinfer1::DataType *inputTypes,
    int nbInputs) const PLUGIN_NOEXCEPT {
  return inputTypes[0];
}

// IPluginV2 Methods
const char *FrcnnSegProposalPlugin::getPluginType() const PLUGIN_NOEXCEPT {
  return PLUGIN_NAME;
}

const char *FrcnnSegProposalPlugin::getPluginVersion() const PLUGIN_NOEXCEPT {
  return PLUGIN_VERSION;
}

int FrcnnSegProposalPlugin::getNbOutputs() const PLUGIN_NOEXCEPT {
   return 2; 
}

size_t FrcnnSegProposalPlugin::getSerializationSize() const PLUGIN_NOEXCEPT {
  return sizeof(mpoint_num);
}

void FrcnnSegProposalPlugin::serialize(void *buffer) const PLUGIN_NOEXCEPT {
    serialize_value(&buffer, mpoint_num);
}


////////////////////// creator /////////////////////////////

FrcnnSegProposalPluginCreator::FrcnnSegProposalPluginCreator() {
  mPluginAttributes = std::vector<PluginField>();
  mPluginAttributes.emplace_back(PluginField("point_num", nullptr, PluginFieldType::kINT32, 1));

  mFC.nbFields = mPluginAttributes.size();
  mFC.fields = mPluginAttributes.data();
}

const char *FrcnnSegProposalPluginCreator::getPluginName() const
    PLUGIN_NOEXCEPT {
  return PLUGIN_NAME;
}

const char *FrcnnSegProposalPluginCreator::getPluginVersion() const
    PLUGIN_NOEXCEPT {
  return PLUGIN_VERSION;
}

IPluginV2 *FrcnnSegProposalPluginCreator::createPlugin(
    const char *name, const PluginFieldCollection *fc) PLUGIN_NOEXCEPT {
  int32_t point_num = 0;
  for (int i = 0; i < fc->nbFields; i++) {
    if (fc->fields[i].data == nullptr) {
      continue;
    }
    std::string field_name(fc->fields[i].name);

    if (field_name.compare("point_num") == 0) {
      point_num = static_cast<const int32_t*>(fc->fields[i].data)[0];
    }
  }

  FrcnnSegProposalPlugin *plugin = new FrcnnSegProposalPlugin(name, point_num);
  plugin->setPluginNamespace(getPluginNamespace());
  return plugin;
}

IPluginV2 *FrcnnSegProposalPluginCreator::deserializePlugin(
    const char *name, const void *serialData,
    size_t serialLength) PLUGIN_NOEXCEPT {
  // This object will be deleted when the network is destroyed, which will
  auto plugin = new FrcnnSegProposalPlugin(name, serialData, serialLength);
  plugin->setPluginNamespace(getPluginNamespace());
  return plugin;
}

}  // namespace plugin
}  // namespace 