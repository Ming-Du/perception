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

#ifndef RS_PERCEPTION_INFER_EXTERNAL_BASE_INFER_H_
#define RS_PERCEPTION_INFER_EXTERNAL_BASE_INFER_H_

#include <vector>
#include <memory>
namespace robosense {

class RsInferConfig {
public:
    std::string trt_path = "";  // tensorrt model path string 
    std::string model_buffer = "";  // model buffer string
    size_t max_workspace = 30;      // max workspace, only support in tensorrt
    std::string gie_path = "";      // model cache path, only support in tensorrt
    int device_id = 0;              // which device to perform inference, only support in tensorrt
    int height = 0;                 // the engine inference height, only support in resizable engine
    int width = 0;                  // the engine inference width, only support in resizable engine
    bool enable_int8 = false;       // whether to perform int8 inference, only support int8 model
    bool enable_fp16 = false;       // whether to perform fp16 inference, only support in fp16-supported device
};

class BaseInfer {
public:
    virtual ~BaseInfer() {}

    using Ptr = std::shared_ptr<BaseInfer>;

    /**
     * @brief Initialize engine configs. init() must be called before invoke infer or other functions
     * 
     * @param RsInferConfig 
     */
    virtual void init(const RsInferConfig& configs) = 0;

    /**
     * @brief Perform inference, before call this function, getInputs() and getOutputs() must
     *        be called to set the input and output pointer, eg.
     * 
     *        auto infer_ptr = createInferEngine(InferenceEngine::TENSORRT);
     *        auto input_ptr = infer_ptr->getInputs();
     *        input_ptr[0] = new float[INPUT_FEATURE_SIZE];
     *        auto output_ptr = infer_ptr->getOutputs();
     *        output_ptr[0] = new float[OUTPUT_FEATURE_SIZE];
     *        infer_ptr->infer();
     */
    virtual void infer() = 0;

    /**
     * @brief Get the pointer pointing to the input data of the engine
     * 
     * @return const std::vector<void*> 
     */
    virtual const std::vector<void*> getInputs() const = 0;

    /**
     * @brief Get the pointer pointing to the output data of the engine
     * 
     * @return const std::vector<void*> 
     */
    virtual const std::vector<void*> getOutputs() const = 0;

    /**
     * @brief Get the input sizes of the engine
     * 
     * @return const std::vector<size_t > 
     */
    virtual const std::vector<size_t > getInputSizes() const = 0;

    /**
     * @brief Get the output sizes of the engine
     * 
     * @return const std::vector<size_t > 
     */
    virtual const std::vector<size_t > getOutputSizes() const = 0;

    /**
     * @brief Another function to perform inference, runing in cpu
     * 
     * @param inputs vector<float*>, the size of vector indicates the number of CNN input,
     *               and in general the number of input is 1. Each element of the vector
     *               indicate the pointer to the allocated memory.
     * @param outputs vector<float*>, the size of vector indicates the number of CNN output.
     *               Each element of the vector indicate the pointer to the allocated memory.
     */
    virtual void inferWithCpus(const std::vector<float*>& inputs, const std::vector<float*>& outputs) = 0;

    /**
     * @brief Another function to perform inference, runing in cpu. Before call this function,
     *        the allocated memory of the output should be assigned first, then through getOutputs() to pass
     *        the pointer of the allocated memory.
     * 
     * @param vector<float*> 
     */
    virtual void inferWithCpuInputs(const std::vector<float*>& inputs) = 0;

    /**
     * @brief Another function to perform inference, runing in cpu. Before call this function,
     *        the allocated memory of the input should be assigned first, then through getInputs() to pass
     *        the pointer of the allocated memory.
     * 
     * @param vector<float*> 
     */
    virtual void inferWithCpuOutputs(const std::vector<float*>& outputs) = 0;
};

}  // namespace robosense

#endif  // RS_PERCEPTION_INFER_EXTERNAL_BASE_INFER_H_
