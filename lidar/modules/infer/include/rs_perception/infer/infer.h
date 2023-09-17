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

#ifndef RS_PERCEPTION_INFER_EXTERNAL_INFER_H_
#define RS_PERCEPTION_INFER_EXTERNAL_INFER_H_

#include "rs_perception/infer/base_infer.h"

namespace robosense {

enum class InferenceEngine : int {
    TENSORRT = 0,
};

class RsTrtInferImpl;
class RsTrtInfer : virtual public BaseInfer {
public:
    using Ptr = std::shared_ptr<RsTrtInfer>;
    RsTrtInfer();
    ~RsTrtInfer();
    void init(const RsInferConfig& configs) override;
    void infer() override;
    const std::vector<void*> getInputs() const override;
    const std::vector<void*> getOutputs() const override;
    const std::vector<size_t > getInputSizes() const override;
    const std::vector<size_t > getOutputSizes() const override;
    void inferWithCpus(const std::vector<float*>& inputs, const std::vector<float*>& outputs) override;
    void inferWithCpuInputs(const std::vector<float*>& inputs) override;
    void inferWithCpuOutputs(const std::vector<float*>& outputs) override;
private:
    RsTrtInferImpl* impl;
};

}  // namespace robosense

#endif  // RS_PERCEPTION_INFER_EXTERNAL_INFER_H_
