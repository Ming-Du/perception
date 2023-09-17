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

#include "rs_perception/infer/infer.h"
#include "rs_perception/infer/rs_trt_infer.h"

namespace robosense {

RsTrtInfer::RsTrtInfer() : impl(new RsTrtInferImpl) {}
RsTrtInfer::~RsTrtInfer() { delete impl; }

void RsTrtInfer::init(const RsInferConfig& configs) {
    impl->init(configs);
}
void RsTrtInfer::infer() {
    impl->infer();
}

const std::vector<void*> RsTrtInfer::getInputs() const {
    return impl->getInputs();
}

const std::vector<void*> RsTrtInfer::getOutputs() const {
    return impl->getOutputs();
}

const std::vector<size_t > RsTrtInfer::getInputSizes() const {
    return impl->getInputSizes();
}

const std::vector<size_t > RsTrtInfer::getOutputSizes() const {
    return impl->getOutputSizes();
}

void RsTrtInfer::inferWithCpus(const std::vector<float*>& inputs, const std::vector<float*>& outputs) {
    impl->inferWithCpus(inputs, outputs);
}

void RsTrtInfer::inferWithCpuInputs(const std::vector<float*>& inputs) {
    impl->inferWithCpuInputs(inputs);
}

void RsTrtInfer::inferWithCpuOutputs(const std::vector<float*>& outputs) {
    impl->inferWithCpuOutputs(outputs);
}


}  // namespace robosense
