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

#ifndef RS_SDK_POINTPILLARSCENTERHEAD_AI_DETECTION_H_
#define RS_SDK_POINTPILLARSCENTERHEAD_AI_DETECTION_H_

#include "common/include/common.h"
#include "common/include/msg/lidar_frame_msg.h"
#include "rs_perception/infer/infer.h"
#include "pointpillars/params.h"
// #include "pointpillars/preprocess.h"
#include "pointpillars/preprocess_cuda.h"
// #include "pointpillars/postprocess.h"
#include "pointpillars/postprocess_cuda.h"

namespace robosense {

class PointPillarsCenterHeadAiDetection {
public:
    using Ptr = std::shared_ptr<PointPillarsCenterHeadAiDetection>;

    PointPillarsCenterHeadAiDetection() = default;
    
    void init(const Rs_YAMLReader &configParse);

    void perception(const LidarFrameMsg::Ptr& msg_ptr);

private:
    void initComponent();
    PointPillarsCenterHeadParams::Ptr params_ptr_;
    BaseInfer::Ptr infer_ptr_;
    // PointPillarsCenterHeadPreprocess::Ptr preprocess_ptr_;
    PointPillarsCenterHeadCudaPreprocess::Ptr preprocess_cuda_ptr_;
    // PointPillarsCenterHeadPostprocess::Ptr postprocess_ptr_;
    PointPillarsCenterHeadCudaPostprocess::Ptr postprocess_cuda_ptr_;

    bool first_process_ = true;

    //=========================================
    // conponent_cost related
    //=========================================
    struct Cost {
        int count = 0;
        double prepare = 0.;
        double preprocess = 0.;
        double infer = 0.;
        double postprocess = 0.;
        double all = 0.;
        double timer, tot_timer;
        void reset() {
            prepare = 0.;
            preprocess = 0.;
            infer = 0.;
            postprocess = 0.;
            all = 0.;
            count = 0;
        }
        void print(const std::string& name) {
            // std::cout << name << ": ***prepare avr cost " << prepare / (count + 1.e-6) << " ms." << std::endl;
            // std::cout << name << ": ***preprocess avr cost " << preprocess / (count + 1.e-6) << " ms." << std::endl;
            // std::cout << name << ": ***infer avr cost " << infer / (count + 1.e-6) << " ms." << std::endl;
            // std::cout << name << ": ***postprocess avr cost " << postprocess / (count + 1.e-6) << " ms." << std::endl;
            // std::cout << name << ": ***all avr cost " << all / (count + 1.e-6) << " ms." << std::endl;
            if (count > 10000) {
                reset();
            }
        }
    }component_cost;

 
};

}  // namespace robosense

#endif  // RS_SDK_POINTPILLARSCENTERHEAD_AI_DETECTION_H_
