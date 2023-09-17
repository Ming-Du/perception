#pragma once

#include "common/include/common.h"
#include "common/include/msg/lidar_frame_msg.h"
#include "lidarrcnn/params.h"
#include "lidarrcnn/postprocess.h"
#include "lidarrcnn/preprocess_cuda.h"
#include "rs_perception/infer/infer.h"

namespace robosense {

class LiDARRCNNAIRefine {
public:
    using Ptr = std::shared_ptr<LiDARRCNNAIRefine>;

    LiDARRCNNAIRefine() = default;

    void init(const Rs_YAMLReader &configParse);

    void perception(const LidarFrameMsg::Ptr &msg_ptr);

private:
    void initComponent();

private:
    LiDARRCNNParams::Ptr params_ptr_;
    BaseInfer::Ptr infer_ptr_;
    LiDARRCNNPreprocess::Ptr preprocess_cuda_ptr_;
    LiDARRCNNPostprocess::Ptr postprocess_ptr_;

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
        void print(const std::string &name) {
            // std::cout << name << ": ***prepare avr cost " << prepare / (count + 1.e-6) << " ms." << std::endl;
            // std::cout << name << ": ***preprocess avr cost " << preprocess / (count + 1.e-6) << " ms." << std::endl;
            // std::cout << name << ": ***infer avr cost " << infer / (count + 1.e-6) << " ms." << std::endl;
            // std::cout << name << ": ***postprocess avr cost " << postprocess / (count + 1.e-6) << " ms." << std::endl;
            // std::cout << name << ": ***all avr cost " << all / (count + 1.e-6) << " ms." << std::endl;
            if (count > 10000) {
                reset();
            }
        }
    } component_cost;
};

} // namespace robosense
