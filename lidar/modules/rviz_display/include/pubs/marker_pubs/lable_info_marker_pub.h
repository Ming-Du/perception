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

#ifndef RS_SDK_LABLE_INFO_MARKER_PUB_H
#define RS_SDK_LABLE_INFO_MARKER_PUB_H
#include "common/base_marker_pub.h"
namespace robosense {
class LabelInfosMarkerPub : public BaseMarkerPub {
public:
    using Ptr = std::shared_ptr<LabelInfosMarkerPub>;

    LabelInfosMarkerPub() = default;

    void init(const MarkerPubOptions &options) override {
        if (options.node_ptr == nullptr) {
            return;
        }
        options_ = options;
    }

    std::vector<ROS_VISUALIZATION_MARKER> &display(const LidarFrameMsg::Ptr &msg_ptr) override;
    std::vector<ROS_VISUALIZATION_MARKER> &display_denoise_current(const LidarFrameMsg::Ptr &msg_ptr);
    std::vector<ROS_VISUALIZATION_MARKER> &display_denoise_history(const LidarFrameMsg::Ptr &msg_ptr);
    std::string name() override {
        return "LabelInfosMarkerPub";
    }
};
}

#endif //RS_SDK_LABLE_INFO_MARKER_PUB_H
