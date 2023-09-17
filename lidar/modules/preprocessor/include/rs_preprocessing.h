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
#ifndef RS_PREPROCESSING_SUBS_RS_RS_PREPROCESSING_H_
#define RS_PREPROCESSING_SUBS_RS_RS_PREPROCESSING_H_

#include "common/include/common.h"
#include "common/include/msg/lidar_frame_msg.h"
#include "common/include/basic_type/range.h"
#include "common/include/config/rs_yamlReader.h"

namespace robosense{

const static char _vehicle_filter[] = "vehicle_filter";
const static char _range_filter[] = "range_filter";
const static char _enable[] = "enable";

class RsPreprocessing{
public:
    using Ptr = std::shared_ptr<RsPreprocessing>;

    RsPreprocessing() = default;
    void init(const Rs_YAMLReader &configParse);
    void perception(const LidarFrameMsg::Ptr& msg_ptr);

private:
    void Filter(const LidarFrameMsg::Ptr& msg_ptr);
    void noisePoint(const LidarFrameMsg::Ptr &msg_ptr);

private:
    PreprocessingParam params;

};

}  // namespace robosense

#endif  // RS_PREPROCESSING_SUBS_RS_RS_PREPROCESSING_H_
