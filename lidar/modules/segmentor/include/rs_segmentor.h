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
#ifndef RS_SEGMENTOR_SUBS_RS_RS_SEGMENTOR_H_
#define RS_SEGMENTOR_SUBS_RS_RS_SEGMENTOR_H_

#include <geometry_msgs/Point.h>
#include <iomanip>

#include "common/include/basic_type/range.h"
#include "common/include/common.h"
#include "common/include/msg/lidar_frame_msg.h"
#include "common/include/rs_define.h"
#include "denseg.h"

#include "common/proto/convexhull.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/hadmap.pb.h"
#include "common/proto/localization.pb.h"
#include "common/proto/object.pb.h"

namespace robosense {

class RsSegmentor {
public:
    using Ptr = std::shared_ptr<RsSegmentor>;

    RsSegmentor() = default;
    void init(const Rs_YAMLReader &configParse);
    int split_large_obj(const LidarFrameMsg::Ptr &msg_ptr, std::vector<Object::Ptr> &obj_vec, int start_id);
    int split_floating_obj(const LidarFrameMsg::Ptr &msg_ptr, std::vector<Object::Ptr> &obj_vec, int start_id);
    void perception(const LidarFrameMsg::Ptr &msg_ptr, const localization::Localization &local_current);

private:
    SegmentorParam params;
    
    DenseSeg denseseg_;
    bool init_denseg = true;
};

} // namespace robosense
#endif // RS_SEGMENTOR_SUBS_RS_RS_SEGMENTOR_H_
