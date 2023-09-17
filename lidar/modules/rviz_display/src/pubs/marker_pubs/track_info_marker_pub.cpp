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
#include "pubs/marker_pubs/track_info_marker_pub.h"

namespace robosense {

std::vector<ROS_VISUALIZATION_MARKER> &TrackInfoPubMarker::display(const LidarFrameMsg::Ptr &msg_ptr){

    const auto& obj_vec = msg_ptr->objects;
    if (params_.max_obj_size < obj_vec.size()){
        params_.max_obj_size = obj_vec.size();
        params_.marker_list.resize(params_.max_obj_size);
    }

    for (size_t i = 0; i < params_.marker_list.size(); ++i) {
        auto& tmp_maker = params_.marker_list[i];
        tmp_maker.color = params_.default_color_type;
        tmp_maker.scale = params_.default_scale_type;
        tmp_maker.ns = "track_info";
        tmp_maker.id = i;
        tmp_maker.header.frame_id = options_.frame_id;
    }

    for (size_t i = 0; i < obj_vec.size(); ++i) {
        const auto& tmp_obj = obj_vec[i];

        if(ObjectType::UNKNOW == tmp_obj->type)
        {
//            continue;
        }
        const auto &center = tmp_obj->center;
        const auto &size = tmp_obj->size;
        double velocity = tmp_obj->velocity.norm();
        auto& tmp_marker = params_.marker_list[i];

        tmp_marker.color.r = 1.;
        tmp_marker.color.g = 1.;
        tmp_marker.color.b = 1.;


        Eigen::Vector3d pos = center;
        pos.z() += size.z() * 0.5f + 0.7f;

        // std::string text_track =
        //     "<" + num2str<int>(tmp_obj->tracker_id, 0) + ">" + num2str<double>(velocity * 3.6f, 1) + "km/h  " + kRoadType2NameMap.at(tmp_obj->status) + " " + kDetectState2NameMap.at(tmp_obj->object_state.detect_state)+" " + kGroupType2NameMap.at(tmp_obj->group);
        std::string text_track =
            "<" + num2str<int>(tmp_obj->tracker_id, 0) + ">" + num2str<double>(velocity * 3.6f, 1) + "km/h " + kRoadType2NameMap.at(tmp_obj->status) + " " + num2str<double>(tmp_obj->type_confidence, 2) + " " + kObjectType2NameMap.at(tmp_obj->type);

        drawText(pos, text_track, tmp_marker, 1.);

    }

    return params_.marker_list;
}


}