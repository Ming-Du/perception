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
#include "pubs/marker_pubs/lable_info_marker_pub.h"

namespace robosense {

std::vector<ROS_VISUALIZATION_MARKER> &LabelInfosMarkerPub::display(const LidarFrameMsg::Ptr &msg_ptr) {

    const auto& obj_vec = msg_ptr->objects;
    if (params_.max_obj_size < obj_vec.size()) {
        params_.max_obj_size = obj_vec.size();
        params_.marker_list.resize(params_.max_obj_size);
    }

    for (size_t i = 0; i < params_.marker_list.size(); ++i) {
        auto& tmp_maker = params_.marker_list[i];
        tmp_maker.color = params_.default_color_type;
        tmp_maker.scale = params_.default_scale_type;
        tmp_maker.ns = "label_info";
        tmp_maker.id = i;
        tmp_maker.header.frame_id = options_.frame_id;
    }

    for (size_t i = 0; i < obj_vec.size(); ++i) {
        const auto& tmp_obj = obj_vec[i];

        const auto &color = getClassColor(tmp_obj);

        const auto &center = tmp_obj->center;
        const auto &size = tmp_obj->size;
        auto& tmp_marker = params_.marker_list[i];

        tmp_marker.color.r = 1.;
        tmp_marker.color.g = 1.;
        tmp_marker.color.b = 1.;
        Eigen::Vector3d pos = center;
        pos.z() += size.z() * 0.5f + 1.5f;
        std::string text_label  = kObjectType2NameMap.at(tmp_obj->type);
        text_label += "," + num2str<double>(tmp_obj->type_confidence, 3);

        drawText(pos, text_label, tmp_marker, 1.);
    }

    return params_.marker_list;
}

std::vector<ROS_VISUALIZATION_MARKER> &LabelInfosMarkerPub::display_denoise_current(const LidarFrameMsg::Ptr &msg_ptr) {

    const auto& obj_vec = msg_ptr->objects;
    if (params_.max_obj_size < obj_vec.size()) {
        params_.max_obj_size = obj_vec.size();
        params_.marker_list.resize(params_.max_obj_size);
    }

    for (size_t i = 0; i < params_.marker_list.size(); ++i) {
        auto& tmp_maker = params_.marker_list[i];
        tmp_maker.color = params_.default_color_type;
        tmp_maker.scale = params_.default_scale_type;
        tmp_maker.ns = "denoise_current";
        tmp_maker.id = i;
        tmp_maker.header.frame_id = options_.frame_id;
    }
    std::map<NoiseState, std::string> myMap = {
        {NoiseState::NOISE_OBJECT, "Obj"},
        {NoiseState::NOISE_NOISE, "Noi"},
        {NoiseState::NOISE_SUSPECTED, "Sus"},
        {NoiseState::NOISE_FLOWERBEDS,"Flow"}
    };
    for (size_t i = 0; i < obj_vec.size(); ++i) {
        const auto& tmp_obj = obj_vec[i];

        ROS_VISUALIZATION_MARKER::_color_type tmp;
        //绿色
        tmp.r = 0.0;
        tmp.g = 1.0;
        tmp.b = 0.0;
        const auto &color = tmp;

        const auto &center = tmp_obj->center;
        const auto &size = tmp_obj->size;
        auto& tmp_marker = params_.marker_list[i];

        tmp_marker.color.r = 1.;
        tmp_marker.color.g = 0.5;
        tmp_marker.color.b = 0.5;
        Eigen::Vector3d pos = center;
        pos.z() += size.z() * 0.5f + 1.5f;
        pos.y() +=0.8f;
        std::string text_label;
        if(tmp_obj->type == ObjectType::UNKNOW){
            text_label  = myMap.at(tmp_obj->denoise_state_frame);
            text_label += "[" + num2str<double>(tmp_obj->denoise_grid_weight, 3)+","+num2str<double>(tmp_obj->denoise_obj_weight, 3)+"]";
        }else{
            text_label = "AI";
        }
        
        drawText(pos, text_label, tmp_marker, 1.);
    }

    return params_.marker_list;
}
std::vector<ROS_VISUALIZATION_MARKER> &LabelInfosMarkerPub::display_denoise_history(const LidarFrameMsg::Ptr &msg_ptr) {

    const auto& obj_vec = msg_ptr->objects;
    if (params_.max_obj_size < obj_vec.size()) {
        params_.max_obj_size = obj_vec.size();
        params_.marker_list.resize(params_.max_obj_size);
    }

    for (size_t i = 0; i < params_.marker_list.size(); ++i) {
        auto& tmp_maker = params_.marker_list[i];
        tmp_maker.color = params_.default_color_type;
        tmp_maker.scale = params_.default_scale_type;
        tmp_maker.ns = "denoise_history";
        tmp_maker.id = i;
        tmp_maker.header.frame_id = options_.frame_id;
    }
    std::map<NoiseState, std::string> myMap = {
        {NoiseState::NOISE_OBJECT, "O"},
        {NoiseState::NOISE_NOISE, "N"},
        {NoiseState::NOISE_SUSPECTED, "S"},
        {NoiseState::NOISE_FLOWERBEDS,"F"}
    };
    for (size_t i = 0; i < obj_vec.size(); ++i) {
        const auto& tmp_obj = obj_vec[i];
        ROS_VISUALIZATION_MARKER::_color_type tmp;
        //红色
        tmp.r = 1.0;
        tmp.g = 0.0;
        tmp.b = 0.0;
        const auto &color = tmp;

        const auto &center = tmp_obj->center;
        const auto &size = tmp_obj->size;
        auto& tmp_marker = params_.marker_list[i];

        tmp_marker.color.r = 1.0;
        tmp_marker.color.g = 1.0;
        tmp_marker.color.b = 0.0;
        Eigen::Vector3d pos = center;
        pos.z() += size.z() * 0.5f + 1.5f;
        pos.y() -=0.8f;
        std::string text_label;
        if(tmp_obj->type == ObjectType::UNKNOW){
            text_label  = myMap.at(tmp_obj->object_state.noise_state);
            text_label += "-";
            for(auto noise:tmp_obj->noise_history){
                text_label += myMap.at(noise);
            }

        }else{
            text_label = "AI";
        }
        drawText(pos, text_label, tmp_marker, 1.);
    }

    return params_.marker_list;
}


}
