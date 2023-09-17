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
#include "pubs/marker_pubs/denoise_maker_pub.h"
#include "common/include/basic_type/rotate_box.h"

namespace robosense{

std::vector<ROS_VISUALIZATION_MARKER> &DenoiseMarkerPub::display(const LidarFrameMsg::Ptr &msg_ptr){
    const auto& obj_vec = msg_ptr->objects_vehicle;
    if (params_.max_obj_size < obj_vec.size()){
        params_.max_obj_size = obj_vec.size();
        params_.marker_list.resize(obj_vec.size());
    }
    for (size_t i = 0; i < params_.marker_list.size(); ++i) {
        auto& tmp_maker = params_.marker_list[i];
        tmp_maker.color = params_.default_color_type;
        tmp_maker.scale = params_.default_scale_type;
        tmp_maker.ns = "denoise";
        tmp_maker.id = i;
        tmp_maker.header.frame_id = options_.frame_id;
    }
    for (size_t i = 0; i < obj_vec.size(); ++i){
        const auto& tmp_obj = obj_vec[i];
        // std::cout<< "\033[32m trkid:"<<tmp_obj->tracker_id<<
        //          " \033[34m isnoise:"<<tmp_obj->is_noise<<"\033[0m"<<std::endl;
        ROS_VISUALIZATION_MARKER::_color_type color;
        if(tmp_obj->object_state.noise_state==NoiseState::NOISE_NOISE){
            color.r = 0.9;
            color.g = 0.;
            color.b = 0.;
        }else if(tmp_obj->object_state.noise_state==NoiseState::NOISE_SUSPECTED){
            color.r = 0.9;
            color.g = 0.9;
            color.b = 0.;
        }else{
            color.r = 0.;
            color.g = 0.9;
            color.b = 0.;
            // color = getClassColor(tmp_obj);
        }
        // const auto &color = getClassColor(tmp_obj);
        const auto &center = tmp_obj->center;
        const auto &size = tmp_obj->size;
        auto& tmp_marker = params_.marker_list[i];

        tmp_marker.color = color;
        drawCube(tmp_obj, tmp_marker, 0.5);
    }
    return params_.marker_list;
}
std::vector<ROS_VISUALIZATION_MARKER> &DenoiseMarkerPub::display_deleted(const LidarFrameMsg::Ptr &msg_ptr){
    const auto& obj_vec_denoise_del = msg_ptr->objects_denoise_del;
    size_t obj_denoise_del_size = obj_vec_denoise_del.size();
    if (params_.max_obj_size < obj_denoise_del_size){
        params_.max_obj_size = obj_denoise_del_size;
        params_.marker_list.resize(obj_denoise_del_size);
    }
    for (size_t i = 0; i < params_.marker_list.size(); ++i) {
        auto& tmp_maker = params_.marker_list[i];
        tmp_maker.color = params_.default_color_type;
        tmp_maker.scale = params_.default_scale_type;
        tmp_maker.ns = "denoise_del";
        tmp_maker.id = i;
        tmp_maker.header.frame_id = options_.frame_id;
    }
    for (size_t i = 0; i < obj_denoise_del_size; ++i){
        const auto& tmp_obj = obj_vec_denoise_del[i];
        // std::cout<< "\033[32m trkid:"<<tmp_obj->tracker_id<<
        //          " \033[34m isnoise:"<<tmp_obj->is_noise<<"\033[0m"<<std::endl;
        ROS_VISUALIZATION_MARKER::_color_type color;
        color.r = 0.77;
        color.g = 0.08;
        color.b = 1.0;
        // const auto &color = getClassColor(tmp_obj);
        const auto &center = tmp_obj->center;
        const auto &size = tmp_obj->size;
        auto& tmp_marker = params_.marker_list[i];

        tmp_marker.color = color;
        drawCube(tmp_obj, tmp_marker, 0.5);
    }

    return params_.marker_list;
}

void DenoiseMarkerPub::drawCube(const Object::Ptr &obj, ROS_VISUALIZATION_MARKER &marker, double alpha) {
    marker.type = ROS_VISUALIZATION_MARKER::CUBE;
    marker.action = ROS_VISUALIZATION_MARKER::ADD;

    const auto& center = obj->center;
    const auto& size = obj->size;
    const auto& direction = obj->direction;
    RotateBox box(center, size, direction);
    double box_size = box.volume();
    if (box_size > 0) {
        marker.color.a = alpha;

        marker.pose.position.x = box.center.x();
        marker.pose.position.y = box.center.y();
        marker.pose.position.z = box.center.z();

        marker.scale.x = box.size.x();
        marker.scale.y = box.size.y();
        marker.scale.z = box.size.z();

        tf::Quaternion quat = tf::createQuaternionFromYaw(box.angle);
        tf::quaternionTFToMsg(quat, marker.pose.orientation);
    } else {
        marker.color.a = 0;
    }
}




}
