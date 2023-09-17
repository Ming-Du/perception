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
#include "pubs/marker_pubs/polygon_marker_pub.h"

namespace robosense{

std::vector<ROS_VISUALIZATION_MARKER> &PolygonMarkerPub::display(const LidarFrameMsg::Ptr &msg_ptr){

    auto& tmp_marker = params_.marker_list[0];
    const auto& obj_vec = msg_ptr->objects;
    tmp_marker.ns = "polygon";
    tmp_marker.id = 0;
    tmp_marker.header.frame_id = options_.frame_id;
    tmp_marker.type = ROS_VISUALIZATION_MARKER::LINE_LIST;
    tmp_marker.action = ROS_VISUALIZATION_MARKER::ADD;
    tmp_marker.scale.x = 0.05;
    tmp_marker.scale.y = 0.05;
    tmp_marker.scale.z = 0.05;
    tmp_marker.color.r = 0.7f;
    tmp_marker.color.g = 0.5f;
    tmp_marker.color.b = 0.f;
    tmp_marker.color.a = 1.f;

    tf::Quaternion quat = tf::createQuaternionFromRPY(0., 0., 0);
    tf::quaternionTFToMsg(quat, tmp_marker.pose.orientation);

    int polygon_size = 0;
    for (size_t i = 0; i < obj_vec.size(); ++i) {
        polygon_size += obj_vec[i]->polygons.size();
    }
    tmp_marker.points.reserve(polygon_size * 6);
    tmp_marker.colors.reserve(polygon_size * 6);
    tmp_marker.points.clear();

    for (size_t i = 0; i < obj_vec.size(); ++i) {
        const auto& obj = obj_vec[i];

        std::vector<ROS_GEOMETRY_POINT> cub_points;

        const auto &polygon = obj->polygons;
        size_t p_size = polygon.size();
        double p_low = obj->center[2] - obj->size[2] * 0.5f;
        double p_high = obj->center[2] + obj->size[2] * 0.5f;

        for (size_t i = 0; i < p_size; ++i) {
            ROS_GEOMETRY_POINT pts;
            pts.x = polygon[i].x();
            pts.y = polygon[i].y();
            pts.z = p_low;
            cub_points.emplace_back(pts);
        }

        for (size_t i = 0; i < p_size; ++i) {
            ROS_GEOMETRY_POINT pts;
            pts.x = polygon[i].x();
            pts.y = polygon[i].y();
            pts.z = p_high;
            cub_points.emplace_back(pts);
        }

        for (size_t i = 0; i < p_size; ++i) {
            size_t next = i + 1;
            next = next < p_size ? next : 0;

            tmp_marker.points.emplace_back(cub_points[i]);
            tmp_marker.points.emplace_back(cub_points[next]);
        }

        for (size_t i = 0; i < p_size; ++i) {
            size_t next = i + 1;
            next = next < p_size ? next : 0;

            tmp_marker.points.emplace_back(cub_points[i + p_size]);
            tmp_marker.points.emplace_back(cub_points[next + p_size]);
        }

        for (size_t i = 0; i < p_size; ++i) {
            tmp_marker.points.emplace_back(cub_points[i]);
            tmp_marker.points.emplace_back(cub_points[i + p_size]);
        }
    }


    return params_.marker_list;
}


std::vector<ROS_VISUALIZATION_MARKER> &PolygonMarkerPub::display_ori(const LidarFrameMsg::Ptr &msg_ptr){

    auto& tmp_marker = params_.marker_list[0];
    const auto& obj_vec = msg_ptr->objects_rule_debug;
    tmp_marker.ns = "polygon_ori";
    tmp_marker.id = 0;
    tmp_marker.header.frame_id = options_.frame_id;
    tmp_marker.type = ROS_VISUALIZATION_MARKER::LINE_LIST;
    tmp_marker.action = ROS_VISUALIZATION_MARKER::ADD;
    tmp_marker.scale.x = 0.05;
    tmp_marker.scale.y = 0.05;
    tmp_marker.scale.z = 0.05;
    tmp_marker.color.r = 1.f;
    tmp_marker.color.g = 0.3f;
    tmp_marker.color.b = 1.f;
    tmp_marker.color.a = 1.f;

    tf::Quaternion quat = tf::createQuaternionFromRPY(0., 0., 0);
    tf::quaternionTFToMsg(quat, tmp_marker.pose.orientation);

    int polygon_size = 0;
    for (size_t i = 0; i < obj_vec.size(); ++i) {
        polygon_size += obj_vec[i]->polygons.size();
    }
    tmp_marker.points.reserve(polygon_size * 6);
    tmp_marker.colors.reserve(polygon_size * 6);
    tmp_marker.points.clear();

    for (size_t i = 0; i < obj_vec.size(); ++i) {
        const auto& obj = obj_vec[i];

        std::vector<ROS_GEOMETRY_POINT> cub_points;

        const auto &polygon = obj->polygons;
        size_t p_size = polygon.size();
        double p_low = obj->center[2] - obj->size[2] * 0.5f;
        double p_high = obj->center[2] + obj->size[2] * 0.5f;

        for (size_t i = 0; i < p_size; ++i) {
            ROS_GEOMETRY_POINT pts;
            pts.x = polygon[i].x();
            pts.y = polygon[i].y();
            pts.z = p_low;
            cub_points.emplace_back(pts);
        }

        for (size_t i = 0; i < p_size; ++i) {
            ROS_GEOMETRY_POINT pts;
            pts.x = polygon[i].x();
            pts.y = polygon[i].y();
            pts.z = p_high;
            cub_points.emplace_back(pts);
        }

        for (size_t i = 0; i < p_size; ++i) {
            size_t next = i + 1;
            next = next < p_size ? next : 0;

            tmp_marker.points.emplace_back(cub_points[i]);
            tmp_marker.points.emplace_back(cub_points[next]);
        }

        for (size_t i = 0; i < p_size; ++i) {
            size_t next = i + 1;
            next = next < p_size ? next : 0;

            tmp_marker.points.emplace_back(cub_points[i + p_size]);
            tmp_marker.points.emplace_back(cub_points[next + p_size]);
        }

        for (size_t i = 0; i < p_size; ++i) {
            tmp_marker.points.emplace_back(cub_points[i]);
            tmp_marker.points.emplace_back(cub_points[i + p_size]);
        }
    }


    return params_.marker_list;
}
std::vector<ROS_VISUALIZATION_MARKER> &PolygonMarkerPub::display_ori_noise(const LidarFrameMsg::Ptr &msg_ptr){

    auto& tmp_marker = params_.marker_list[0];
    const auto& obj_vec = msg_ptr->objects;
    tmp_marker.ns = "polygon_ori_noise";
    tmp_marker.id = 0;
    tmp_marker.header.frame_id = options_.frame_id;
    tmp_marker.type = ROS_VISUALIZATION_MARKER::LINE_LIST;
    tmp_marker.action = ROS_VISUALIZATION_MARKER::ADD;
    tmp_marker.scale.x = 0.05;
    tmp_marker.scale.y = 0.05;
    tmp_marker.scale.z = 0.05;
    tmp_marker.color.r = 1.f;
    tmp_marker.color.g = 0.f;
    tmp_marker.color.b = 0.f;
    tmp_marker.color.a = 1.f;

    tf::Quaternion quat = tf::createQuaternionFromRPY(0., 0., 0);
    tf::quaternionTFToMsg(quat, tmp_marker.pose.orientation);

    int polygon_size = 0;
    for (size_t i = 0; i < obj_vec.size(); ++i) {
        if (obj_vec[i]->exist_confidence < 0) {
            continue;
        }
        polygon_size += obj_vec[i]->polygons.size();
    }
    tmp_marker.points.reserve(polygon_size * 6);
    tmp_marker.colors.reserve(polygon_size * 6);
    tmp_marker.points.clear();

    for (size_t i = 0; i < obj_vec.size(); ++i) {
        if (obj_vec[i]->exist_confidence < 0) {
            continue;
        }
        const auto& obj = obj_vec[i];
        // if(obj->object_state.noise_state==NoiseState::NOISE_OBJECT){
        if(obj->noise_state_obj!=NoiseState::NOISE_NOISE){
            continue;
        }
        std::vector<ROS_GEOMETRY_POINT> cub_points;

        const auto &polygon = obj->polygons;
        size_t p_size = polygon.size();
        double p_low = obj->center[2] - obj->size[2] * 0.5f;
        double p_high = obj->center[2] + obj->size[2] * 0.5f;

        for (size_t i = 0; i < p_size; ++i) {
            ROS_GEOMETRY_POINT pts;
            pts.x = polygon[i].x();
            pts.y = polygon[i].y();
            pts.z = p_low;
            cub_points.emplace_back(pts);
        }

        for (size_t i = 0; i < p_size; ++i) {
            ROS_GEOMETRY_POINT pts;
            pts.x = polygon[i].x();
            pts.y = polygon[i].y();
            pts.z = p_high+0.2;
            cub_points.emplace_back(pts);
        }

        // for (size_t i = 0; i < p_size; ++i) {
        //     size_t next = i + 1;
        //     next = next < p_size ? next : 0;

        //     tmp_marker.points.emplace_back(cub_points[i]);
        //     tmp_marker.points.emplace_back(cub_points[next]);
        // }

        for (size_t i = 0; i < p_size; ++i) {
            size_t next = i + 1;
            next = next < p_size ? next : 0;

            tmp_marker.points.emplace_back(cub_points[i + p_size]);
            tmp_marker.points.emplace_back(cub_points[next + p_size]);
        }

        // for (size_t i = 0; i < p_size; ++i) {
        //     tmp_marker.points.emplace_back(cub_points[i]);
        //     tmp_marker.points.emplace_back(cub_points[i + p_size]);
        // }
    }


    return params_.marker_list;
}


std::vector<ROS_VISUALIZATION_MARKER> &PolygonMarkerPub::display_ori_object(const LidarFrameMsg::Ptr &msg_ptr){

    auto& tmp_marker = params_.marker_list[0];
    const auto& obj_vec = msg_ptr->objects;
    tmp_marker.ns = "polygon_ori_object";
    tmp_marker.id = 0;
    tmp_marker.header.frame_id = options_.frame_id;
    tmp_marker.type = ROS_VISUALIZATION_MARKER::LINE_LIST;
    tmp_marker.action = ROS_VISUALIZATION_MARKER::ADD;
    tmp_marker.scale.x = 0.05;
    tmp_marker.scale.y = 0.05;
    tmp_marker.scale.z = 0.05;
    tmp_marker.color.r = 0.f;
    tmp_marker.color.g = 1.f;
    tmp_marker.color.b = 0.f;
    tmp_marker.color.a = 1.f;

    tf::Quaternion quat = tf::createQuaternionFromRPY(0., 0., 0);
    tf::quaternionTFToMsg(quat, tmp_marker.pose.orientation);

    int polygon_size = 0;
    for (size_t i = 0; i < obj_vec.size(); ++i) {
        if (obj_vec[i]->exist_confidence < 0) {
            continue;
        }
        polygon_size += obj_vec[i]->polygons.size();
    }
    tmp_marker.points.reserve(polygon_size * 6);
    tmp_marker.colors.reserve(polygon_size * 6);
    tmp_marker.points.clear();

    for (size_t i = 0; i < obj_vec.size(); ++i) {
        if (obj_vec[i]->exist_confidence < 0) {
            continue;
        }
        const auto& obj = obj_vec[i];
        // if(obj->object_state.noise_state==NoiseState::NOISE_NOISE){
        if(obj->noise_state_obj!=NoiseState::NOISE_OBJECT){
            continue;
        }
        std::vector<ROS_GEOMETRY_POINT> cub_points;

        const auto &polygon = obj->polygons;
        size_t p_size = polygon.size();
        double p_low = obj->center[2] - obj->size[2] * 0.5f;
        double p_high = obj->center[2] + obj->size[2] * 0.5f;

        for (size_t i = 0; i < p_size; ++i) {
            ROS_GEOMETRY_POINT pts;
            pts.x = polygon[i].x();
            pts.y = polygon[i].y();
            pts.z = p_low;
            cub_points.emplace_back(pts);
        }

        for (size_t i = 0; i < p_size; ++i) {
            ROS_GEOMETRY_POINT pts;
            pts.x = polygon[i].x();
            pts.y = polygon[i].y();
            pts.z = p_high+0.2;
            cub_points.emplace_back(pts);
        }

        // for (size_t i = 0; i < p_size; ++i) {
        //     size_t next = i + 1;
        //     next = next < p_size ? next : 0;

        //     tmp_marker.points.emplace_back(cub_points[i]);
        //     tmp_marker.points.emplace_back(cub_points[next]);
        // }

        for (size_t i = 0; i < p_size; ++i) {
            size_t next = i + 1;
            next = next < p_size ? next : 0;

            tmp_marker.points.emplace_back(cub_points[i + p_size]);
            tmp_marker.points.emplace_back(cub_points[next + p_size]);
        }

        // for (size_t i = 0; i < p_size; ++i) {
        //     tmp_marker.points.emplace_back(cub_points[i]);
        //     tmp_marker.points.emplace_back(cub_points[i + p_size]);
        // }
    }


    return params_.marker_list;
}


}
