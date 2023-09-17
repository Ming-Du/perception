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
#include "pubs/marker_pubs/box_marker_pub.h"

namespace robosense {

void showObjData(const std::vector<Object::Ptr> &obj_vec, std::string info,std::string debug_info,
    float xmin,float xmax,float ymin,float ymax){
    for(auto obj : obj_vec){
        if(obj->center(0)>xmin && obj->center(0)<xmax
          && obj->center(1)>ymin && obj->center(1)<ymax){
            // debug show
            // std::cout<<debug_info<<info<<obj->tracker_id
            // <<",x:"<<obj->center(0)<<",y:"<<obj->center(1)
            // <<",xSize:"<<obj->size(0)<<",ySize:"<<obj->size(1)
            // <<",xDir:"<<obj->direction(0)<<",yDir:"<<obj->direction(1)
            // <<std::endl;
        }
    }
}
std::vector<ROS_VISUALIZATION_MARKER> &BoxMarkerPub::display(const LidarFrameMsg::Ptr &msg_ptr) {

    const auto &obj_vec = msg_ptr->objects;

    // showObjData(obj_vec, "rviz box obj:",HDEBUG_G,25,30,-3,2);

    if (params_.max_obj_size < obj_vec.size()) {
        params_.max_obj_size = obj_vec.size();
    }

    auto &box_marker = params_.marker_list[0];
    box_marker.color = params_.default_color_type;
    box_marker.scale.x = 0.1;
    box_marker.scale.y = 0.1;
    box_marker.scale.z = 0.1;
    box_marker.ns = "box";
    box_marker.id = 0;
    box_marker.header.frame_id = options_.frame_id;
    box_marker.type = ROS_VISUALIZATION_MARKER::LINE_LIST;
    box_marker.action = ROS_VISUALIZATION_MARKER::ADD;

    tf::Quaternion quat = tf::createQuaternionFromRPY(0., 0., 0);
    tf::quaternionTFToMsg(quat, box_marker.pose.orientation);

    if (box_marker.points.size() < (params_.max_obj_size * 24)) {
        box_marker.points.resize(params_.max_obj_size * 24);
        box_marker.colors.resize(params_.max_obj_size * 24);
    }

    for (size_t i = 0; i < box_marker.points.size(); ++i) {
        box_marker.colors[i].a = 0.;
    }

    for (size_t i = 0; i < obj_vec.size(); ++i) {
        const auto &tmp_obj = obj_vec[i];

        const auto &color = getClassColor(tmp_obj);
        bool is_attention = (tmp_obj->attention_type == AttentionType::ATTENTION);
        double tmp_alpha;
        if (is_attention) {
            tmp_alpha = 1.;
        } else {
            tmp_alpha = 0.7;
        }

        if (ObjectType::CAR == tmp_obj->type) {
            box_marker.scale.x = 0.2;
            box_marker.scale.y = 0.2;
            box_marker.scale.z = 0.2;
        }

        const auto &center = tmp_obj->center;
        const auto &size = tmp_obj->size;
        const auto &direction = tmp_obj->direction;
        RotateBox box(center, size, direction);

        int idx = i * 24;
        std::vector<ROS_GEOMETRY_POINT> cub_points;

        std::vector<Eigen::Vector3d> corners;
        box.corners(corners);

        for (int i = 0; i < 8; ++i) {
            ROS_GEOMETRY_POINT pts;
            pts.x = corners[i].x();
            pts.y = corners[i].y();
            pts.z = corners[i].z();
            cub_points.push_back(pts);
        }

        for (int j = 0; j < 24; ++j) {
            box_marker.colors[idx + j].a = tmp_alpha;
            box_marker.colors[idx + j].r = color.r;
            box_marker.colors[idx + j].g = color.g;
            box_marker.colors[idx + j].b = color.b;
        }
        box_marker.points[idx + 0] = cub_points[0];
        box_marker.points[idx + 1] = cub_points[1];
        box_marker.points[idx + 2] = cub_points[1];
        box_marker.points[idx + 3] = cub_points[2];
        box_marker.points[idx + 4] = cub_points[2];
        box_marker.points[idx + 5] = cub_points[3];
        box_marker.points[idx + 6] = cub_points[3];
        box_marker.points[idx + 7] = cub_points[0];

        box_marker.points[idx + 8] = cub_points[4];
        box_marker.points[idx + 9] = cub_points[5];
        box_marker.points[idx + 10] = cub_points[5];
        box_marker.points[idx + 11] = cub_points[6];
        box_marker.points[idx + 12] = cub_points[6];
        box_marker.points[idx + 13] = cub_points[7];
        box_marker.points[idx + 14] = cub_points[7];
        box_marker.points[idx + 15] = cub_points[4];

        box_marker.points[idx + 16] = cub_points[0];
        box_marker.points[idx + 17] = cub_points[4];
        box_marker.points[idx + 18] = cub_points[1];
        box_marker.points[idx + 19] = cub_points[5];
        box_marker.points[idx + 20] = cub_points[2];
        box_marker.points[idx + 21] = cub_points[6];
        box_marker.points[idx + 22] = cub_points[3];
        box_marker.points[idx + 23] = cub_points[7];
    }

    return params_.marker_list;
}

std::vector<ROS_VISUALIZATION_MARKER> &BoxMarkerPub::display_rule(const LidarFrameMsg::Ptr &msg_ptr) {

    const auto &obj_vec = msg_ptr->objects_rule_debug;

    // showObjData(obj_vec, "rviz box obj:",HDEBUG_G,25,30,-3,2);

    if (params_.max_obj_size < obj_vec.size()) {
        params_.max_obj_size = obj_vec.size();
    }

    auto &box_marker = params_.marker_list[0];
    box_marker.color.r = 85;
    box_marker.color.g = 180;
    box_marker.color.b = 70;

    box_marker.scale.x = 0.1;
    box_marker.scale.y = 0.1;
    box_marker.scale.z = 0.1;
    box_marker.ns = "box_rule";
    box_marker.id = 0;
    box_marker.header.frame_id = options_.frame_id;
    box_marker.type = ROS_VISUALIZATION_MARKER::LINE_LIST;
    box_marker.action = ROS_VISUALIZATION_MARKER::ADD;

    tf::Quaternion quat = tf::createQuaternionFromRPY(0., 0., 0);
    tf::quaternionTFToMsg(quat, box_marker.pose.orientation);

    if (box_marker.points.size() < (params_.max_obj_size * 24)) {
        box_marker.points.resize(params_.max_obj_size * 24);
        box_marker.colors.resize(params_.max_obj_size * 24);
    }

    for (size_t i = 0; i < box_marker.points.size(); ++i) {
        box_marker.colors[i].a = 0.;
    }

    for (size_t i = 0; i < obj_vec.size(); ++i) {
        const auto &tmp_obj = obj_vec[i];

        //const auto &color = getClassColor(tmp_obj);
        bool is_attention = (tmp_obj->attention_type == AttentionType::ATTENTION);
        double tmp_alpha;
        if (is_attention) {
            tmp_alpha = 1.;
        } else {
            tmp_alpha = 0.7;
        }

        if (ObjectType::CAR == tmp_obj->type) {
            box_marker.scale.x = 0.2;
            box_marker.scale.y = 0.2;
            box_marker.scale.z = 0.2;
        }

        const auto &center = tmp_obj->center;
        const auto &size = tmp_obj->size;
        const auto &direction = tmp_obj->direction;
        RotateBox box(center, size, direction);

        int idx = i * 24;
        std::vector<ROS_GEOMETRY_POINT> cub_points;

        std::vector<Eigen::Vector3d> corners;
        box.corners(corners);

        for (int i = 0; i < 8; ++i) {
            ROS_GEOMETRY_POINT pts;
            pts.x = corners[i].x();
            pts.y = corners[i].y();
            pts.z = corners[i].z();
            cub_points.push_back(pts);
        }

        for (int j = 0; j < 24; ++j) {
            box_marker.colors[idx + j].a = tmp_alpha;
            box_marker.colors[idx + j].r = 0.3;
            box_marker.colors[idx + j].g = 0.8;
            box_marker.colors[idx + j].b = 0.2;
        }
        box_marker.points[idx + 0] = cub_points[0];
        box_marker.points[idx + 1] = cub_points[1];
        box_marker.points[idx + 2] = cub_points[1];
        box_marker.points[idx + 3] = cub_points[2];
        box_marker.points[idx + 4] = cub_points[2];
        box_marker.points[idx + 5] = cub_points[3];
        box_marker.points[idx + 6] = cub_points[3];
        box_marker.points[idx + 7] = cub_points[0];

        box_marker.points[idx + 8] = cub_points[4];
        box_marker.points[idx + 9] = cub_points[5];
        box_marker.points[idx + 10] = cub_points[5];
        box_marker.points[idx + 11] = cub_points[6];
        box_marker.points[idx + 12] = cub_points[6];
        box_marker.points[idx + 13] = cub_points[7];
        box_marker.points[idx + 14] = cub_points[7];
        box_marker.points[idx + 15] = cub_points[4];

        box_marker.points[idx + 16] = cub_points[0];
        box_marker.points[idx + 17] = cub_points[4];
        box_marker.points[idx + 18] = cub_points[1];
        box_marker.points[idx + 19] = cub_points[5];
        box_marker.points[idx + 20] = cub_points[2];
        box_marker.points[idx + 21] = cub_points[6];
        box_marker.points[idx + 22] = cub_points[3];
        box_marker.points[idx + 23] = cub_points[7];
    }

    return params_.marker_list;
}

std::vector<ROS_VISUALIZATION_MARKER> &BoxMarkerPub::display_ai(const LidarFrameMsg::Ptr &msg_ptr) {

    const auto &obj_vec = msg_ptr->objects_ai_debug;

    // showObjData(obj_vec, "rviz box obj:",HDEBUG_G,25,30,-3,2);

    if (params_.max_obj_size < obj_vec.size()) {
        params_.max_obj_size = obj_vec.size();
    }

    auto &box_marker = params_.marker_list[0];
    box_marker.color.r = 85;
    box_marker.color.g = 180;
    box_marker.color.b = 70;

    box_marker.scale.x = 0.1;
    box_marker.scale.y = 0.1;
    box_marker.scale.z = 0.1;
    box_marker.ns = "box_ai";
    box_marker.id = 0;
    box_marker.header.frame_id = options_.frame_id;
    box_marker.type = ROS_VISUALIZATION_MARKER::LINE_LIST;
    box_marker.action = ROS_VISUALIZATION_MARKER::ADD;

    tf::Quaternion quat = tf::createQuaternionFromRPY(0., 0., 0);
    tf::quaternionTFToMsg(quat, box_marker.pose.orientation);

    if (box_marker.points.size() < (params_.max_obj_size * 24)) {
        box_marker.points.resize(params_.max_obj_size * 24);
        box_marker.colors.resize(params_.max_obj_size * 24);
    }

    for (size_t i = 0; i < box_marker.points.size(); ++i) {
        box_marker.colors[i].a = 0.;
    }

    for (size_t i = 0; i < obj_vec.size(); ++i) {
        const auto &tmp_obj = obj_vec[i];

        //const auto &color = getClassColor(tmp_obj);
        bool is_attention = (tmp_obj->attention_type == AttentionType::ATTENTION);
        double tmp_alpha;
        if (is_attention) {
            tmp_alpha = 1.;
        } else {
            tmp_alpha = 0.7;
        }

        // if (ObjectType::CAR == tmp_obj->type) {
        //     box_marker.scale.x = 0.2;
        //     box_marker.scale.y = 0.2;
        //     box_marker.scale.z = 0.2;
        // }

        const auto &center = tmp_obj->center;
        const auto &size = tmp_obj->size;
        const auto &direction = tmp_obj->direction;
        RotateBox box(center, size, direction);

        int idx = i * 24;
        std::vector<ROS_GEOMETRY_POINT> cub_points;

        std::vector<Eigen::Vector3d> corners;
        box.corners(corners);

        for (int i = 0; i < 8; ++i) {
            ROS_GEOMETRY_POINT pts;
            pts.x = corners[i].x();
            pts.y = corners[i].y();
            pts.z = corners[i].z();
            cub_points.push_back(pts);
        }

        for (int j = 0; j < 24; ++j) {
            box_marker.colors[idx + j].a = tmp_alpha;
            box_marker.colors[idx + j].r = 0.8;
            box_marker.colors[idx + j].g = 0.3;
            box_marker.colors[idx + j].b = 0.2;
        }
        box_marker.points[idx + 0] = cub_points[0];
        box_marker.points[idx + 1] = cub_points[1];
        box_marker.points[idx + 2] = cub_points[1];
        box_marker.points[idx + 3] = cub_points[2];
        box_marker.points[idx + 4] = cub_points[2];
        box_marker.points[idx + 5] = cub_points[3];
        box_marker.points[idx + 6] = cub_points[3];
        box_marker.points[idx + 7] = cub_points[0];

        box_marker.points[idx + 8] = cub_points[4];
        box_marker.points[idx + 9] = cub_points[5];
        box_marker.points[idx + 10] = cub_points[5];
        box_marker.points[idx + 11] = cub_points[6];
        box_marker.points[idx + 12] = cub_points[6];
        box_marker.points[idx + 13] = cub_points[7];
        box_marker.points[idx + 14] = cub_points[7];
        box_marker.points[idx + 15] = cub_points[4];

        box_marker.points[idx + 16] = cub_points[0];
        box_marker.points[idx + 17] = cub_points[4];
        box_marker.points[idx + 18] = cub_points[1];
        box_marker.points[idx + 19] = cub_points[5];
        box_marker.points[idx + 20] = cub_points[2];
        box_marker.points[idx + 21] = cub_points[6];
        box_marker.points[idx + 22] = cub_points[3];
        box_marker.points[idx + 23] = cub_points[7];
    }

    return params_.marker_list;
}

}
