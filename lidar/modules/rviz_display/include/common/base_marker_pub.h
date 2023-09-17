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
#ifndef RS_RVIZ_DISPLAY_COMMON_BASE_MARKER_PUB_H_
#define RS_RVIZ_DISPLAY_COMMON_BASE_MARKER_PUB_H_

#include "common/base_pub.h"
#include "common/include/msg/lidar_frame_msg.h"
#include "common/include/basic_type/rotate_box.h"
namespace robosense{

class MarkerPubOptions:public BasePubOptions{
public:

};

class BaseMarkerPub{
public:
    using Ptr = std::shared_ptr<BaseMarkerPub>;

    virtual ~BaseMarkerPub() = default;

    virtual void init(const MarkerPubOptions& options) = 0;

    virtual std::vector<ROS_VISUALIZATION_MARKER>& display(const LidarFrameMsg::Ptr& msg_ptr) = 0;

    virtual std::string name() = 0;

protected:
    ros::Publisher pub_;
    MarkerPubOptions options_;

    struct Params{
        std::vector<ROS_VISUALIZATION_MARKER> marker_list;
        std::map<std::string,ROS_VISUALIZATION_MARKER::_color_type> color_kind_map;


        std::map<ObjectType,std::string> class_color_map = {
        {ObjectType::UNKNOW,_Lavender},
        {ObjectType::PED,_Yellow},
        {ObjectType::BIC,_Green},
        {ObjectType::CAR,_CyanBlue},
        {ObjectType::TRUCK,_CyanBlue},
        {ObjectType::BUS,_CyanBlue},
        };

        ROS_VISUALIZATION_MARKER::_color_type default_color_type;
        ROS_VISUALIZATION_MARKER::_scale_type default_scale_type;
        size_t max_obj_size = 1;

        Params(){
            marker_list.resize(1);
            color_kind_map.clear();
            ROS_VISUALIZATION_MARKER::_color_type tmp;
            //淡紫色
            tmp.r = 0.6;
            tmp.g = 0.5;
            tmp.b = 0.6;
            color_kind_map[_Lavender] = tmp;
            //黄色
            tmp.r = 1.;
            tmp.g = 1.;
            tmp.b = 0.;
            color_kind_map[_Yellow] = tmp;
            //青色
            tmp.r = 0.;
            tmp.g = 1.;
            tmp.b = 1.;
            color_kind_map[_CyanBlue] = tmp;
            //红色
            tmp.r = 0.7;
            tmp.g = 0.;
            tmp.b = 0.;
            color_kind_map[_Red] = tmp;
            //蓝色
            tmp.r = 0.;
            tmp.g = 0.;
            tmp.b = 1.;
            color_kind_map[_Blue] = tmp;
            //绿色
            tmp.r = 0.;
            tmp.g = 1.;
            tmp.b = 0.;
            color_kind_map[_Green] = tmp;

            default_color_type.r = 0;
            default_color_type.g = 0;
            default_color_type.b = 0;
            default_color_type.a = 0;

            default_scale_type.x = 1.0;
            default_scale_type.y = 1.0;
            default_scale_type.z = 1.0;
        }
    }params_;

    ROS_VISUALIZATION_MARKER::_color_type getClassColor(const Object::Ptr &obj){
        if (params_.class_color_map.find(obj->type) == params_.class_color_map.end()){
            return params_.color_kind_map[_Lavender];
        }else{
            return params_.color_kind_map[params_.class_color_map.at(obj->type)];
        }
    }
    void drawText(const Eigen::Vector3d &pos, const std::string &info,
                                 ROS_VISUALIZATION_MARKER &marker, double alpha) {
        marker.type = ROS_VISUALIZATION_MARKER::TEXT_VIEW_FACING;
        marker.action = ROS_VISUALIZATION_MARKER::ADD;
        marker.pose.position.x = pos.x();
        marker.pose.position.y = pos.y();
        marker.pose.position.z = pos.z();
        tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, 0);
        tf::quaternionTFToMsg(quat, marker.pose.orientation);
        marker.color.a = alpha;
        marker.text = info;
    }
    template<typename T>
    inline std::string num2str(T num, int precision) {
        std::stringstream ss;
        ss.setf(std::ios::fixed, std::ios::floatfield);
        ss.precision(precision);
        std::string st;
        ss << num;
        ss >> st;

        return st;
    }
};


}  // namespace robosense

#endif  // RS_RVIZ_DISPLAY_COMMON_BASE_MARKER_PUB_H_
