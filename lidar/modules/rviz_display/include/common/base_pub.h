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
#ifndef RS_RVIZ_DISPLAY_COMMON_BASE_PUB_H_
#define RS_RVIZ_DISPLAY_COMMON_BASE_PUB_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

enum DisplayModule{
    INVALID,
    PREPROCESS,
    ROI_FILTER,
    GROUND_FILTER,
    DENOISE,
    SEGMENTOR,
    REFINER,
    TRACKER
};

namespace robosense{

using ROS_VISUALIZATION_MARKER = visualization_msgs::Marker;
using ROS_VISUALIZATION_MARKERARRAY = visualization_msgs::MarkerArray;
using ROS_GEOMETRY_POINT = geometry_msgs::Point;

static const char _Lavender[] = "Lavender";
static const char _Yellow[] = "Yellow";
static const char _CyanBlue[] = "CyanBlue";
static const char _Red[] = "Red";
static const char _Blue[] = "Blue";
static const char _Green[] = "Green";

/**
 * Base Pub Options
 */
class BasePubOptions{
public:
    std::string pre_fix = "";
    std::string frame_id = "base_link";
    ros::NodeHandlePtr node_ptr = nullptr;
};

}

#endif  // RS_RVIZ_DISPLAY_COMMON_BASE_PUB_H_
