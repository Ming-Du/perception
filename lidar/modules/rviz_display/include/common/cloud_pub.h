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
#ifndef RS_RVIZ_DISPLAY_COMMON_BASE_CLOUD_PUB_H_
#define RS_RVIZ_DISPLAY_COMMON_BASE_CLOUD_PUB_H_

#include "common/base_pub.h"
#include "common/include/msg/lidar_frame_msg.h"

namespace robosense{

class CloudPubOptions:public BasePubOptions{
public:

};

class CloudPub{
public:
    using Ptr = std::shared_ptr<CloudPub>;


    void init(const CloudPubOptions& options) {
        options_ = options;
        pub_origin = options_.node_ptr->advertise<pcl::PointCloud<pcl::PointXYZI> >(options_.pre_fix + "percept_cloud_rviz",1,true);
        pub_ground = options_.node_ptr->advertise<pcl::PointCloud<pcl::PointXYZI> >(options_.pre_fix + "percept_ground_rviz",1,true);
        pub_non_ground = options_.node_ptr->advertise<pcl::PointCloud<pcl::PointXYZI> >(options_.pre_fix + "percept_non_ground_rviz",1,true);
        pub_noise = options_.node_ptr->advertise<pcl::PointCloud<pcl::PointXYZI> >(options_.pre_fix + "percept_noise_rviz",1,true);
        pub_lshape = options_.node_ptr->advertise<pcl::PointCloud<pcl::PointXYZI> >(options_.pre_fix + "percept_Lshape",1,true);
        pub_outmap = options_.node_ptr->advertise<pcl::PointCloud<pcl::PointXYZI> >(options_.pre_fix + "percept_outmap_rviz",1,true);
        pub_map = options_.node_ptr->advertise<pcl::PointCloud<pcl::PointXYZI> >(options_.pre_fix + "percept_sementic_map_rviz",1,true);
    }

    void pubCloud(const LidarFrameMsg::Ptr& msg_ptr, const DisplayModule &display_module) ;

    std::string name();

protected:
    ros::Publisher pub_origin;
    ros::Publisher pub_ground;
    ros::Publisher pub_non_ground;
    ros::Publisher pub_noise;
    ros::Publisher pub_lshape;
    ros::Publisher pub_outmap;
    ros::Publisher pub_map;
    CloudPubOptions options_;
};


}  // namespace robosense

#endif  // RS_RVIZ_DISPLAY_COMMON_BASE_CLOUD_PUB_H_
