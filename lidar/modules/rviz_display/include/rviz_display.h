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
#ifndef RS_RVIZ_DISPLAY_RVIZ_DISPLAY_H_
#define RS_RVIZ_DISPLAY_RVIZ_DISPLAY_H_

#include "common/cloud_pub.h"
#include "common/include/config/rs_yamlReader.h"
#include "common/include/config/rs_yaml_pro.h"
#include "pubs/marker_pubs/arrow_marker_pub.h"
#include "pubs/marker_pubs/box_marker_pub.h"
#include "pubs/marker_pubs/cube_maker_pub.h"
#include "pubs/marker_pubs/denoise_maker_pub.h"
#include "pubs/marker_pubs/lable_info_marker_pub.h"
#include "pubs/marker_pubs/polygon_marker_pub.h"
#include "pubs/marker_pubs/track_info_marker_pub.h"
#include "pubs/marker_pubs/refine_maker_pub.h"
#include <common/include/msg/lidar_frame_msg.h>
#include <yaml-cpp/yaml.h>
namespace robosense {

class RvizDisplay {
public:
    using Ptr = std::shared_ptr<RvizDisplay>;

    RvizDisplay(const Rs_YAMLReader &configParse);

    void display(const LidarFrameMsg::Ptr &msg_ptr, const DisplayModule &display_module);

private:
    std::string name() { return "RvizDisplay"; }

    ros::NodeHandlePtr node_ptr;

    RvizParam params_;
    bool ground_enable = false;

    ros::Publisher pub_map_, pub_road_flag_, pub_perception_, pub_roadmap_, pub_binmap_;
    ros::Publisher pub_denoisemap_object_, pub_denoisemap_noise_;
    ROS_VISUALIZATION_MARKERARRAY::Ptr marker_array_ptr_;

    BoxMarkerPub::Ptr box_pub_;
    CubeMarkerPub::Ptr cube_pub_;
    DenoiseMarkerPub::Ptr denoise_pub_;
    ArrowMarkerPub::Ptr arrow_pub_;
    LabelInfosMarkerPub::Ptr label_pub_;
    PolygonMarkerPub::Ptr polygon_pub_;
    TrackInfoPubMarker::Ptr track_pub_;
    RefineMarkerPub::Ptr refine_pub_;
    CloudPub::Ptr cloud_pub_ptr_;
};

} // namespace robosense

#endif // RS_RVIZ_DISPLAY_RVIZ_DISPLAY_H_
