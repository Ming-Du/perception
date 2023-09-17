#pragma once

#include "lable_local_info_marker_pub.h"
#include "lable_global_info_marker_pub.h"
#include "polygon_marker_pub.h"


namespace perception {
namespace radar {


void RadarFusionDisplay(const perception::RadarObjects& radar_fusion,
                        ros::Publisher& publisher,
                        Params& param,
                        localization::Localization local_current);

}  // namespace radar
}  // namespace perception 

