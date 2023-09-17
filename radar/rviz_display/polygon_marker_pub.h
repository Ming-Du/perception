#pragma once

#include "common/proto/object.pb.h"
#include <common/proto/localization.pb.h>
#include "base_marker_pub.h"
#include <tf/tf.h>


namespace perception {
namespace radar {


void trackMarkerPolygonPub(const perception::RadarObjects& radar_fusion,
                           Params& param,
                           localization::Localization local_current);


}  // namespace radar
}  // namespace perception 

