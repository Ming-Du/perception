#pragma once

#include "common/proto/object.pb.h"
#include <common/proto/localization.pb.h>
#include "base_marker_pub.h"


namespace perception {
namespace radar {


void trackMarkerLocalPub(const perception::RadarObjects& radar_fusion,
                         Params& param,
                         localization::Localization local_current);


}  // namespace radar
}  // namespace perception 