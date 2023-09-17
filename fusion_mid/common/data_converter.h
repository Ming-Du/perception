#pragma once

#include <ros/ros.h>

#include "common/proto/localization.pb.h"
#include "common/proto/object.pb.h"
#include "common/proto/hadmap.pb.h"
#include <common/include/pb_common.h>
#include "perception/fusion_mid/common/object_type_convert.h"
#include "perception/fusion_mid/common/mid_struct.h"
#include "perception/fusion_mid/common/vec2.h"
#include "perception/fusion_mid/tracker/base/frame.h"
#include "perception/fusion_mid/common/define.h"
#include "perception/fusion_mid/common/coordinate_transformer.h"

namespace perception {
namespace mid_fusion {

enum class FpnetPred {
  center_x = 0,
  center_y = 1,
  center_z = 2,
  size_x = 3,
  size_y = 4,
  size_z = 5,
  heading = 6,
  confidence = 7,
  dim_size = 9,
};

// inline void ModelPred2Mogo();
void Mogo2Tracker(const perception::TrackedObject& det_obj, mid_fusion::ObjectPtr& tracker_obj);
void Tracker2Mogo(const mid_fusion::Frame& detected_frame,
                  const mid_fusion::FramePtr& tracked_frame,
                  perception::TrackedObjects& fpnet_tracked_result,
                  const localization::Localization& local_current);
void TrackerObj2Mogo(mid_fusion::ObjectPtr& object_ptr,
                     perception::TrackedObject* tracker_obj_ptr,
                     const localization::Localization& local_current);
void GenerateContour(perception::Object* obj_ptr);
void ConvertFpnetResultToObject(const std::vector<float>& pred_box3d_temp,
                                const PreprocessFpntInput& pre_res,
                                const localization::Localization& local_current,
                                const int index,
                                perception::TrackedObject& lidar_obj_temp);
void ConvertMapMsg2LaneData(hadmap::MapMsg map,
                            perception::mid_fusion::LaneMarkerDataRecord& lanemark,
                            const localization::Localization& local_current);
void HadmapMsgToLaneMarkerData(const hadmap::LaneBoundary* hadmap_laneboundary,
                               perception::mid_fusion::LaneMarkerData& marker_data,
                               const localization::Localization& local_current);

}  // namespace mid_fusion
}  // namespace perception
