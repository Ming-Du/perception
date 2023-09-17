#ifndef RS_DEBUG_SHOW_H
#define RS_DEBUG_SHOW_H

#include "common/include/basic_type/object.h"
#include "common/include/msg/lidar_frame_msg.h"
// #include "common/target_kalman.h"
// #include "common/track_object.h"
#include "common/include/config/rs_yamlReader.h"
#include "common/include/config/rs_yaml_pro.h"
#include <vector>

namespace robosense {

class DebugShow {
public:
    using Ptr = std::shared_ptr<DebugShow>;
    static Ptr debug_show_ptr_;

    DebugShow() {
    }

    void init(const Rs_YAMLReader& configParse);
    void showObjData(std::vector<Object::Ptr> &obj_vec, std::string info, std::string debug_info);
    std::vector<int> showVehicleAxisData(const LidarFrameMsg::Ptr &msg_ptr, std::string info, std::string debug_info);
    // std::vector<int> showVehicleAxisData(std::vector<TrackObject::Ptr> &track_object_list, const LidarFrameMsg::Ptr &msg_ptr, std::string info, std::string debug_info);
    // void showTrackList(std::vector<TargetKalman::Ptr> &tracker_list_, std::vector<int> &trk_ids);
    // void showTrackObjValid(std::pair<TrackObject::Ptr, int> &track_obj_valid, TrackObject::Ptr &current, TrackObject::Ptr &cur_buf);
    void showObject(Object::Ptr &obj, std::string debug_info);
    // void showTrackObject(TrackObject::Ptr &obj, std::string debug_info);
    // void showTargetBuffer(boost::circular_buffer<TrackObject::Ptr> &target_buffer_, Object::Ptr &obj, std::string debug_info);
    debugInfoParam params;
};
} // namespace robosense

#endif // RS_DEBUG_SHOW_H