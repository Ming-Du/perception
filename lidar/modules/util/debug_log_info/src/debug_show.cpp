#include "debug_show.h"

namespace robosense {

DebugShow::Ptr DebugShow::debug_show_ptr_(new DebugShow());

void DebugShow::init(const Rs_YAMLReader& configParse) {
    params = configParse.getValue<debugInfoParam>("debug_log_info");
}

void DebugShow::showObjData(std::vector<Object::Ptr> &obj_vec, std::string info, std::string debug_info) {
    for (auto obj : obj_vec) {
        if (obj->center(0) > params.xmin && obj->center(0) < params.xmax && obj->center(1) > params.ymin && obj->center(1) < params.ymax) {
            // debug show
            std::cout << debug_info << info << obj->tracker_id
                      << ",type: " << (obj->type == ObjectType::UNKNOW ? "RB" : "AI")
                      << ",type: " << (int)(obj->type) //kObjectType2NameMap.at(obj->type)
                      << ",x:" << obj->center(0) << ",y:" << obj->center(1)
                      << ",xSize:" << obj->size(0) << ",ySize:" << obj->size(1)
                      << ",xDir:" << obj->direction(0) << ",yDir:" << obj->direction(1)
                      << std::endl;
        }
    }
}

std::vector<int> DebugShow::showVehicleAxisData(const LidarFrameMsg::Ptr &msg_ptr, std::string info, std::string debug_info) {
    auto &obj_vec = msg_ptr->objects;
    Eigen::Matrix4d transform_mat;
    msg_ptr->transGlobal2VehicleMat(transform_mat);
    std::vector<int> trk_ids;
    for (auto moi : obj_vec) {
        Object::Ptr obj(new Object);
        { //center
            Eigen::Vector4d tt_e;
            auto &tmp = moi->center;
            tt_e << tmp(0), tmp(1), tmp(2), 1;
            tt_e = transform_mat * tt_e;
            obj->center(0) = tt_e.x();
            obj->center(1) = tt_e.y();
            obj->center(2) = tt_e.z();
        }
        { //direction
            Eigen::Vector4d tt_e;
            auto &tmp = moi->direction;
            tt_e << tmp.x(), tmp.y(), tmp.z(), 0;
            tt_e = transform_mat * tt_e;
            obj->direction(0) = tt_e.x();
            obj->direction(1) = tt_e.y();
            obj->direction(2) = tt_e.z();
        }
        //Keep the global parameters below
        obj->tracker_id = moi->tracker_id;
        obj->type = moi->type;
        obj->size(0) = moi->size(0);
        obj->size(1) = moi->size(1);
        obj->size(2) = moi->size(2);

        if (obj->center(0) > params.xmin && obj->center(0) < params.xmax && obj->center(1) > params.ymin && obj->center(1) < params.ymax) {
            trk_ids.push_back(obj->tracker_id);
            // debug show.
            std::cout << debug_info << info << obj->tracker_id
                      << ",type: " << (int)(obj->type) //kObjectType2NameMap.at(obj->type)
                      << ",x:" << obj->center(0) << ",y:" << obj->center(1)
                      << ",xSize:" << obj->size(0) << ",ySize:" << obj->size(1)
                      << ",xDir:" << obj->direction(0) << ",yDir:" << obj->direction(1)
                      << std::endl;
        }
    }
    return trk_ids;
}
// std::vector<int> DebugShow::showVehicleAxisData(std::vector<TrackObject::Ptr> &track_object_list, const LidarFrameMsg::Ptr &msg_ptr, std::string info, std::string debug_info) {
//     Eigen::Matrix4d transform_mat;
//     msg_ptr->transGlobal2VehicleMat(transform_mat);
//     std::vector<int> trk_ids;
//     for (auto moi : track_object_list) {
//         Object::Ptr obj(new Object);
//         { //center
//             Eigen::Vector4d tt_e;
//             auto &tmp = moi->center;
//             tt_e << tmp(0), tmp(1), tmp(2), 1;
//             tt_e = transform_mat * tt_e;
//             obj->center(0) = tt_e.x();
//             obj->center(1) = tt_e.y();
//             obj->center(2) = tt_e.z();
//         }
//         { //direction
//             Eigen::Vector4d tt_e;
//             auto &tmp = moi->direction;
//             tt_e << tmp.x(), tmp.y(), tmp.z(), 0;
//             tt_e = transform_mat * tt_e;
//             obj->direction(0) = tt_e.x();
//             obj->direction(1) = tt_e.y();
//             obj->direction(2) = tt_e.z();
//         }
//         //Keep the global parameters below
//         obj->tracker_id = moi->tracker_id;
//         obj->type = moi->track_type;
//         obj->size(0) = moi->size(0);
//         obj->size(1) = moi->size(1);
//         obj->size(2) = moi->size(2);

//         if (obj->center(0) > params.xmin && obj->center(0) < params.xmax && obj->center(1) > params.ymin && obj->center(1) < params.ymax) {
//             trk_ids.push_back(obj->tracker_id);
//             // debug show
//             std::cout << debug_info << info << obj->tracker_id
//                       << ",type: " << (obj->type == ObjectType::UNKNOW ? "RB" : "AI")
//                       << ",type: " << (int)(obj->type) //kObjectType2NameMap.at(obj->type)
//                       << ",x:" << obj->center(0) << ",y:" << obj->center(1)
//                       << ",xSize:" << obj->size(0) << ",ySize:" << obj->size(1)
//                       << ",xDir:" << obj->direction(0) << ",yDir:" << obj->direction(1)
//                       << std::endl;
//         }
//     }
//     return trk_ids;
// }

// void DebugShow::showTrackList(std::vector<TargetKalman::Ptr> &tracker_list_, std::vector<int> &trk_ids) {
//     for (size_t i = 0; i < tracker_list_.size(); ++i) {
//         int id = tracker_list_[i]->trajectory->target_buffer_.back()->tracker_id;
//         if (find(trk_ids.begin(), trk_ids.end(), id) == trk_ids.end()) {
//             continue;
//         } else {
//             std::cout << "tracker_id: " << id << "  size:" << tracker_list_[i]->trajectory->target_buffer_.size()
//                       << " isVis:" << ((tracker_list_[i]->trajectory->state == TrackerState::VISIBLE) ? "VISIBLE" : "INVIS") << std::endl;
//         }
//     }
// }

// void DebugShow::showTrackObjValid(std::pair<TrackObject::Ptr, int> &track_obj_valid, TrackObject::Ptr &current, TrackObject::Ptr &cur_buf) {
//     if (!params.enable_collect)
//         return;
//     bool show_id = true;
//     if (show_id) {
//         std::cout << "  colTrkRes" << HDEBUG_R << "trk_list id:" << current->tracker_id << ", " << cur_buf->tracker_id
//                   << "trk_list assid:" << current->asso_id << ", " << cur_buf->asso_id << std::endl;
//     }
//     auto &obj = track_obj_valid.first;
//     showTrackObject(obj, HDEBUG_R);
// }

void DebugShow::showObject(Object::Ptr &obj, std::string debug_info) {
    if ((int)(obj->type) > 0) { //&& obj->size(0) < 0.05) {
        std::cout << debug_info << obj->tracker_id
                  << ",type: " << (int)(obj->type) //kObjectType2NameMap.at(obj->type)
                  << ",x:" << obj->center(0) << ",y:" << obj->center(1)
                  << ",xSize:" << obj->size(0) << ",ySize:" << obj->size(1)
                  << ",xDir:" << obj->direction(0) << ",yDir:" << obj->direction(1)
                  << std::endl;
    }
}

// void DebugShow::showTrackObject(TrackObject::Ptr &obj, std::string debug_info) {
//     if ((int)(obj->track_type) > 0) { //&& obj->size(0) < 0.05) {
//         std::cout << debug_info << obj->tracker_id
//                   << ",type: " << (int)(obj->track_type) //kObjectType2NameMap.at(obj->type)
//                   << ",x:" << obj->center(0) << ",y:" << obj->center(1)
//                   << ",xSize:" << obj->size(0) << ",ySize:" << obj->size(1)
//                   << ",xDir:" << obj->direction(0) << ",yDir:" << obj->direction(1)
//                   << std::endl;
//     }
// }

// void DebugShow::showTargetBuffer(boost::circular_buffer<TrackObject::Ptr> &target_buffer_, Object::Ptr &obj, std::string debug_info) {
//     if ((int)(obj->type) > 0) { //&& obj->size(0) < 0.05) {
//         for (auto &ci : target_buffer_) {
//             std::cout << debug_info << "buffer:" << ci->tracker_id
//                       << ",type: " << (int)(ci->track_type) //kObjectType2NameMap.at(obj->type)
//                       << ",x:" << ci->center(0) << ",y:" << ci->center(1)
//                       << ",xSize:" << ci->size(0) << ",ySize:" << ci->size(1)
//                       << std::endl;
//         }
//     }
// }
} // namespace robosense