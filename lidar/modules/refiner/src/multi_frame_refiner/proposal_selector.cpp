#include "multi_frame_refiner/proposal_selector.h"

namespace robosense {

ProposalSelector::ProposalSelector() {
}

void ProposalSelector::init(const RefinerParam &param) {
    param_ = param;
}

void ProposalSelector::perception(const LidarFrameMsg::Ptr &msg_ptr,
                                  std::vector<std::pair<int, int>> &box_refine_list,
                                  std::vector<std::pair<int, std::vector<int>>> &box_merge_list) {
    std::map<int, std::vector<int>> &measure_to_trackers = msg_ptr->refine_data.measure_to_trackers;
    std::map<int, std::vector<int>> &tracker_to_measures = msg_ptr->refine_data.tracker_to_measures;
    auto &proposals = msg_ptr->objects_proposals;
    auto &objs_measure = msg_ptr->objects_refine;
    auto &objs_tracker = msg_ptr->objects;

    std::vector<int> erase_objs, erase_merge_list, erase_refine_list;
    std::map<ObjectType, std::vector<std::pair<NameMapping, int>>, std::greater<ObjectType>> candidate;

    for (size_t i = 0; i < box_merge_list.size(); i++) {
        if (objs_tracker[box_merge_list[i].first]->type == ObjectType::TRUCK ||
            objs_tracker[box_merge_list[i].first]->type == ObjectType::BUS) {
            candidate[objs_tracker[box_merge_list[i].first]->type].emplace_back(std::make_pair(MergeList, i));
        }
    }
    if (/*param_.pps_isfalcon*/ false) {
        for (size_t i = 0; i < objs_tracker.size(); i++) {
            if (tracker_to_measures.find(i) == tracker_to_measures.end()) {
                candidate[objs_tracker[i]->type].emplace_back(std::make_pair(NoList, i));
            }
        }
    }
    double difheading, dif_x, dif_y, heading1, heading2;
    for (size_t i = 0; i < box_refine_list.size(); i++) {
        auto &refine_list = box_refine_list[i];
        if (objs_tracker[refine_list.second]->type == ObjectType::UNKNOW) {
            continue;
        }
        if (objs_measure[refine_list.first]->type == ObjectType::UNKNOW) {
            candidate[objs_tracker[refine_list.second]->type].emplace_back(std::make_pair(RefineListTracker, i)); // TODO: use tracker box or rb box?
        } else {
            if (objs_measure[refine_list.first]->type == ObjectType::PED &&
                objs_tracker[refine_list.second]->type == ObjectType::PED) {
                continue;
            }
            heading1 = std::atan2(objs_measure[refine_list.first]->direction(1), objs_measure[refine_list.first]->direction(0));
            heading2 = objs_tracker[refine_list.second]->heading;
            difheading = fabs(control_psi(heading1 - heading2));
            if (difheading > 3.14159265 / 2) {
                difheading -= 3.14159265;
            }
            if (fabs(difheading) > 3.14159265 / 18) {
                // candidate[objs_tracker[refine_list.second]->type].emplace_back(std::make_pair(RefineListTracker, i));
                continue;
            }
            dif_x = fabs(objs_measure[refine_list.first]->size(0) - objs_tracker[refine_list.second]->size(0));
            dif_y = fabs(objs_measure[refine_list.first]->size(1) - objs_tracker[refine_list.second]->size(1));
            if (dif_x > 1.5 || dif_y > 1) {
                if (objs_measure[refine_list.first]->size(0) + objs_measure[refine_list.first]->size(1) >
                    objs_tracker[refine_list.second]->size(0) + objs_tracker[refine_list.second]->size(1)) {
                    // candidate[objs_measure[refine_list.first]->type].emplace_back(std::make_pair(RefineListMeasure, i));
                } else {
                    // candidate[objs_tracker[refine_list.second]->type].emplace_back(std::make_pair(RefineListTracker, i));
                }
            }
        }
    }
    int timecost = (ros::Time::now().toSec() - msg_ptr->sys_timestamp) * 1000; // unit is ms
    int selected_num;

    // TODO: 
    // if (timecost < 60) {
    //     selected_num = param_.airefine_batch_size * 3;
    // } else if (timecost < 70) {
    //     selected_num = param_.airefine_batch_size * 2;
    // } else {
    //     selected_num = param_.airefine_batch_size;
    // }
    selected_num = param_.airefine_batch_size;
    proposals.reserve(selected_num);
    for (auto &one_class_proposals : candidate) {
        if (proposals.size() == selected_num) {
            break;
        }
        for (auto &tmp : one_class_proposals.second) {
            if (proposals.size() == selected_num) {
                break;
            }
            Object::Ptr temp_obj(new Object);
            switch (tmp.first) {
            case MergeList:
                temp_obj->clone(*objs_tracker[box_merge_list[tmp.second].first]);
                temp_obj->is_ai_refine = AiRefine_State::NO_AIREFINE;
                temp_obj->ismergelist = true;
                temp_obj->measure_id = -1;
                temp_obj->cell_indices.clear();
                proposals.emplace_back(temp_obj);
                erase_merge_list.emplace_back(tmp.second);
                for (auto i : box_merge_list[tmp.second].second) {
                    erase_objs.emplace_back(i);
                }
                break;
            case NoList:
                temp_obj->clone(*objs_tracker[tmp.second]);
                temp_obj->is_ai_refine = AiRefine_State::NO_AIREFINE;
                temp_obj->ismergelist = false;
                temp_obj->measure_id = -1;
                temp_obj->cell_indices.clear();
                proposals.emplace_back(temp_obj);
                break;
            case RefineListTracker:
                temp_obj->clone(*objs_tracker[box_refine_list[tmp.second].second]);
                temp_obj->is_ai_refine = AiRefine_State::NO_AIREFINE;
                temp_obj->ismergelist = false;
                if (objs_measure[box_refine_list[tmp.second].first]->type == ObjectType::UNKNOW) {
                    temp_obj->measure_id = box_refine_list[tmp.second].first;
                }
                else {
                    temp_obj->measure_id = -1;
                }
                temp_obj->cell_indices.clear();
                proposals.emplace_back(temp_obj);
                erase_refine_list.emplace_back(tmp.second);
                erase_objs.emplace_back(box_refine_list[tmp.second].first);
                break;
            case RefineListMeasure:
                temp_obj->clone(*objs_measure[box_refine_list[tmp.second].first]);
                temp_obj->is_ai_refine = AiRefine_State::NO_AIREFINE;
                temp_obj->ismergelist = false;
                temp_obj->measure_id = -1;
                temp_obj->cell_indices.clear();
                temp_obj->heading = std::atan2(temp_obj->direction(1), temp_obj->direction(0));
                proposals.emplace_back(temp_obj);
                erase_refine_list.emplace_back(tmp.second);
                erase_objs.emplace_back(box_refine_list[tmp.second].first);
                break;
            default:
                break;
            }
        }
    }
    CellInfoPtr cell_info_ptr;
    double expand_proposal_meter;
    Eigen::Vector3d proposal_size;
    for (auto &cell_id : msg_ptr->grid_map_ptr->roi_cell_id_vec) {
        cell_info_ptr = msg_ptr->grid_map_ptr->getCellInfo(cell_id);
        for (auto &obj : proposals) {
            Eigen::Vector2d point;
            expand_proposal_meter = param_.expand_proposal_meter;
            if (obj->size(0) > param_.proposal_cls_thres) {
                expand_proposal_meter = param_.other_expand_ratio * obj->size(0);
            }
            proposal_size(0) = obj->size(0) + expand_proposal_meter;
            proposal_size(1) = obj->size(1) + expand_proposal_meter;
            std::vector<Eigen::Vector2d> pps_box(4);
            pps_box[0] << obj->center(0), obj->center(1);
            pps_box[1] << cos(obj->heading), sin(obj->heading);
            pps_box[2] << -sin(obj->heading), cos(obj->heading);
            pps_box[3] = proposal_size.head<2>();
            point << cell_info_ptr->local_x_, cell_info_ptr->local_y_;
            // point << msg_ptr->scan_ptr->points[cell_info_ptr->points_indices_[0]].x, msg_ptr->scan_ptr->points[cell_info_ptr->points_indices_[0]].y;
            if (IsPointInBBox(pps_box[0], pps_box[1], pps_box[2], pps_box[3], point)) {
                obj->cell_indices.emplace_back(cell_id);
                break;
            }
        }
    }

    if (erase_merge_list.size() != 0) {
        reverse(erase_merge_list.begin(), erase_merge_list.end());
        for (const auto &i : erase_merge_list) {
            box_merge_list.erase(box_merge_list.begin() + i);
        }
    }
    if (erase_refine_list.size() != 0) {
        reverse(erase_refine_list.begin(), erase_refine_list.end());
        for (const auto &i : erase_refine_list) {
            box_refine_list.erase(box_refine_list.begin() + i);
        }
    }

    msg_ptr->refine_data.erase_objs.insert(msg_ptr->refine_data.erase_objs.end(), erase_objs.begin(), erase_objs.end());
}

} // namespace robosense