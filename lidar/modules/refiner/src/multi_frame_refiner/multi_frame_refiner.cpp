#include "multi_frame_refiner/multi_frame_refiner.h"

#include "common/include/basic_type/rotate_box.h"
#include "common/include/tic_toc.h"

#include <stack>

namespace robosense {

MultiFrameRefiner::MultiFrameRefiner()
    : proposal_selector_ptr_(new ProposalSelector)
    , special_obj_filter_ptr_(new SpecialObjectFilter)
    , ai_refiner_enable_(false) 
{

}

MultiFrameRefiner::~MultiFrameRefiner() {
    
}

void MultiFrameRefiner::init(const RefinerParam &param) {
    special_obj_filter_ptr_->init(param);
    proposal_selector_ptr_->init(param);
}

void MultiFrameRefiner::set_cv(std::condition_variable *cv, bool ai_refiner_enable) {
    ai_refiner_cv_ = cv;
    ai_refiner_enable_ = ai_refiner_enable;
}

void MultiFrameRefiner::perception(const LidarFrameMsg::Ptr &msg_ptr) {
    add_objs.clear();
    rb_cluster_list.clear();
    box_refine_list.clear();
    box_merge_list.clear();
    box_split_list.clear();

    {
        TicToc timer("perception/refiner/multi frame refiner/matcher");
        TRY_CATCH
        match_meas_tracker(msg_ptr, rb_cluster_list, box_refine_list, box_merge_list, box_split_list);
        END_TRY_CATCH
    }

    {
        if (ai_refiner_enable_) {
            TicToc timer("perception/refiner/multi frame refiner/pps");
            TRY_CATCH
            proposal_selector_ptr_->perception(msg_ptr, box_refine_list, box_merge_list);
            END_TRY_CATCH

            ai_refiner_cv_->notify_one();
        }
    }

    {
        TicToc timer("perception/refiner/multi frame refiner/split");
        TRY_CATCH
        objs_split_merge(msg_ptr, box_merge_list, box_split_list);
        END_TRY_CATCH
    }
    // multi_box_refine(msg_ptr, box_refine_list);
    {
        TicToc timer("perception/refiner/multi frame refiner/merge");
        TRY_CATCH
        merge_unmatched_rb(msg_ptr, rb_cluster_list);
        END_TRY_CATCH
    }

    // TODO merge unmatched RB
    // TODO denoise P2
    //  update_refineobj_vec(msg_ptr, erase_objs, add_objs);
    {
        TicToc timer("perception/refiner/multi frame refiner/update");
        TRY_CATCH
        add_refineobj_vec(msg_ptr);
        END_TRY_CATCH
    }

    {
        TicToc timer("perception/refiner/multi frame refiner/filter");
        TRY_CATCH
        special_obj_filter_ptr_->perception(msg_ptr);
        END_TRY_CATCH
    }
}

void MultiFrameRefiner::match_meas_tracker(const LidarFrameMsg::Ptr &msg_ptr, std::vector<int> &rb_cluster_list,
                                           std::vector<std::pair<int, int>> &box_refine_list,
                                           std::vector<std::pair<int, std::vector<int>>> &box_merge_list,
                                           std::vector<std::pair<int, std::vector<int>>> &box_split_list) {
    std::map<int, std::vector<int>> &measure_to_trackers = msg_ptr->refine_data.measure_to_trackers;
    std::map<int, std::vector<int>> &tracker_to_measures = msg_ptr->refine_data.tracker_to_measures;
    std::map<int, bool> pass_meas;
    auto &objs_measure = msg_ptr->objects_refine;
    auto &objs_tracker = msg_ptr->objects;
    double dist, dx, dy;

    for (size_t i = 0; i < objs_measure.size(); i++) {
        if (objs_measure[i]->group == GroupType::GROUP) {
            continue;
        }
        for (size_t j = 0; j < objs_tracker.size(); j++) {
            if (objs_tracker[j]->group == GroupType::GROUP) {
                continue;
            }
            dx = objs_measure[i]->center(0) - objs_tracker[j]->center(0);
            dy = objs_measure[i]->center(1) - objs_tracker[j]->center(1);
            dist = sqrt(dx * dx + dy * dy);
            if (dist > 10) {
                continue;
            }
            if (isBoxIntersection(objs_measure[i], objs_tracker[j])) {
                measure_to_trackers[i].emplace_back(j);
                tracker_to_measures[j].emplace_back(i);
            }
        }
        if (measure_to_trackers.find(i) == measure_to_trackers.end() &&
            objs_measure[i]->type == ObjectType::UNKNOW && objs_measure[i]->status == RoadType::ROAD) {
            rb_cluster_list.emplace_back(i);
        }
    }

    for (auto &track_list : measure_to_trackers) {
        int measure_id = track_list.first;
        if (pass_meas.find(measure_id) != pass_meas.end()) {
            continue;
        }
        if (track_list.second.size() == 1) {
            int tracker_id = track_list.second[0];
            if (tracker_to_measures[tracker_id].size() == 1) {
                std::pair<int, int> tmp(measure_id, tracker_id);
                box_refine_list.emplace_back(tmp);
            } else {
                std::pair<int, std::vector<int>> tmp;
                std::vector<int> tmp_vec;
                for (auto &idx : tracker_to_measures[tracker_id]) {
                    if (pass_meas.find(idx) != pass_meas.end()) {
                        continue;
                    }
                    if (measure_to_trackers[idx].size() == 1) {
                        tmp_vec.emplace_back(idx);
                        pass_meas[idx] = true;
                    }
                }
                if (tmp_vec.size() > 1) {
                    tmp.first = tracker_id;
                    tmp.second = tmp_vec;
                    box_merge_list.emplace_back(tmp);
                }
            }
        } else {
            std::pair<int, std::vector<int>> tmp;
            std::vector<int> tmp_vec;
            for (auto &idx : track_list.second) {
                if (tracker_to_measures[idx].size() == 1) {
                    tmp_vec.emplace_back(idx);
                }
            }
            if (tmp_vec.size() > 1) {
                tmp.first = measure_id;
                tmp.second = tmp_vec;
                box_split_list.emplace_back(tmp);
            }
        }
    }
    // std::cout << "refine box num: " << box_refine_list.size() << std::endl;
    // std::cout << "merge box num: " << box_merge_list.size() << std::endl;
    // std::cout << "split box num: " << box_split_list.size() << std::endl;
}

void MultiFrameRefiner::objs_split_merge(const LidarFrameMsg::Ptr &msg_ptr,
                                         std::vector<std::pair<int, std::vector<int>>> &box_merge_list,
                                         std::vector<std::pair<int, std::vector<int>>> &box_split_list) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto &erase_objs = msg_ptr->refine_data.erase_objs;
    auto &objs_measure = msg_ptr->objects_refine;
    auto &objs_tracker = msg_ptr->objects;
    auto &objs_history = msg_ptr->objects_history;

    // split
    for (auto &split_list : box_split_list) {
        if (objs_measure[split_list.first]->type != ObjectType::UNKNOW) {
            continue;
        }
        std::vector<int> split_id;
        for (auto &idx : split_list.second) { // each trackers
            if (objs_tracker[idx]->type == ObjectType::UNKNOW) {
                continue;
            }
            int match_time = 0;
            int match_ai_time = 0;
            for (auto &his_obj : objs_history[idx]) { // each circular_buffer
                if (his_obj->useful) {
                    if (his_obj->type != ObjectType::UNKNOW) {
                        match_time++;
                        match_ai_time++;
                    } else {
                        match_time++;
                    }
                }
            }
            if (match_time > 8 || match_ai_time > 5) {
                split_id.emplace_back(idx);
            }
        }
        if (split_id.size() > split_list.second.size() / 2) {
            erase_objs.emplace_back(split_list.first);
            for (auto &id : split_id) {
                Object::Ptr split_obj(new Object);
                split_obj->clone(*objs_tracker[id]);
                add_objs.push_back(split_obj);
            }
        }
    }

    // merge
    GridMapPtr grid_map_ptr = msg_ptr->grid_map_ptr;
    bool is_debugobj = false;
    RsObjectBuilderInitOptions build_option;
    RsObjectBuilder builder(build_option);
    const auto &cloud_ptr = msg_ptr->scan_ptr;

    for (auto &merge_list : box_merge_list) {
        if (/*objs_tracker[merge_list.first]->type == ObjectType::UNKNOW ||*/ // TODO: need to complete multi_box_refine
            objs_tracker[merge_list.first]->status != RoadType::ROAD) {
            continue;
        }
        int match_time = 0;
        int match_ai_time = 0;
        for (auto &his_obj : objs_history[merge_list.first]) {
            if (his_obj->useful) {
                if (his_obj->type != ObjectType::UNKNOW) {
                    match_time++;
                    match_ai_time++;
                } else {
                    match_time++;
                }
            }
        }
        if (match_time < 3 && match_ai_time < 2) {
            continue;
        }
        std::vector<int> merge_id;
        for (auto &idx : merge_list.second) {
            if (objs_measure[idx]->type == ObjectType::UNKNOW && objs_measure[idx]->status == RoadType::ROAD) {
                merge_id.emplace_back(idx);
            }
        }
        if (merge_id.size() > 1) {
            Object::Ptr merge_obj(new Object);
            for (auto &idx : merge_id) {
                // objs_measure[idx]->getAllPointsFromCells(grid_map_ptr);
                merge_obj->cloud_indices.insert(merge_obj->cloud_indices.end(), objs_measure[idx]->cloud_indices.begin(), objs_measure[idx]->cloud_indices.end());
                merge_obj->polygons.insert(merge_obj->polygons.end(), objs_measure[idx]->polygons.begin(), objs_measure[idx]->polygons.end());
            }
            if (builder.buildObjectByPolygon(merge_obj->polygons, merge_obj, is_debugobj)) {
                double difheading;
                merge_obj->type = objs_tracker[merge_list.first]->type;
                merge_obj->exist_confidence = objs_tracker[merge_list.first]->exist_confidence;
                merge_obj->type_confidence = objs_tracker[merge_list.first]->type_confidence;
                if (merge_obj->type_confidence < 0.01) {
                    merge_obj->type_confidence = 0.2;
                }
                merge_obj->heading = std::atan2(merge_obj->direction(1), merge_obj->direction(0));
                difheading = fabs(control_psi(merge_obj->heading - objs_tracker[merge_list.first]->heading));
                if (difheading > 3.1415926 / 4) {
                    double tmp_size;
                    merge_obj->heading = control_psi(merge_obj->heading + (3.1415926 / 2));
                    merge_obj->direction << cosf(merge_obj->heading), sinf(merge_obj->heading), 0.0;
                    tmp_size = merge_obj->size(0);
                    merge_obj->size(0) = merge_obj->size(1);
                    merge_obj->size(1) = tmp_size;
                }
                // std::cout << "refine merge: " << objs_tracker[merge_list.first]->tracker_id << std::endl;
                add_objs.push_back(merge_obj);
                erase_objs.insert(erase_objs.end(), merge_id.begin(), merge_id.end());
            }
        }
    }

    // std::cout << "delete size: " << erase_objs.size() << std::endl;

    // std::cout << "add size: " << add_objs.size() << std::endl;
}

#if 0
void MultiFrameRefiner::multi_box_refine(const LidarFrameMsg::Ptr &msg_ptr, std::vector<std::pair<int, int>> &box_refine_list) {
    auto &objs_measure = msg_ptr->objects_refine;
    auto &objs_tracker = msg_ptr->objects;

    for (auto &idx : box_refine_list) {
        if (objs_measure[idx.first]->type == ObjectType::UNKNOW && objs_tracker[idx.second]->type != ObjectType::UNKNOW) {}
    }
}
#endif

void MultiFrameRefiner::merge_unmatched_rb(const LidarFrameMsg::Ptr &msg_ptr, std::vector<int> &rb_cluster_list) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto &erase_objs = msg_ptr->refine_data.erase_objs;
    auto &objs_measure = msg_ptr->objects_refine;

    std::vector<int> label_mat;
    int tot_obj_num = 0;
    float x, y, xtmp, ytmp, dist;

    label_mat.resize(rb_cluster_list.size(), 0);
    for (size_t i = 0; i < rb_cluster_list.size(); i++) {
        if (label_mat[i] != 0) {
            continue;
        }

        std::stack<int> neighbor_objs;
        neighbor_objs.push(i);
        tot_obj_num++;
        label_mat[i] = tot_obj_num;

        while (!neighbor_objs.empty()) {
            int cur_obj = neighbor_objs.top();
            neighbor_objs.pop();
            x = objs_measure[rb_cluster_list[cur_obj]]->center(0);
            y = objs_measure[rb_cluster_list[cur_obj]]->center(1);

            for (size_t j = 0; j < rb_cluster_list.size(); j++) {
                if (label_mat[j] != 0) {
                    continue;
                }
                xtmp = objs_measure[rb_cluster_list[j]]->center(0);
                ytmp = objs_measure[rb_cluster_list[j]]->center(1);
                dist = sqrt((x - xtmp) * (x - xtmp) + (y - ytmp) * (y - ytmp));
                if (dist > 4) {
                    continue;
                }
                label_mat[j] = tot_obj_num;
                neighbor_objs.push(j);
            }
        }
    }

    std::vector<std::vector<int>> tmp_obj_vec(tot_obj_num);
    const auto &cloud_ptr = msg_ptr->scan_ptr;
    GridMapPtr grid_map_ptr = msg_ptr->grid_map_ptr;
    RsObjectBuilderInitOptions build_option;
    RsObjectBuilder builder(build_option);
    bool is_debugobj = false;
    int label;
    for (size_t i = 0; i < label_mat.size(); i++) {
        label = label_mat[i] - 1;
        tmp_obj_vec[label].emplace_back(i);
    }
    for (auto &obj : tmp_obj_vec) {
        if (obj.size() > 1) {
            Object::Ptr cluster_obj(new Object);
            std::vector<int> tmp_erase;
            RoadType tmp_status = RoadType::ROAD;
            for (auto &idx : obj) {
                if (objs_measure[rb_cluster_list[idx]]->status != RoadType::ROAD) {
                    tmp_status = objs_measure[rb_cluster_list[idx]]->status;
                }
                tmp_erase.emplace_back(rb_cluster_list[idx]);
                // objs_measure[rb_cluster_list[idx]]->getAllPointsFromCells(grid_map_ptr);
                cluster_obj->cloud_indices.insert(cluster_obj->cloud_indices.end(), objs_measure[rb_cluster_list[idx]]->cloud_indices.begin(), objs_measure[rb_cluster_list[idx]]->cloud_indices.end());
                cluster_obj->polygons.insert(cluster_obj->polygons.end(), objs_measure[rb_cluster_list[idx]]->polygons.begin(), objs_measure[rb_cluster_list[idx]]->polygons.end());
            }
            cluster_obj->status = tmp_status;
            if (builder.buildObjectByPolygon(cluster_obj->polygons, cluster_obj, is_debugobj)) {
                cluster_obj->type = ObjectType::UNKNOW;
                add_objs.push_back(cluster_obj);
                erase_objs.insert(erase_objs.end(), tmp_erase.begin(), tmp_erase.end());
            }
        }
    }
}

void MultiFrameRefiner::add_refineobj_vec(const LidarFrameMsg::Ptr &msg_ptr) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto &objs_measure = msg_ptr->objects_refine;

    // // delete measurement
    // if (erase_objs.size() != 0) {
    //     std::sort(erase_objs.begin(), erase_objs.end(), std::greater<int>());
    //     for (const auto &i : erase_objs) {
    //         objs_measure.erase(objs_measure.begin() + i);
    //     }
    // }
    // add measurement
    objs_measure.insert(objs_measure.end(), add_objs.begin(), add_objs.end());
}

bool MultiFrameRefiner::isBoxIntersection(Object::Ptr &measure_obj, Object::Ptr &tracker_obj) {
    // compute measure box
    const auto &center_measure = measure_obj->center;
    const auto &size_measure = measure_obj->size;
    const auto &directionr_measure = measure_obj->direction;
    RotateBox box_measure(center_measure, size_measure, directionr_measure);
    std::vector<Eigen::Vector3d> corners_measure;
    box_measure.corners(corners_measure);

    // compute tracker box
    const auto &center_tracker = tracker_obj->center;
    const auto &size_tracker = tracker_obj->size * 1.2;
    const auto &directionr_tracker = tracker_obj->direction;
    RotateBox box_tracker(center_tracker, size_tracker, directionr_tracker);
    std::vector<Eigen::Vector3d> corners_tracker;
    box_tracker.corners(corners_tracker);

    // intersection or not
    for (size_t n = 0; n < 4; n++) {
        auto &input_x = corners_measure[n][0];
        auto &input_y = corners_measure[n][1];
        int left_cross = 0, right_cross = 0;
        for (int i = 0; i < 4; ++i) {
            auto &p1 = corners_tracker[i];
            auto &p2 = corners_tracker[(i + 1) % 4];

            if (p1.y() == p2.y() && std::fabs(input_y - p1.y()) < 0.0001 && input_x < std::max(p1.x(), p2.x()) && input_x > std::min(p1.x(), p2.x())) {
                return true;
            }
            if (p1.y() == p2.y()) {
                continue;
            }
            if (input_y <= std::min(p1.y(), p2.y())) {
                continue;
            }
            if (input_y > std::max(p1.y(), p2.y())) {
                continue;
            }
            double x = (input_y - p1.y()) * (p2.x() - p1.x()) / (p2.y() - p1.y()) + p1.x();
            if (std::fabs(x - input_x) < 0.005) {
                return true;
            } else if (x > input_x)
                right_cross++;
            else
                left_cross++;
        }
        if (right_cross % 2 == 1 && left_cross % 2 == 1) {
            return true;
        }
    }

    for (int i = 0; i < 4; ++i) {
        auto &input_x = corners_tracker[i].x();
        auto &input_y = corners_tracker[i].y();
        int left_cross = 0, right_cross = 0;
        for (size_t n = 0; n < 4; n++) {
            auto &p1 = corners_measure[n];
            auto &p2 = corners_measure[(n + 1) % 4];

            if (p1.y() == p2.y() && std::fabs(input_y - p1.y()) < 0.0001 && input_x < std::max(p1.x(), p2.x()) && input_x > std::min(p1.x(), p2.x())) {
                return true;
            }
            if (p1.y() == p2.y()) {
                continue;
            }
            if (input_y <= std::min(p1.y(), p2.y())) {
                continue;
            }
            if (input_y > std::max(p1.y(), p2.y())) {
                continue;
            }
            double x = (input_y - p1.y()) * (p2.x() - p1.x()) / (p2.y() - p1.y()) + p1.x();
            if (std::fabs(x - input_x) < 0.005) {
                return true;
            } else if (x > input_x)
                right_cross++;
            else
                left_cross++;
        }
        if (right_cross % 2 == 1 && left_cross % 2 == 1) {
            return true;
        }
    }

    return false;
}

bool MultiFrameRefiner::isBoxIntersection_v2(Object::Ptr &measure_obj, Object::Ptr &tracker_obj) {
    std::vector<Eigen::Vector2d> tracker_box(4);
    std::vector<Eigen::Vector2d> measure_box(4);
    std::vector<Eigen::Vector3d> corners_measure(4);
    Eigen::Vector2d point;

    tracker_box[0] << tracker_obj->center(0), tracker_obj->center(1);
    tracker_box[1] << cos(tracker_obj->heading), sin(tracker_obj->heading);
    tracker_box[2] << -sin(tracker_obj->heading), cos(tracker_obj->heading);
    tracker_box[3] = tracker_obj->size.head<2>() * 1.1;
    measure_obj->heading = std::atan2(measure_obj->direction(1), measure_obj->direction(0));
    measure_obj->compute_bndbox_points(corners_measure);
    measure_box[0] << measure_obj->center(0), measure_obj->center(1);
    measure_box[1] << cos(measure_obj->heading), sin(measure_obj->heading);
    measure_box[2] << -sin(measure_obj->heading), cos(measure_obj->heading);
    measure_box[3] = measure_obj->size.head<2>();

    for (auto &pt : corners_measure) {
        point << pt(0), pt(1);
        if (IsPointInBBox(tracker_box[0], tracker_box[1], tracker_box[2], tracker_box[3], point)) {
            return true;
        }
    }
    point << tracker_obj->center(0), tracker_obj->center(1);
    if (IsPointInBBox(measure_box[0], measure_box[1], measure_box[2], measure_box[3], point)) {
        return true;
    }

    return false;
}

} // namespace robosense