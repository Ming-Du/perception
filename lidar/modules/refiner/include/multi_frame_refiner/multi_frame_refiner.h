#pragma once

#include <condition_variable>
#include <mutex>
#include <thread>

#include "multi_frame_refiner/proposal_selector.h"
#include "multi_frame_refiner/special_object_filter.h"

#include "common/include/build_object/object_build.h"
#include "common/include/config/rs_yamlReader.h"
#include "common/include/msg/lidar_frame_msg.h"

namespace robosense {

class MultiFrameRefiner {
public:
    using Ptr = std::shared_ptr<MultiFrameRefiner>;

    MultiFrameRefiner();
    ~MultiFrameRefiner();

    void init(const RefinerParam &param);
    void perception(const LidarFrameMsg::Ptr &msg_ptr);

    void set_cv(std::condition_variable *cv, bool ai_refiner_enable);

private:
    void match_meas_tracker(const LidarFrameMsg::Ptr &msg_ptr, std::vector<int> &rb_cluster_list,
                            std::vector<std::pair<int, int>> &box_refine_list,
                            std::vector<std::pair<int, std::vector<int>>> &box_merge_list,
                            std::vector<std::pair<int, std::vector<int>>> &box_split_list);

    void objs_split_merge(const LidarFrameMsg::Ptr &msg_ptr,
                          std::vector<std::pair<int, std::vector<int>>> &box_merge_list,
                          std::vector<std::pair<int, std::vector<int>>> &box_split_list);

    void merge_unmatched_rb(const LidarFrameMsg::Ptr &msg_ptr, std::vector<int> &rb_cluster_list);

    void multi_box_refine(const LidarFrameMsg::Ptr &msg_ptr, std::vector<std::pair<int, int>> &box_refine_list);

    void add_refineobj_vec(const LidarFrameMsg::Ptr &msg_ptr);

    bool isBoxIntersection(Object::Ptr &measure_obj, Object::Ptr &tracker_obj);

    bool isBoxIntersection_v2(Object::Ptr &measure_obj, Object::Ptr &tracker_obj);

    void Process();

private:
    ProposalSelector::Ptr proposal_selector_ptr_;
    SpecialObjectFilter::Ptr special_obj_filter_ptr_;

    std::vector<Object::Ptr> add_objs;
    std::vector<int> rb_cluster_list;
    std::vector<std::pair<int, int>> box_refine_list;
    std::vector<std::pair<int, std::vector<int>>> box_merge_list; // first is tracker id, second are measures' id
    std::vector<std::pair<int, std::vector<int>>> box_split_list; // first is measure id, second are trackers' id

    std::mutex mutex_;
    std::condition_variable *ai_refiner_cv_;
    bool ai_refiner_enable_;
};

} // namespace robosense