#include "bev_aggregation/rule_object_filter.h"

namespace robosense {

RuleObjectFilter::RuleObjectFilter() {
}

void RuleObjectFilter::init(const RefinerParam& param){
    param_ = param;
}

void RuleObjectFilter::perception(const LidarFrameMsg::Ptr &msg_ptr){
    if(param_.floating_filter)
        ghostObjFilter(msg_ptr);
    if(param_.floating_filter)
        floatingObjFilter(msg_ptr);
    if(param_.flowerbed_filter)
        flowerbedObjFilter(msg_ptr);
}

void RuleObjectFilter::ghostObjFilter(const LidarFrameMsg::Ptr &msg_ptr){
    auto &obj_rb = msg_ptr->objects_rule;
    for (auto it = obj_rb.begin(); it != obj_rb.end();) {
        if (param_.ghost_range.inRange((*it)->center(0), (*it)->center(1), (*it)->center(2)) 
            && (*it)->size(2) < param_.ghost_size.at(2) 
            && (*it)->size(1) < param_.ghost_size.at(1) 
            && (*it)->size(0) < param_.ghost_size.at(0)) {
            obj_rb.erase(it);
        } else{
            it++;
        }
    }
}

void RuleObjectFilter::floatingObjFilter(const LidarFrameMsg::Ptr &msg_ptr){
    auto &obj_rb = msg_ptr->objects_rule;
    for (auto it = obj_rb.begin(); it != obj_rb.end();) {
        double obj_height_min = (*it)->center(2) - (*it)->size(2) / 2;
        if (param_.floating_range.inRange2D((*it)->center(0), (*it)->center(1)) 
            && obj_height_min > param_.floating_height_limited
            && (*it)->size(2) < param_.floating_object_size.at(2)
            && (*it)->size(1) < param_.floating_object_size.at(1)
            && (*it)->size(0) < param_.floating_object_size.at(0)) {
            obj_rb.erase(it);
        } else{
            it++;
        }
    }
}

void RuleObjectFilter::flowerbedObjFilter(const LidarFrameMsg::Ptr &msg_ptr){
    auto &obj_rb = msg_ptr->objects_rule;
    for (auto it = obj_rb.begin(); it != obj_rb.end();) {
        double obj_height_min = (*it)->center(2) - (*it)->size(2) / 2;
        if ((*it)->status >= RoadType::ROADSIDE
            && param_.flowerbed_range.inRange2D((*it)->center(0), (*it)->center(1)) 
            && ((*it)->size(2) < param_.flowerbed_object_size.at(2) || obj_height_min > param_.flowerbed_height_limited)) {
            obj_rb.erase(it);
        } else{
            it++;
        }
    }
}

} // namespace robosense