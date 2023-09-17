#pragma once

#include "common/include/tic_toc.h"
#include "common/include/msg/lidar_frame_msg.h"
#include "common/include/config/rs_yamlReader.h"
#include "common/include/build_object/object_build.h"

#include "bev_aggregation/rule_object_filter.h"

namespace robosense {

class BevAggregation {
public:
    using Ptr = std::shared_ptr<BevAggregation>;

    BevAggregation();

    void init(const RefinerParam& param);
    void perception(const LidarFrameMsg::Ptr &msg_ptr);

private:
    bool isPolygonIntersection(Object::Ptr& rb_obj, Object::Ptr& ai_obj);
    void clipeRBbyCellinfo(const LidarFrameMsg::Ptr &msg_ptr, Object::Ptr& rb_obj, Object::Ptr& ai_obj, size_t rb_i);
    bool splitRb(const LidarFrameMsg::Ptr &msg_ptr, Object::Ptr& rb_obj, Object::Ptr& ai_obj,Object::Ptr& rb_inAIpart, Object::Ptr& rb_outAIpart, size_t rb_i);
    bool buildClippedObject(const PointCloud::Ptr& cloud_ptr, const Object::Ptr& obj_ptr);
    bool isNeedSplit(const LidarFrameMsg::Ptr &msg_ptr,Object::Ptr& rb_obj, Object::Ptr& ai_obj);
    bool isInAibox(double input_x,double input_y, Object::Ptr& ai_obj);

private:
    RefinerParam params_;

    RuleObjectFilter::Ptr rule_object_filter_ptr_;
    std::map<int,NoiseType> cellid_noisetype_map_;

};

} // namespace robosense