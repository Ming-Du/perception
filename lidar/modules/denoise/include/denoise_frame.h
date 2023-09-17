#ifndef DENOISE_FRAME_H_
#define DENOISE_FRAME_H_

#include <ros/ros.h>
#include <iostream>
#include <vector>

#include "common/include/msg/lidar_frame_msg.h"
#include "common/include/basic_type/rotate_box.h"

#include "common/proto/convexhull.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/hadmap.pb.h"
#include "common/proto/localization.pb.h"
#include "common/proto/object.pb.h"

#include "xgb.h"
#include "denoise_feature_extractor.h"

namespace robosense {

class DenoiseFrame {
public:
    using Ptr = std::shared_ptr<DenoiseFrame>;

    DenoiseFrame(){};

    void init(const Rs_YAMLReader &configParse);
    void perception(const LidarFrameMsg::Ptr &msg_ptr);

private:
    NoiseState multiTrkObjState( std::vector<NoiseState> &noise_states);
    NoiseState calclabelFromTracker(const int trk_id, std::vector<boost::circular_buffer<Object::Ptr>> &obj_trk, Object::Ptr obj_ri);
    bool print_obj(const Object::Ptr &ori, std::string name,double x_min,double x_max,double y_min,double y_max);
    void setDenoiseHistory(Object::Ptr &obj_ri, boost::circular_buffer<Object::Ptr> &list);
    // void init(const Rs_YAMLReader &configParse);
    bool isInRange(const LidarFrameMsg::Ptr &msg_ptr, double x, double y);
    double calcWeightGrid(int grid_num, int noise_count, int obj_count, double gridsingle_weight);
    bool isWeightInvalid(double weight);
    bool isSameSign(double a, double b);
    void recalcSuspectedbyWeight(const LidarFrameMsg::Ptr &msg_ptr);
    NoiseState weightedSum(double grid_weight,double obj_weight, double invalid_threshold, double valid_threshold);
    void calcLabelFromGridObj(const LidarFrameMsg::Ptr &msg_ptr);
    bool isRatioEnoughInHistory(boost::circular_buffer<Object::Ptr> list, int near_history_num, float detect_min_ratio, std::function<bool(Object::Ptr)> condition,bool is_countInvisible);
    void setLabelFromHistory(const LidarFrameMsg::Ptr &msg_ptr);
    void showObjCorners(Object::Ptr& obj);

public:
    bool denoise_enable_;
    DenoiseParam params_;

    std::mutex mutex_;
};


} // namespace robosense
#endif