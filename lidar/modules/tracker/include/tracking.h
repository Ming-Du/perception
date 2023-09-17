#pragma once

#include "multifilter.h"

#define SIZE 1000

namespace robosense {

class Tracking {
public:
    using Ptr = std::shared_ptr<Tracking>;
    Tracking() : dt_(0.1), timestamp_(0), id_(0), idlist_(SIZE, false){};
    ~Tracking() = default;
    void init(const Rs_YAMLReader &configParse);
    void predict(const LidarFrameMsg::Ptr &msg_ptr);
    void perception(const LidarFrameMsg::Ptr &msg_ptr);

private:
    void preprocess(const LidarFrameMsg::Ptr &msg_ptr);
    void associate();
    void allocate();
    void update(double timestamp);
    void report(const LidarFrameMsg::Ptr &msg_ptr);
    void get_new_id();

private:
    TrackerParam params_;

    std::vector<MultiFilter> targets_;
    std::vector<Object::Ptr> input_objs_;
    std::vector<int> bestInd_;
    std::vector<bool> idlist_;
    double timestamp_;
    double dt_;
    int id_;
};

} // namespace robosense