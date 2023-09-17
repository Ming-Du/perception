#pragma once

#include <float.h>
#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>

#include "common/include/basic_type/object.h"
#include "common/include/msg/lidar_frame_msg.h"
#include "common/include/config/rs_yamlReader.h"

#define PI 3.1415926

namespace robosense {

enum TargetState {
    TRACK_STATE_DELETE = 0,
    TRACK_STATE_INIT,
    TRACK_STATE_VISIBLE,
    TRACK_STATE_INVISIBLE,
    TRACK_STATE_ABNORMAL
};

enum ModelType {
    CV = 0,
    CTRV
};

enum ResidualType {
    CENTER = 0,
    CORNER,
    EDGE
};

struct AssociateInfo {
    double score = FLT_MAX;
    // double dx = FLT_MAX;
    // double dy = FLT_MAX;
    ModelType model = CV;
    ResidualType res_type = CENTER;

    double cv_dcenter = 0;
    double cv_dcorner = 0;
    double cv_dcorner_x = 0;
    double cv_dcorner_y = 0;
    std::pair<int, int> cv_corner_idx = std::make_pair(-1, -1);
};

class MultiFilter {
public:
    MultiFilter();
    // ~MultiFilter() = default;

    void init(const Object::Ptr &obj, const int &id, const TrackerParam &params);
    void predict(double dt);
    AssociateInfo score(const Object::Ptr &obj, double scale);
    void update(const Object::Ptr &obj, double timestamp);
    void update_rb(const Object::Ptr &obj);
    void event();
    std::pair<int, int> corner_registration(std::vector<Eigen::Vector3d> &predpts, std::vector<Eigen::Vector3d> measpts);
    void meas_correction(double cvvel, Eigen::Vector3d &measurement, bool reset_size);
    bool overlapfilter(std::vector<MultiFilter> &targets);
    void syc(ModelType model);
    void syc_result();

    // number of measurements associated with this target in this frame
    int myBboxNum_;
    // infomation of the history measurements
    // boost::circular_buffer<Object::Ptr> meas_buffer_;
    // infomation of the history trackers
    boost::circular_buffer<Object::Ptr> target_buffer_;
    // current TrackObj info
    Object::Ptr filtered_obj_;
    // target id
    int id_;
    // associated measurement id
    int meas_id_;
    // Current State
    TargetState state_;
    // motion model
    ModelType model_;
    // AssociateInfo
    AssociateInfo asso_info_;
    // matched measurement's type
    ObjectType meas_type_;
    // size of boundng box
    Eigen::Vector3d box_size_;

private:
    TrackerParam params_;
    // state vector
    Eigen::Matrix<double, 5, 1> x_ctrv_;
    Eigen::Matrix<double, 4, 1> x_cv_;
    // state covariance matrix
    Eigen::Matrix<double, 5, 5> p_ctrv_;
    Eigen::Matrix<double, 4, 4> p_cv_;
    // process covariance matrix
    Eigen::Matrix<double, 5, 5> q_ctrv_;
    Eigen::Matrix<double, 4, 4> q_cv_;
    // Jacobian matrix
    Eigen::Matrix<double, 5, 5> j_;
    // state transition matrix
    Eigen::Matrix<double, 4, 4> f_;
    // trans matrix
    Eigen::Matrix<double, 3, 5> h_ctrv_;
    Eigen::Matrix<double, 2, 4> h_cv_;
    // measurement covariance matrix
    Eigen::Matrix3d r_ctrv_;
    Eigen::Matrix2d r_cv_;
    // process noise standard deviation for a
    double std_noise_a_;
    // process noise standard deviation for yaw acceleration
    double std_noise_yaw_a_;
    // Detection state count to active
    uint16_t detect2activeCount_;
    // Detection state count to free
    uint16_t detect2freeCount_;
    // Active state count to free
    uint16_t active2freeCount_;
    // association times
    uint16_t heartbeat_;
    // the type of target
    ObjectType type_;
    // distance threshold for association
    double thredist_;
    // timestamp, unit is second
    double timestamp_;
    // theta of measurements that decide static or not
    std::vector<Eigen::Vector3d> measures_;
    // ResidualType
    ResidualType res_type_;
    // box points
    std::vector<Eigen::Vector3d> box_pts_ctrv_ = std::vector<Eigen::Vector3d>(4);
    std::vector<Eigen::Vector3d> box_pts_cv_ = std::vector<Eigen::Vector3d>(4);
    // times of maintain ai result
    uint16_t maintain_ai_result_;
    // frame counter after rb to ai
    uint16_t rb2ai_counter_;
};

} // namespace robosense