#include "multifilter.h"
#include "common/include/basic_type/rotate_box.h"
namespace robosense {

MultiFilter::MultiFilter() {
    // below parameters initialized only in construction
    h_ctrv_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 0, 1, 0;

    h_cv_ << 1, 0, 0, 0,
        0, 1, 0, 0;

    p_ctrv_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 500, 0, 0,
        0, 0, 0, 2.4, 0,
        0, 0, 0, 0, 0.4;

    p_cv_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 500, 0,
        0, 0, 0, 500;

    r_ctrv_ << 0.2, 0, 0,
        0, 0.2, 0,
        0, 0, 0.01;

    r_cv_ << 0.2, 0,
        0, 0.2;

    state_ = TRACK_STATE_INIT;
    active2freeCount_ = 0;
    detect2activeCount_ = 0;
    detect2freeCount_ = 0;
    heartbeat_ = 0;

    maintain_ai_result_ = 0;
    rb2ai_counter_ = 0;

    std_noise_a_ = 5;
    std_noise_yaw_a_ = 0.6;

    // below parameters will be initialized again according to values of measerments, so values here don't matter
    x_ctrv_ << 0, 0, 0, 0, 0; // x, y, v, theta, w
    x_cv_ << 0, 0, 0, 0;      // x, y, vx, vy

    j_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

    f_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

    q_ctrv_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

    q_cv_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    box_size_ << 0, 0, 0; // length, width, height
    type_ = ObjectType::UNKNOW;
    id_ = -1;
    model_ = CTRV;
    thredist_ = 0;
    myBboxNum_ = 1;
    timestamp_ = 0;
    // meas_buffer_ = boost::circular_buffer<Object::Ptr>(10);
    target_buffer_ = boost::circular_buffer<Object::Ptr>(10);
}

void MultiFilter::init(const Object::Ptr &obj, const int &id, const TrackerParam &params) {
    Object::Ptr measure_obj(new Object);
    measure_obj->clone(*obj);
    
    x_ctrv_ << obj->center(0), obj->center(1), 0, obj->heading, 0;
    x_cv_ << obj->center(0), obj->center(1), 0, 0;
    box_size_ = obj->size;
    id_ = id;
    myBboxNum_ = 1;
    filtered_obj_ = measure_obj;
    if (obj->type == ObjectType::UNKNOW && obj->status == RoadType::ROAD &&
        (obj->size(0) > 0.8 || obj->size(1) > 0.8) && (obj->size(0) < 2.8 && obj->size(1) < 2.8)) {
        p_ctrv_(2, 2) = 100;
        p_cv_(2, 2) = 100;
        p_cv_(3, 3) = 100;
    }
    else if (obj->type == ObjectType::UNKNOW) {
        p_ctrv_(2, 2) = 1;
        p_cv_(2, 2) = 1;
        p_cv_(3, 3) = 1;
    }
    else if ((box_size_(0) < 1.3 && box_size_(1) < 1.3) || (box_size_(0) * box_size_(1) < 0.8)) {
        p_ctrv_(2, 2) = 5;
        p_cv_(2, 2) = 5;
        p_cv_(3, 3) = 5;
    }

    params_ = params;
}

void MultiFilter::predict(double dt) {
    ModelType tmp_type;
    double std_noise_a, std_noise_yaw_a;
    // if(sqrt(x_cv_(2)*x_cv_(2) + x_cv_(3)*x_cv_(3)) > highspeed) {
    // 	std_noise_a = std_noise_a_ / 2;
    // 	std_noise_yaw_a = std_noise_yaw_a_ / 2;
    // }
    // else {
    // 	std_noise_a = std_noise_a_;
    // 	std_noise_yaw_a = std_noise_yaw_a_;
    // }
    std_noise_a = std_noise_a_;
    std_noise_yaw_a = std_noise_yaw_a_;

    // below is ctrv predict
    // State transiction matrix  assumes a  constant turn rate and velocity(CTRV) motion with dt dependancy
    /*	                      v*sin(w*dt + theta)/w - v*sin(theta)/w + x(t)
    state(t+dt) = -v*cos(w*dt + theta)/w + v*cos(theta)/w + y(t)                   when w != 0
                                                            v
                                                            w*dt + theta
                                                            w

                                                      v*cos(theta)*dt + x(t)
    state(t+dt) = v*sin(theta)*dt + y(t)                       when w == 0
                                                      v
                                                      w*dt +theta
                                                      w

    */
   if (maintain_ai_result_ == 0) {
        double xpos, ypos, v, theta, w;
        // double highspeed = 10;

        xpos = x_ctrv_(0);
        ypos = x_ctrv_(1);
        v = x_ctrv_(2);
        theta = x_ctrv_(3);
        w = x_ctrv_(4);

        if (fabs(w) > 0.0001) {
            x_ctrv_(0) = xpos + (v / w) * (sin(w * dt + theta) - sin(theta));
            x_ctrv_(1) = ypos + (v / w) * (-cos(w * dt + theta) + cos(theta));
            x_ctrv_(2) = v;
            x_ctrv_(3) = w * dt + theta;
            x_ctrv_(4) = w;
            x_ctrv_(3) = control_psi(x_ctrv_(3));

            v = x_ctrv_(2);
            theta = x_ctrv_(3);
            w = x_ctrv_(4);

            j_(0, 2) = (-sin(theta) + sin(dt * w + theta)) / w;
            j_(0, 3) = v * (-cos(theta) + cos(dt * w + theta)) / w;
            j_(0, 4) = dt * v * cos(dt * w + theta) / w - v * (-sin(theta) + sin(dt * w + theta)) / (w * w);
            j_(1, 2) = (cos(theta) - cos(dt * w + theta)) / w;
            j_(1, 3) = v * (-sin(theta) + sin(dt * w + theta)) / w;
            j_(1, 4) = dt * v * sin(dt * w + theta) / w - v * (cos(theta) - cos(dt * w + theta)) / (w * w);
            j_(3, 4) = dt;
        } else {
            x_ctrv_(0) = xpos + v * cos(theta) * dt;
            x_ctrv_(1) = ypos + v * sin(theta) * dt;
            x_ctrv_(2) = v;
            x_ctrv_(3) = w * dt + theta;
            x_ctrv_(4) = w;
            x_ctrv_(3) = control_psi(x_ctrv_(3));

            v = x_ctrv_(2);
            theta = x_ctrv_(3);

            j_(0, 2) = dt * cos(theta);
            j_(0, 3) = -dt * v * sin(theta);
            j_(1, 2) = dt * sin(theta);
            j_(1, 3) = dt * v * cos(theta);
            j_(3, 4) = dt;
        }

        // Process covariance matrix assumes to have some acceleration noise q_
        Eigen::MatrixXd G_ctrv = Eigen::MatrixXd::Zero(5, 2);
        Eigen::MatrixXd Qv_ctrv = Eigen::MatrixXd::Zero(2, 2);
        G_ctrv(0, 0) = 0.5 * dt * dt * cos(theta);
        G_ctrv(1, 0) = 0.5 * dt * dt * sin(theta);
        G_ctrv(2, 0) = dt;
        G_ctrv(3, 1) = 0.5 * dt * dt;
        G_ctrv(4, 1) = dt;
        Qv_ctrv(0, 0) = std_noise_a * std_noise_a;
        Qv_ctrv(1, 1) = std_noise_yaw_a * std_noise_yaw_a;
        q_ctrv_ = G_ctrv * Qv_ctrv * G_ctrv.transpose();
        p_ctrv_ = j_ * p_ctrv_ * j_.transpose() + q_ctrv_;
    }

    // below is cv predict
    Eigen::MatrixXd G_cv = Eigen::MatrixXd::Zero(4, 2);
    Eigen::MatrixXd Qv_cv = Eigen::MatrixXd::Zero(2, 2);
    G_cv(0, 0) = 0.5 * dt * dt;
    G_cv(1, 1) = 0.5 * dt * dt;
    G_cv(2, 0) = dt;
    G_cv(3, 1) = dt;
    Qv_cv(0, 0) = std_noise_a * std_noise_a;
    Qv_cv(1, 1) = std_noise_a * std_noise_a;
    q_cv_ = G_cv * Qv_cv * G_cv.transpose();
    f_(0, 2) = dt;
    f_(1, 3) = dt;
    x_cv_ = f_ * x_cv_;
    p_cv_ = f_ * p_cv_ * f_.transpose() + q_cv_;

    // save corner pts
    tmp_type = CTRV;
    syc(tmp_type);
    filtered_obj_->compute_bndbox_points(box_pts_ctrv_);
    tmp_type = CV;
    syc(tmp_type);
    filtered_obj_->compute_bndbox_points(box_pts_cv_);
}

AssociateInfo MultiFilter::score(const Object::Ptr &obj, double scale) {
    std::vector<AssociateInfo> distance;
    AssociateInfo retval, tmp;
    std::pair<int, int> cr_idx, cv_corner_idx;
    double cvvel;
    double dx, dy, difheading;
    double cv_dcenter, cv_dcorner, cv_dcorner_x, cv_dcorner_y;
    bool headingjudge, target_cluster, meas_cluster;

    if (params_.respective_match && filtered_obj_->status != RoadType::ROAD) {
        if ((meas_type_ == ObjectType::UNKNOW && obj->type != ObjectType::UNKNOW) ||
            (obj->status == RoadType::ROAD)) {
            retval.score = -1;
            return retval;
        }
    }
    cvvel = sqrt(x_cv_(2) * x_cv_(2) + x_cv_(3) * x_cv_(3));
    cv_dcenter = 99;
    // distance limitation
    tmp.model = CTRV;
    dx = obj->center(0) - x_ctrv_(0);
    dy = obj->center(1) - x_ctrv_(1);
    tmp.score = sqrt(dx * dx + dy * dy);
    tmp.res_type = CENTER;
    distance.emplace_back(tmp);

    // cr_idx = corner_registration(box_pts_ctrv_, obj->box_pts);
    // dx = obj->box_pts[cr_idx.second](0) - box_pts_ctrv_[cr_idx.first](0);
    // dy = obj->box_pts[cr_idx.second](1) - box_pts_ctrv_[cr_idx.first](1);
    // tmp.score = sqrt(dx * dx + dy * dy);
    // tmp.res_type = CORNER;
    // distance.emplace_back(tmp);

    tmp.model = CV;
    if (type_ == ObjectType::UNKNOW && state_ == TRACK_STATE_INIT &&
        (obj->type == ObjectType::TRUCK || obj->type == ObjectType::BUS || obj->type == ObjectType::CAR)) {
        //pass center association;
    }
    else {
        dx = obj->center(0) - x_cv_(0);
        dy = obj->center(1) - x_cv_(1);
        tmp.score = sqrt(dx * dx + dy * dy);
        tmp.res_type = CENTER;
        cv_dcenter = tmp.score;
        distance.emplace_back(tmp);
    }

    cr_idx = corner_registration(box_pts_cv_, obj->box_pts);
    dx = obj->box_pts[cr_idx.second](0) - box_pts_cv_[cr_idx.first](0);
    dy = obj->box_pts[cr_idx.second](1) - box_pts_cv_[cr_idx.first](1);
    tmp.score = sqrt(dx * dx + dy * dy);
    tmp.res_type = CORNER;
    cv_dcorner = tmp.score;
    cv_dcorner_x = dx;
    cv_dcorner_y = dy;
    cv_corner_idx = cr_idx;
    distance.emplace_back(tmp);

    tmp.score = FLT_MAX;
    for (size_t i = 1; i < distance.size(); ++i) {
        if (distance[i].score < tmp.score) {
            tmp = distance[i];
        }
    }

    retval = tmp;
    retval.cv_dcenter = cv_dcenter;
    retval.cv_dcorner = cv_dcorner;
    retval.cv_dcorner_x = cv_dcorner_x;
    retval.cv_dcorner_y = cv_dcorner_y;
    retval.cv_corner_idx = cv_corner_idx;

    thredist_ = params_.thredist * scale;
    if (heartbeat_ > 3) {
        if (rb2ai_counter_ != 0) {
            thredist_ = 2.0 * scale;
        }
        else if (fabs(cvvel) > 1.5) {
            if ((type_ == ObjectType::TRUCK || type_ == ObjectType::BUS) &&
                (obj->type == ObjectType::TRUCK || obj->type == ObjectType::BUS)) {
                thredist_ = 1.7 * scale;
            }
            else if (maintain_ai_result_ != 0) {
                thredist_ = 1.7 * scale;
            }
            else {
                if (type_ != ObjectType::UNKNOW) {
                    thredist_ = 1.3 * scale;
                }
                else {
                    thredist_ = 1.7 * scale;
                }
            }
        }
        else {
            thredist_ = 0.7 * scale;
        }
    }
    thredist_ = (thredist_ < params_.thredistmin) ? params_.thredistmin : thredist_;
    thredist_ = (thredist_ > params_.thredistmax) ? params_.thredistmax : thredist_;
    if (params_.respective_match && filtered_obj_->status != RoadType::ROAD) {
        thredist_ = params_.thredist * scale;
    }

    // heading limitation
    if ((type_ == ObjectType::PED && obj->type == ObjectType::PED)
        || (type_ == ObjectType::UNKNOW || obj->type == ObjectType::UNKNOW)) {
        headingjudge = true;
    }
    else if (params_.respective_match && obj->type == ObjectType::UNKNOW && filtered_obj_->status != RoadType::ROAD) {
        headingjudge = true;
        thredist_ = params_.thredist * scale;
    }
    else {
        difheading = control_psi(x_ctrv_(3) - obj->heading);
        if (fabs(difheading) > PI / 2) {
            difheading = (difheading < 0) ? (difheading + PI) : (difheading - PI);
        }
        if (fabs(difheading) < params_.threheading) {
            headingjudge = true;
        } else {
            headingjudge = false;
        }
    }
    // if (retval.score < 5 && (obj->type == ObjectType::TRUCK || obj->type == ObjectType::BUS)) {
    //     std::cout << "tracker id: " << id_ << ", score and thre: " << retval.score << ", " << thredist_ << ", heading: " << x_ctrv_(3) << ", " << obj->heading << ", motion model type: " << model_ << ", corner dif: " << retval.cv_dcorner <<std::endl;
    // }

    if (retval.score > thredist_ || headingjudge == false) {
        retval.score = -1;
    }

    return retval;
}

void MultiFilter::update_rb(const Object::Ptr &obj) {
    filtered_obj_->clone(*obj);
    type_ = obj->type;
}

void MultiFilter::update(const Object::Ptr &obj, double timestamp) {
    Eigen::Vector3d measurement;
    double cvvel, diffdeading, ori_meas_heading;
    bool reset_size = false;
    bool reset_heading = false;
    bool flip_heading = false;
    bool rb_to_ai = false;

    diffdeading = 0;
    if (filtered_obj_->status != RoadType::ROAD || obj->status != RoadType::ROAD) {
        reset_size = true;
    } else if (type_ == ObjectType::UNKNOW && obj->type != ObjectType::UNKNOW) {
        reset_size = true;
    } else {
        reset_size = false;
    }
    if (rb2ai_counter_ > 0) {
        if (rb2ai_counter_ < 3) {
            rb2ai_counter_++;
        }
        else {
            rb2ai_counter_ = 0;
        }
    }
    model_ = asso_info_.model;
    if (obj->type == ObjectType::UNKNOW && type_ != ObjectType::UNKNOW && maintain_ai_result_ < params_.maintainAItimes) {
        maintain_ai_result_++;
    }
    else {
        maintain_ai_result_ = 0;
        if (type_ == ObjectType::UNKNOW && obj->type != ObjectType::UNKNOW) {
            rb_to_ai = true;
            rb2ai_counter_ = 1;
        }
        type_ = obj->type;
        filtered_obj_->type_confidence = obj->type_confidence;
        filtered_obj_->exist_confidence = obj->exist_confidence;
    }
    

    filtered_obj_->polygons = obj->polygons;
    filtered_obj_->center(2) = obj->center(2);
    filtered_obj_->object_state = obj->object_state;
    filtered_obj_->noise_state_obj = obj->noise_state_obj;
    filtered_obj_->denoise_obj_weight = obj->denoise_obj_weight ;
    filtered_obj_->denoise_grid_weight= obj->denoise_grid_weight;
    filtered_obj_->denoise_state_frame = obj->denoise_state_frame;
    filtered_obj_->noise_history = obj->noise_history;
    measurement(0) = obj->center(0);
    measurement(1) = obj->center(1);
    measurement(2) = control_psi(obj->heading);

    // below is CTRV update
    if (maintain_ai_result_ == 0) {
        ori_meas_heading = measurement(2);
        cvvel = sqrt(x_cv_(2) * x_cv_(2) + x_cv_(3) * x_cv_(3));
        meas_correction(cvvel, measurement, reset_size);
        // Eigen::MatrixXd r = myBboxNum * myBboxNum * boxlength * r_;
        Eigen::MatrixXd r_ctrv = r_ctrv_;
        Eigen::MatrixXd Ht = h_ctrv_.transpose();
        Eigen::MatrixXd PHt = p_ctrv_ * Ht;
        Eigen::MatrixXd y = measurement - h_ctrv_ * x_ctrv_;
        Eigen::MatrixXd S = h_ctrv_ * PHt + r_ctrv;
        Eigen::MatrixXd K = PHt * S.inverse();

        // new estimate
        y(2) = control_psi(y(2));
        x_ctrv_ = x_ctrv_ + (K * y);
        x_ctrv_(3) = control_psi(x_ctrv_(3));
        int x_size = x_ctrv_.size();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
        p_ctrv_ = (I - K * h_ctrv_) * p_ctrv_;
        diffdeading = fabs(y(2));
        // break ctrv, must use measurement heading
        if (fabs(x_ctrv_(4)) > 0.8 || diffdeading > params_.threheading || rb_to_ai) {
            double vp;
            reset_heading = true;
            x_ctrv_ << measurement(0), measurement(1), cvvel, ori_meas_heading, 0;
            vp = (p_cv_(2, 2) > p_cv_(3, 3)) ? (p_cv_(2, 2)) : (p_cv_(3, 3));
            p_ctrv_ << 1, 0, 0, 0, 0,
                0, 1, 0, 0, 0,
                0, 0, vp, 0, 0,
                0, 0, 0, 0.2, 0,
                0, 0, 0, 0, 0.4;
        }
        // flip heading
        if (x_ctrv_(2) < -2.8) {
            x_ctrv_(3) = (x_ctrv_(3) < 0) ? (x_ctrv_(3) + PI) : (x_ctrv_(3) - PI);
            x_ctrv_(2) *= -1;
            flip_heading = true;
        }
    }
    // reset cv vel cov
    if (rb_to_ai) {
        p_cv_(2, 2) = 100;
        p_cv_(3, 3) = 100;
    }
    // below is cv update
    Eigen::VectorXd meascv;
    Eigen::MatrixXd r_cv = r_cv_;
    meascv = Eigen::VectorXd::Zero(2);
    meascv << measurement(0), measurement(1);
    Eigen::MatrixXd Htcv = h_cv_.transpose();
    Eigen::MatrixXd PHtcv = p_cv_ * Htcv;
    Eigen::VectorXd ycv = meascv - h_cv_ * x_cv_;
    if (asso_info_.cv_dcenter > asso_info_.cv_dcorner) {
        ycv(0) = asso_info_.cv_dcorner_x;
        ycv(1) = asso_info_.cv_dcorner_y;
    }
    Eigen::MatrixXd Scv = h_cv_ * PHtcv + r_cv;
    Eigen::MatrixXd Kcv = PHtcv * Scv.inverse();

    // new estimate
    x_cv_ = x_cv_ + (Kcv * ycv);
    // if(fabs(cvvel) > 20) {
    // 	std::cout << "kalman id : " << id_ << std::endl;
    // 	std::cout << std::fixed << "residual : " << ycv(0) << ", "<<ycv(1)<<std::endl;
    // 	std::cout << std::fixed << "corner info : " << asso_info_.cv_dcenter << ", "<<asso_info_.cv_dcorner<<std::endl;
    // 	std::cout << std::fixed << "after : " << x_cv_(0) << ", "<<x_cv_(1)<<", "<<x_cv_(2)<<", "<<x_cv_(3)<<std::endl;
    // }
    int xcv_size = x_cv_.size();

    Eigen::MatrixXd Icv = Eigen::MatrixXd::Identity(xcv_size, xcv_size);
    p_cv_ = (Icv - Kcv * h_cv_) * p_cv_;

    // update bounding box
    Eigen::Vector3d meas_size;
    meas_size = obj->size;
    if (asso_info_.cv_dcenter > asso_info_.cv_dcorner && flip_heading == false) {
        std::vector<Eigen::Vector3d> before_box(4), after_box(4);
        Eigen::Vector3d tmp_dif(ycv(0), ycv(1), 0);
        for (size_t i = 0; i < 4; i++) {
            before_box[i] = box_pts_cv_[i] + tmp_dif;
        }
        if (maintain_ai_result_ > 0) {
            // dont upate size
        }
        else if (reset_size || diffdeading > params_.threheading) {
            box_size_ = meas_size;
        }
        else if ((meas_size(0) > box_size_(0) || meas_size(1) > box_size_(1))) {
            if (obj->is_ai_refine == AiRefine_State::AIREFINE) {
                box_size_ = 0.8 * box_size_ + 0.2 * meas_size;
            }
            else {
                box_size_ = meas_size;
            }
        }
        else {
            box_size_ = 0.8 * box_size_ + 0.2 * meas_size;
        }
        filtered_obj_->size = box_size_;
        ModelType tmp_model = CV;
        syc(tmp_model);
        filtered_obj_->compute_bndbox_points(after_box);
        if (reset_heading) {
            x_cv_(0) = x_cv_(0) - (after_box[asso_info_.cv_corner_idx.second](0) - before_box[asso_info_.cv_corner_idx.first](0));
            x_cv_(1) = x_cv_(1) - (after_box[asso_info_.cv_corner_idx.second](1) - before_box[asso_info_.cv_corner_idx.first](1));
        }
        else {
            x_cv_(0) = x_cv_(0) - (after_box[asso_info_.cv_corner_idx.first](0) - before_box[asso_info_.cv_corner_idx.first](0));
            x_cv_(1) = x_cv_(1) - (after_box[asso_info_.cv_corner_idx.first](1) - before_box[asso_info_.cv_corner_idx.first](1));
        }
    } else {
        if (maintain_ai_result_ > 0) {
            // dont upate size
        }
        else if (reset_size || diffdeading > params_.threheading) {
            box_size_ = meas_size;
        }
        else if ((meas_size(0) > box_size_(0) || meas_size(1) > box_size_(1))) {
            if (obj->is_ai_refine == AiRefine_State::AIREFINE) {
                box_size_ = 0.8 * box_size_ + 0.2 * meas_size;
            }
            else {
                box_size_ = meas_size;
            }
        }
        else {
            box_size_ = 0.8 * box_size_ + 0.2 * meas_size;
        }
        filtered_obj_->size = box_size_;
    }

    // check velocity is abnormal or not
    // check_velocity(measurement, timestamp);
}

void MultiFilter::event() { // Eigen::Isometry3d &pose
    uint16_t thre;

    if (myBboxNum_ > 0 && heartbeat_ < 2000) {
        heartbeat_++;
    }

    switch (state_) {
    case TRACK_STATE_INIT:
        if (myBboxNum_) {
            // Hit Event
            detect2freeCount_ = 0;
            detect2activeCount_++;
            thre = params_.det2actthre;
            if (detect2activeCount_ > thre) {
                state_ = TRACK_STATE_VISIBLE;
            }
        } else {
            // Miss
            detect2freeCount_++;
            if (detect2activeCount_ > 0) {
                detect2activeCount_--;
            }
            if (detect2freeCount_ > params_.det2freethre) {
                state_ = TRACK_STATE_DELETE;
            }
        }
        break;
    case TRACK_STATE_VISIBLE:
        if (myBboxNum_) {
            // Hit Event
            active2freeCount_ = 0;
        } else {
            // Miss
            active2freeCount_++;
            if (active2freeCount_ > (params_.active2freethre - 1) || type_ == ObjectType::UNKNOW)
                state_ = TRACK_STATE_INVISIBLE;
        }
        break;
    case TRACK_STATE_INVISIBLE:
        if (myBboxNum_) {
            // Hit Event
            active2freeCount_ = 0;
            state_ = TRACK_STATE_VISIBLE;
        } else {
            // Miss
            active2freeCount_++;
            double cvvel = sqrt(x_cv_(2) * x_cv_(2) + x_cv_(3) * x_cv_(3));
            if (fabs(cvvel) < 0.4) {
                thre = 5;
            } else {
                thre = params_.active2freethre;
            }
            if (active2freeCount_ > thre) {
                state_ = TRACK_STATE_DELETE;
            }
        }
        break;
    default:
        break;
    }
}

std::pair<int, int> MultiFilter::corner_registration(std::vector<Eigen::Vector3d> &predpts, std::vector<Eigen::Vector3d> measpts) {
    std::vector<std::pair<double, int>> dif_boxes, dif_pts;
    std::pair<double, int> dif_box, min_box, dif_pt, min_pt;
    int idx, cornersize;
    std::pair<int, int> retval(-1, -1); // first is kalman box index, second is measurement box indx

    cornersize = 4;
    for (int i = 0; i < cornersize; ++i) {
        dif_box.first = 0;
        for (int j = 0; j < cornersize; ++j) {
            idx = (i + j) % cornersize;
            dif_pt.first = pow(predpts[j](0) - measpts[idx](0), 2) + pow(predpts[j](1) - measpts[idx](1), 2);
            dif_pt.second = j;
            dif_box.first += dif_pt.first;
            dif_pts.emplace_back(dif_pt);
        }
        dif_box.second = i;
        dif_boxes.emplace_back(dif_box);
    }
    min_box.first = FLT_MAX;
    for (auto &dif : dif_boxes) {
        if (min_box.first > dif.first) {
            min_box.first = dif.first;
            min_box.second = dif.second;
        }
    }
    min_pt.first = FLT_MAX;
    for (int j = min_box.second * cornersize; j < (min_box.second + 1) * cornersize; j++) {
        if (min_pt.first > dif_pts[j].first) {
            min_pt.first = dif_pts[j].first;
            min_pt.second = dif_pts[j].second;
        }
    }
    idx = (min_box.second + min_pt.second) % cornersize;
    retval.first = min_pt.second; // kalman box index
    retval.second = idx;          // measurement box indx

    return retval;
}

bool isBoxIntersection(Object::Ptr& measure_obj, Object::Ptr& tracker_obj) {
    //compute measure box
    const auto &center_measure = measure_obj->center;
    const auto &size_measure = measure_obj->size;
    const auto &directionr_measure = measure_obj->direction;
    RotateBox box_measure(center_measure, size_measure, directionr_measure);
    std::vector<Eigen::Vector3d> corners_measure;
    box_measure.corners(corners_measure);

    //compute tracker box
    const auto &center_tracker = tracker_obj->center;
    const auto &size_tracker = tracker_obj->size;
    const auto &directionr_tracker = tracker_obj->direction;
    RotateBox box_tracker(center_tracker, size_tracker, directionr_tracker);
    std::vector<Eigen::Vector3d> corners_tracker;
    box_tracker.corners(corners_tracker);

    //intersection or not
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

bool MultiFilter::overlapfilter(std::vector<MultiFilter> &targets) {
    double dx, dy, dist;
    
	for(auto &target : targets) {
		if (id_ == target.id_) {
			continue;
		}
        if (target.state_ == TRACK_STATE_DELETE) {
            continue;
        }
        dx = filtered_obj_->center(0) - target.filtered_obj_->center(0);
        dy = filtered_obj_->center(1) - target.filtered_obj_->center(1);
        dist = sqrt(dx*dx + dy*dy);
        if (dist > 8) {
            continue;
        }
        if (isBoxIntersection(filtered_obj_, target.filtered_obj_)) {
            if (heartbeat_ > 3 && target.state_ == TRACK_STATE_INIT && state_ == TRACK_STATE_DELETE) {
                int tmp_id;
                tmp_id = id_;
                id_ = target.id_;
                target.id_ = tmp_id;
                target.x_cv_(2) = x_cv_(2);
                target.x_cv_(3) = x_cv_(3);
                target.filtered_obj_->tracker_id = target.id_;
                target.filtered_obj_->velocity(0) = target.x_cv_(2);
                target.filtered_obj_->velocity(1) = target.x_cv_(3);
                target.state_ = TRACK_STATE_VISIBLE;
            }
            else if (heartbeat_ > 3 && target.state_ == TRACK_STATE_INIT) {
                target.state_ = TRACK_STATE_DELETE;
            }
            else {
                state_ = TRACK_STATE_DELETE;
            }
        }
	}

	return false;
}

void MultiFilter::meas_correction(double cvvel, Eigen::Vector3d &measurement, bool reset_size) {
    double cvheading, cvheadingtemp, mearheadingtemp, cvdiff, difheading;
    double threheading = PI / 9;

    // modify heading
    if (cvvel > 5 && reset_size == false) { // FLAGS_velthr_meas_heading
        // cvdiff = sqrt(pow(x_cv_(0) - measurement(0), 2) + pow(x_cv_(1) - measurement(1), 2));
        if (/*cvdiff < thredist_*/ true) {
            cvheading = atan2(x_cv_(3), x_cv_(2));
            if (fabs(control_psi(x_ctrv_(3) - measurement(2))) > PI / 2) {
                mearheadingtemp = (measurement(2) < 0) ? (measurement(2) + PI) : (measurement(2) - PI);
            } else {
                mearheadingtemp = measurement(2);
            }

            if (fabs(control_psi(x_ctrv_(3) - cvheading)) > PI / 2) {
                cvheadingtemp = (cvheading < 0) ? (cvheading + PI) : (cvheading - PI);
            } else {
                cvheadingtemp = cvheading;
            }

            if (fabs(control_psi(cvheadingtemp - x_ctrv_(3))) < fabs(control_psi(mearheadingtemp - x_ctrv_(3)))) {
                double tmp_dif;
                tmp_dif = fabs(control_psi(cvheadingtemp - mearheadingtemp));
                if (tmp_dif > threheading / 2) {
                    measurement(2) = cvheading;
                }
            }
        }
    }
    difheading = control_psi(x_ctrv_(3) - measurement(2));
    if (fabs(difheading) > PI / 2) {
        measurement(2) = (measurement(2) < 0) ? (measurement(2) + PI) : (measurement(2) - PI);
    }
}

void MultiFilter::syc(ModelType model) {
    if (params_.respective_match && meas_type_ == ObjectType::UNKNOW && filtered_obj_->status != RoadType::ROAD) {
        return;
    }
    if (model == CTRV) {
        filtered_obj_->center(0) = x_ctrv_(0);
        filtered_obj_->center(1) = x_ctrv_(1);
    } else {
        filtered_obj_->center(0) = x_cv_(0);
        filtered_obj_->center(1) = x_cv_(1);
    }
    filtered_obj_->heading = x_ctrv_(3);
    filtered_obj_->direction << cosf(filtered_obj_->heading), sinf(filtered_obj_->heading), 0.0;
}

void MultiFilter::syc_result() {
    if (params_.respective_match && meas_type_ == ObjectType::UNKNOW && filtered_obj_->status != RoadType::ROAD) {
        if (myBboxNum_ == 0) {
            filtered_obj_->type_confidence = 0;
        }
        else {
            filtered_obj_->type_confidence = -1;
        }
        filtered_obj_->tracker_id = id_;
        return;
    }
    if (model_ == CTRV) {
        filtered_obj_->center(0) = x_ctrv_(0);
        filtered_obj_->center(1) = x_ctrv_(1);
    } else {
        filtered_obj_->center(0) = x_cv_(0);
        filtered_obj_->center(1) = x_cv_(1);
    }
    filtered_obj_->heading = x_ctrv_(3);
    filtered_obj_->velocity(0) = x_cv_(2);
    filtered_obj_->velocity(1) = x_cv_(3);
    if (myBboxNum_ == 0) {
        filtered_obj_->type_confidence = 0;
    } else if (meas_type_ == ObjectType::UNKNOW) {
        filtered_obj_->type_confidence = -1;
    }
    filtered_obj_->tracker_id = id_;
    filtered_obj_->type = type_;
    filtered_obj_->direction << cosf(filtered_obj_->heading), sinf(filtered_obj_->heading), 0.0;
    auto buffer_size = target_buffer_.size();
    int thre = 5;
    if (buffer_size >= thre) {
        filtered_obj_->acceleration(0) = (target_buffer_[buffer_size-1]->velocity(0) - target_buffer_[buffer_size-thre]->velocity(0)) / (0.1*(thre -1));
        filtered_obj_->acceleration(1) = (target_buffer_[buffer_size-1]->velocity(1) - target_buffer_[buffer_size-thre]->velocity(1)) / (0.1*(thre -1));
    }
    else {
        filtered_obj_->acceleration(0) = (target_buffer_[buffer_size-1]->velocity(0) - target_buffer_[0]->velocity(0)) / (0.1*(buffer_size -1));
        filtered_obj_->acceleration(1) = (target_buffer_[buffer_size-1]->velocity(1) - target_buffer_[0]->velocity(1)) / (0.1*(buffer_size -1));
    }
}

} // namespace robosense