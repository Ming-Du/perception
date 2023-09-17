#include "tracking.h"

#include "common/include/tic_toc.h"

namespace robosense {

void Tracking::init(const Rs_YAMLReader &configParse) {
    params_ = configParse.getValue<TrackerParam>("tracker");
}

void Tracking::perception(const LidarFrameMsg::Ptr &msg_ptr) {
    msg_ptr->transAxis(AxisType::GLOBAL_AXIS);
    preprocess(msg_ptr);
    {
        // TicToc timer("jhlTracking");
        associate();
        allocate();
        update(msg_ptr->timestamp); // uptate kalman and obj states
    }
    // postprocess();
    report(msg_ptr);
}

void Tracking::preprocess(const LidarFrameMsg::Ptr &msg_ptr) {
    input_objs_ = msg_ptr->objects_refine;
    for (auto & obj : input_objs_) {
        if (obj->size(0) < obj->size(1)) {
            double tmp_size;
            obj->heading = control_psi(obj->heading + (3.1415926 / 2));
            obj->direction << cosf(obj->heading), sinf(obj->heading), 0.0;
            tmp_size = obj->size(0);
            obj->size(0) = obj->size(1);
            obj->size(1) = tmp_size;
        }
        obj->compute_bndbox_points(obj->box_pts);
    }
 
    bestInd_.clear();
    bestInd_.resize(input_objs_.size(), -1);
    // TODO: check in perception manager
    if (fabs(dt_) > 2) {
        targets_.clear();
    }
}

void Tracking::predict(const LidarFrameMsg::Ptr &msg_ptr) {
    TicToc timer("perception/tracker/predict");
    double timestamp = msg_ptr->timestamp;
    auto &obj_vec = msg_ptr->objects;

    dt_ = timestamp - timestamp_;
    timestamp_ = timestamp;

    Eigen::Matrix4d transform_mat;
    msg_ptr->transGlobal2VehicleMat(transform_mat);

    for (auto &target : targets_) {
        if (target.meas_type_ == ObjectType::UNKNOW && params_.respective_match &&
            target.filtered_obj_->status != RoadType::ROAD) {
            // pass
        }
        else {
            target.predict(dt_);
        }
        target.filtered_obj_->polygons.clear();
        // trans result for Refiner module
        target.syc_result();
        target.myBboxNum_ = 0;

        // transform history object predict result from global to vehicle;
        // this value has be used in Refiner module
        Object::Ptr temp_obj(new Object);
        temp_obj->clone(*target.filtered_obj_);
        temp_obj->tracker_id = target.id_;
        temp_obj->transform(transform_mat);

        obj_vec.push_back(std::move(temp_obj));
        
        msg_ptr->objects_history.push_back(target.target_buffer_);
    }
}

void Tracking::associate() {
    AssociateInfo result;
    std::vector<AssociateInfo> bestScore(input_objs_.size());
    int bboxID, myBboxNum;
    double scoretemp, scorefinal;
    double scale;

    bboxID = -1;
    scale = dt_ / 0.1;

    for (auto &target : targets_) {
        for (size_t i = 0; i < input_objs_.size(); ++i) {
            result = target.score(input_objs_[i], scale);
            if (result.score < 0) {
                continue;
            }
            if (result.score < bestScore[i].score) {
                bestInd_[i] = target.id_;
                bestScore[i] = result;
            }
        }
    }
    double area_tracker, area_measure, area_ratio;
    double tmp_ratio = 0;
    for (auto &target : targets_) {
        myBboxNum = 0;
        scorefinal = FLT_MAX;
        bboxID = -1;
        area_tracker = target.box_size_(0) * target.box_size_(1);
        for (size_t i = 0; i < input_objs_.size(); ++i) {
            area_measure = input_objs_[i]->size(0) * input_objs_[i]->size(1);
            area_ratio = (area_tracker < area_measure) ? (area_tracker/area_measure) : (area_measure/area_tracker);
            if (bestInd_[i] == target.id_) {
                scoretemp = bestScore[i].score;
                if (scorefinal == FLT_MAX) {
                    scorefinal = scoretemp;
                    bboxID = i;
                    myBboxNum = 1;
                    tmp_ratio = area_ratio;
                } else {
                    if (scoretemp < scorefinal && (area_ratio > 0.5 || area_ratio > tmp_ratio)) {
                        bestInd_[bboxID] = -1;
                        scorefinal = scoretemp;
                        bboxID = i;
                        tmp_ratio = area_ratio;
                    } else {
                        bestInd_[i] = -1;
                    }
                    myBboxNum++;
                }
            }
        }
        target.myBboxNum_ = myBboxNum;
        if (myBboxNum) {
            target.meas_id_ = bboxID;
            target.model_ = bestScore[bboxID].model;
            target.asso_info_ = bestScore[bboxID];
        }
    }
}

void Tracking::allocate() {
    int newNum = 0;

    for (size_t i = 0; i < input_objs_.size(); ++i) {
        if (bestInd_[i] == -1) {
            MultiFilter newtarget;
            newtarget.init(input_objs_[i], id_, params_);
            newtarget.meas_id_ = i;
            newNum++;
            targets_.push_back(newtarget);
            bestInd_[i] = id_;
            idlist_[id_] = true;
            get_new_id();
        }
    }
}

void Tracking::update(double timestamp) {
    double x, y, theta;
    std::vector<int> erase_object;
    // Object::Ptr input_obj(new Object);
    Object::Ptr placehold_obj(new Object);
    bool overlap;
    placehold_obj->useful = false;
    
    for (size_t i = 0; i < targets_.size(); ++i) {
        auto &target = targets_[i];
        if (target.myBboxNum_) {
            // input_obj = input_objs_[target.meas_id_];
            target.meas_type_ = input_objs_[target.meas_id_]->type;
            target.filtered_obj_->status = input_objs_[target.meas_id_]->status;
            if (target.meas_type_ == ObjectType::UNKNOW && params_.respective_match &&
                target.filtered_obj_->status != RoadType::ROAD) {
                target.update_rb(input_objs_[target.meas_id_]);
            }
            else {
                target.update(input_objs_[target.meas_id_], timestamp);
            }
            // target.meas_buffer_.push_back(input_obj);
            target.target_buffer_.push_back(std::make_shared<Object>(*target.filtered_obj_));
        }
        else {
            // target.meas_buffer_.push_back(placehold_obj);
            // target.target_buffer_.push_back(placehold_obj);
            Object::Ptr temp(new Object);
            temp->clone(*target.filtered_obj_);
            temp->useful = false;
            target.target_buffer_.push_back(temp);
        }
        target.event();
        target.syc_result();
    }
    // remove overlap obj
    for (size_t i = 0; i < targets_.size(); ++i) {
        auto &target = targets_[i];
        overlap = false;
        if (target.myBboxNum_ == 0) {
            overlap = target.overlapfilter(targets_);
        }
        if (target.state_ == TRACK_STATE_INIT) {
            overlap = target.overlapfilter(targets_);
        }
    }
    // delete tracker
    for (size_t i = 0; i < targets_.size(); ++i) {
        if (targets_[i].state_ == TRACK_STATE_DELETE) {
            erase_object.push_back(i);
            idlist_[targets_[i].id_] = false;
        }
    }
    if (erase_object.size() != 0) {
        reverse(erase_object.begin(), erase_object.end());
        for (const auto &i : erase_object) {
            targets_.erase(targets_.begin() + i);
        }
    }
}

void Tracking::report(const LidarFrameMsg::Ptr &msg_ptr) {
    auto &obj_vec = msg_ptr->objects;

    Eigen::Matrix4d transform_mat;
    msg_ptr->transGlobal2VehicleMat(transform_mat);

    obj_vec.clear();
    for (auto &target : targets_) {
        if (target.state_ == TRACK_STATE_VISIBLE) {
            // double cvvel = sqrt(tmp->velocity(0) * tmp->velocity(0) + tmp->velocity(1) * tmp->velocity(1));
            // if(fabs(cvvel) > 15){
            //     std::cout << "tracker is: " << target.id_ << std::endl;
            //     for(auto& tmp1 : target.target_buffer_) {
            //         std::cout << std::fixed << "xy: " << tmp1->useful << ", " << tmp1->center(0)<<", "<<tmp1->center(1)<< ", "<< tmp1->velocity(0)<<", "<<tmp1->velocity(1) <<std::endl;
            //     }
            //     std::cout << "measure is" << std::endl;
            //     for(auto& tmp1 : target.meas_buffer_) {
            //         std::cout << std::fixed << "xy: " << tmp1->useful << ", " << tmp1->center(0)<<", "<<tmp1->center(1)<<std::endl;
            //     }
            // }
            Object::Ptr temp(new Object);
            temp->clone(*target.filtered_obj_);
            obj_vec.push_back(temp);

            Object::Ptr temp_obj(new Object);
            temp_obj->clone(*target.filtered_obj_);
            temp_obj->transform(transform_mat);
            msg_ptr->objects_vehicle.push_back(temp_obj);
        }
    }
    msg_ptr->transAxis(AxisType::VEHICLE_AXIS);
}

void Tracking::get_new_id() {
    while (1) {
        id_++;
        id_ = id_ % SIZE;
        if (idlist_[id_] == false) {
            break;
        }
    }
}


} // namespace robosense
