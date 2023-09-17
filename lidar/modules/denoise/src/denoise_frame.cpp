#include "denoise_frame.h"

namespace robosense {
void DenoiseFrame::init(const Rs_YAMLReader &configParse) {
    params_ = configParse.getValue<DenoiseParam>("denoise");
    denoise_enable_ = params_.enable;
    if(!denoise_enable_){
        return;
    }
}
bool DenoiseFrame::isInRange(const LidarFrameMsg::Ptr &msg_ptr, double x, double y) {
    return (x > msg_ptr->denoise_grid.xmin && x < msg_ptr->denoise_grid.xmax &&
            y > msg_ptr->denoise_grid.ymin && y < msg_ptr->denoise_grid.ymax);
}

double DenoiseFrame::calcWeightGrid(int grid_num, int noise_count, int obj_count, double gridsingle_weight) {
    double grid_weight = 0.0;
    if (grid_num < 1) {
        grid_weight = 0.0;
    } else if (grid_num == 1) {
        if (noise_count > 0) {
            grid_weight = -0.3;
        } else if (obj_count > 0) {
            grid_weight = 0.3;
        } else {
            grid_weight = 0.0;
        }
    } else {
        grid_weight = 1.0 * (noise_count - obj_count) / grid_num;
    }
    return grid_weight;
}
bool DenoiseFrame::isWeightInvalid(double weight){
    if(fabs(weight)<0.01){
        return true;
    }else{
        return false;
    }
}
bool DenoiseFrame::isSameSign(double a, double b) {
    if ((a > 0 && b > 0) || (a < 0 && b < 0)) {
        return true;
    } else {
        return false;
    }
}
NoiseState DenoiseFrame::weightedSum(double grid_weight,double obj_weight, double invalid_threshold, double valid_threshold){
    double threshold = 0.0;
    if(isWeightInvalid(grid_weight) || isWeightInvalid(obj_weight)){
        threshold = invalid_threshold;
    }else{
        threshold = valid_threshold;
        if(!isSameSign(grid_weight,obj_weight)){
            return NoiseState::NOISE_SUSPECTED;
        }
    }
    double sum_weight = grid_weight+obj_weight;
    if(sum_weight<-threshold){
        return NoiseState::NOISE_OBJECT;
    }else if(sum_weight>threshold){
        return NoiseState::NOISE_NOISE;
    }else{
        return NoiseState::NOISE_SUSPECTED;
    }
}
void DenoiseFrame::calcLabelFromGridObj(const LidarFrameMsg::Ptr &msg_ptr) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto &cloud_ptr = msg_ptr->scan_ptr;
    auto &obj_refine = msg_ptr->objects_refine;
    auto &erase_objs = msg_ptr->refine_data.erase_objs;
    for (auto i = 0; i<obj_refine.size();i++){
        auto &obj = obj_refine[i];
        if(obj->type != ObjectType::UNKNOW){
            continue;
        }
        if (std::find(erase_objs.begin(), erase_objs.end(), i) != erase_objs.end()){
            continue;
        }

        double x = obj->center(0);
        double y = obj->center(1);
        if (!isInRange(msg_ptr, x, y)) {
            continue;
        }
        ROS_DEBUG_STREAM(HDENOISE_G<<"calcLabelFromGridObj--center x:"<<x<<"  y:"<<y<<" xsize:"<<obj->size(0)<<" ysize:"<<obj->size(1));
        std::map<int,bool> searched_map;//(obj->cloud_indices.size(),false);
        std::vector<int> noise_obj_count(2,0);
        for(auto cii:obj->cloud_indices){
            auto &pt = cloud_ptr->points.at(cii);
            if (!std::isfinite (pt.x) ||
                !std::isfinite (pt.y) ||
                !std::isfinite (pt.z)){
                continue;
            }
            if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
                continue;
            }
            if (!isInRange(msg_ptr, pt.x, pt.y)) {
                continue;
            }
            int x_id = static_cast<int>((pt.x - msg_ptr->denoise_grid.xmin) / msg_ptr->denoise_grid.unit_size);
            int y_id = static_cast<int>((pt.y - msg_ptr->denoise_grid.ymin) / msg_ptr->denoise_grid.unit_size);
            int grid_idx = x_id * msg_ptr->denoise_grid.ysize + y_id;
            if (searched_map.find(grid_idx)==searched_map.end()){
                searched_map[grid_idx] = true;
                if(msg_ptr->denoise_grid.noise_type[grid_idx]==NoiseType::NOISE){
                    noise_obj_count[0]++;
                    ROS_DEBUG_STREAM(HDENOISE_G<<"        noise xid:"<<x_id<<" yid:"<<y_id);
                }else if(msg_ptr->denoise_grid.noise_type[grid_idx]==NoiseType::OBJECT){
                    noise_obj_count[1]++;
                    ROS_DEBUG_STREAM(HDENOISE_G<<"        obj xid:"<<x_id<<" yid:"<<y_id);
                }
            }
        }
        int grid_num = noise_obj_count[0]+noise_obj_count[1];
        double grid_weight = 0.0;
        double gridsingle_weight = 0.3;
        grid_weight = calcWeightGrid(grid_num,noise_obj_count[0],noise_obj_count[1],gridsingle_weight);
        obj->denoise_grid_weight = grid_weight;
        double obj_weight = obj->denoise_obj_weight;
        // std::cout<<"x:"<<x<<", y:"<<y<<std::endl;
        // std::cout<<"grid_weight:"<<grid_weight<<", obj_weight:"<<obj_weight<<std::endl;
        double invalid_threshold = 0.7;
        double valid_threshold = 0.7;
        obj->object_state.noise_state = weightedSum(grid_weight,obj_weight, invalid_threshold, valid_threshold);
        obj->denoise_state_frame = obj->object_state.noise_state;

        // float grid_min_noise_ratio = 0.70;
        // float grid_min_suspected_ratio = 0.30;
        // if(grid_num<1){
        //     obj->object_state.noise_state = NoiseState::NOISE_SUSPECTED;//H:TODO: suspected or object？
        // }else if(noise_obj_count[0]>grid_min_noise_ratio*grid_num){
        //     obj->object_state.noise_state = NoiseState::NOISE_NOISE;
        // }else if(noise_obj_count[0]>grid_min_suspected_ratio*grid_num){
        //     obj->object_state.noise_state = NoiseState::NOISE_SUSPECTED;
        // }else{
        //     obj->object_state.noise_state = NoiseState::NOISE_OBJECT;
        // }
        ROS_DEBUG_STREAM("        grid_num:"<<grid_num);
        ROS_DEBUG_STREAM("        noise_obj_count[0]:"<<noise_obj_count[0]);
        ROS_DEBUG_STREAM("        obj->object_state.noise_state:"<<kNoiseState2NameMap.at(obj->object_state.noise_state));
    }
}

bool DenoiseFrame::isRatioEnoughInHistory(boost::circular_buffer<Object::Ptr> list, int near_history_num, float detect_min_ratio, std::function<bool(Object::Ptr)> condition,bool is_countInvisible) {
    int num = 0;
    int condition_num = 0;
    // const std::map<TrackerState,std::string> state2string = {
    //     {TrackerState::VISIBLE,"V "},
    //     {TrackerState::INVISIBLE,"I "}
    // };
    // std::cout<<H_DEBUG_R<<"trackid:"<<list.back()->tracker_id<<"  ";
    // for (auto tbi:list){
    //     std::cout<<state2string.at(tbi->state);
    // }
    // std::cout<<std::endl;
    // std::cout<<"history num:"<<list.size()<<std::endl;
    for (auto it = list.rbegin(); it != list.rend(); ++it) {
        auto li = *it;
        if(!is_countInvisible && (li->useful==false || li->type!=ObjectType::UNKNOW)){
            break;
        }
        num++;
        if (condition(li)) {
            condition_num++;
        }
    }
    if (num < near_history_num && condition_num < 2) {
        return false;
    }
    return condition_num > (detect_min_ratio * num);
}
// void DenoiseFrame::buildMapMeasureTracker(const LidarFrameMsg::Ptr &msg_ptr, std::vector<int> &erase_objs) {
//     auto &objs_measure = msg_ptr->objects_refine;
//     auto &objs_tracker = msg_ptr->objects;
//     map_measure_trackers_.clear();
//     map_tracker_measures_.clear();
//     for (size_t i = 0; i < objs_measure.size(); i++) {
//         if (std::find(erase_objs.begin(), erase_objs.end(), i) != erase_objs.end()){
//             continue;
//         }
//         for (size_t j = 0; j < objs_tracker.size(); j++) {
//             if ((objs_measure[i]->type == ObjectType::UNKNOW && objs_tracker[j]->type == ObjectType::UNKNOW) && isBoxIntersection(objs_measure[i], objs_tracker[j])) {
//                 map_measure_trackers_[i].push_back(j);
//                 map_tracker_measures_[j].push_back(i);
//             }
//         }
//     }
    
//     // std::cout<<"map_measure_trackers_ setted : " << std::endl;
//     // for(auto &mi: map_measure_trackers_){
//     //     std::cout << "key: "<< mi.first << ", size: " << mi.second.size() << std::endl;
//     // }
// }
void DenoiseFrame::setDenoiseHistory(Object::Ptr &obj_ri, boost::circular_buffer<Object::Ptr> &list) {
    std::vector<NoiseState> noise_history_reverse;
    noise_history_reverse.push_back(obj_ri->denoise_state_frame);
    for (auto it = list.rbegin(); it != list.rend(); ++it) {
        auto li = *it;
        // if(!is_countInvisible &&it != list.rbegin() && li->state==TrackerState::INVISIBLE){
        if(li->useful==false || li->type!=ObjectType::UNKNOW){
            break;
        }
        noise_history_reverse.push_back(li->denoise_state_frame);
    }
    auto &noise_history = obj_ri->noise_history;
    noise_history.clear();
    noise_history.reserve(noise_history_reverse.size());
    for (auto it = noise_history_reverse.rbegin(); it != noise_history_reverse.rend(); ++it) {
        noise_history.push_back(*it);
    }
}
bool DenoiseFrame::print_obj(const Object::Ptr &ori, std::string name,double x_min,double x_max,double y_min,double y_max) {
    std::cout<<name << std::endl;
    // double xmin_xmax_ymin_ymax[4] = {660705.9045513305,660707.8725730907,2965611.759377761,2965614.8675846444};
    bool is_print = true;
    // auto &ori = target.filtered_obj_;
    if(ori->type == ObjectType::UNKNOW){
        for(auto &rpi:ori->polygons){
            auto &rb_polygon_x = rpi[0];
            auto &rb_polygon_y = rpi[1];
            auto &rb_polygon_z = rpi[2];
            // double x_min = xmin_xmax_ymin_ymax[0]-1;
            // double x_max = xmin_xmax_ymin_ymax[1]+1;
            // double y_min = xmin_xmax_ymin_ymax[2]-1;
            // double y_max = xmin_xmax_ymin_ymax[3]+1;
            if(rb_polygon_x>x_max || rb_polygon_x<x_min || rb_polygon_y>y_max || rb_polygon_y<y_min){
                is_print = false;
            }
            // std::cout<< "   polygon.xyz: "<<rb_polygon_x <<","<< rb_polygon_y <<","<< rb_polygon_z <<";";
        }
        std::cout<< std::endl;
        if (is_print){
            std::cout<< "   polygon.xyz: "<<std::endl;
            for(auto &rpi:ori->polygons){
                auto &rb_polygon_x = rpi[0];
                auto &rb_polygon_y = rpi[1];
                auto &rb_polygon_z = rpi[2];
                std::cout<<std::setprecision(18)<<rb_polygon_x <<","<< rb_polygon_y <<","<< rb_polygon_z <<";";
            }
            std::cout<<std::endl;
            std::cout<<"Select:" << "ori->noise_state_obj:"<< kNoiseState2NameMap.at(ori->noise_state_obj)<< std::endl;
        }else{
            std::cout<<"NoSelect:" << "ori->noise_state_obj:"<< kNoiseState2NameMap.at(ori->noise_state_obj)<< std::endl;
        }
    }
    return is_print;
}
NoiseState DenoiseFrame::calclabelFromTracker(const int trk_id, std::vector<boost::circular_buffer<Object::Ptr>> &obj_trk, Object::Ptr obj_ri) {
    int near_history_num = 2;
    float detect_min_noise_ratio = 0.75;
    float detect_min_obj_ratio = 0.75;
    bool is_countInvisible = false;
    // int trk_id = trk_rb_ids.at(0);
    if(trk_id>=obj_trk.size()-1){
        // obj_ri->object_state.noise_state = NoiseState::NOISE_SUSPECTED;
        return NoiseState::NOISE_SUSPECTED;
    }
    auto &current_tracker = obj_trk.at(trk_id);
    setDenoiseHistory(obj_ri, current_tracker);

    switch(obj_ri->denoise_state_frame){
        case NoiseState::NOISE_OBJECT:
            // std::cout<<">>    NOISE_OBJECT" <<"  "<<H_DEBUG_R<< std::endl;
            if (isRatioEnoughInHistory(current_tracker, near_history_num, detect_min_obj_ratio,
                            [](Object::Ptr li) -> bool { return li->denoise_state_frame == NoiseState::NOISE_OBJECT; }, is_countInvisible)) {
                // std::cout<<"# >>    NOISE_OBJECT" <<"  "<<H_DEBUG_R<< std::endl;
                return NoiseState::NOISE_OBJECT;
            }else{
                // std::cout<<"#/ >>    NOISE_SUSPECTED" <<"  "<<H_DEBUG_R<< std::endl;
                return NoiseState::NOISE_SUSPECTED;
            }
            break;
        case NoiseState::NOISE_NOISE:
            // std::cout<<">>    NOISE_NOISE" <<"  "<<H_DEBUG_R<< std::endl;
            if (isRatioEnoughInHistory(current_tracker, near_history_num, detect_min_noise_ratio,
                            [](Object::Ptr li) -> bool { return li->denoise_state_frame == NoiseState::NOISE_NOISE; }, is_countInvisible)) {
                // std::cout<<"# >>    NOISE_NOISE" <<"  "<<H_DEBUG_R<< std::endl;
                return NoiseState::NOISE_NOISE;
            }else{
                // std::cout<<"# >>    NOISE_SUSPECTED" <<"  "<<H_DEBUG_R<< std::endl;
                return NoiseState::NOISE_SUSPECTED;
            }
            break;
        default:
            // std::cout<<">>    NOISE_SUSPECTED" <<"  "<<H_DEBUG_R<< std::endl;
            return NoiseState::NOISE_SUSPECTED;
    }
}
NoiseState DenoiseFrame::multiTrkObjState( std::vector<NoiseState> &noise_states){
    int noise_cnt = 0;
    int obj_cnt = 0;
    int all_cnt = 0;
    for (auto &nsi:noise_states){
        if(nsi == NoiseState::NOISE_NOISE){
            noise_cnt++;
        }else if(nsi == NoiseState::NOISE_OBJECT){
            obj_cnt++;
        }
        all_cnt++;
    }
    if(noise_cnt==all_cnt){
        return NoiseState::NOISE_NOISE;
    }else if(obj_cnt==all_cnt){
        return NoiseState::NOISE_OBJECT;
    }else{
        return NoiseState::NOISE_SUSPECTED;
    }
}
void DenoiseFrame::showObjCorners(Object::Ptr& obj) {
    const auto &center = obj->center;
    const auto &size = obj->size;
    const auto &direction = obj->direction;
    RotateBox box(center, size, direction);
    std::vector<Eigen::Vector3d> corners;
    box.corners(corners);
    for (size_t i = 0; i < 4; i++) {
        std::cout << "        corner " << i << ": " << corners[i].transpose() << std::endl;
    }
}
void DenoiseFrame::setLabelFromHistory(const LidarFrameMsg::Ptr &msg_ptr) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::map<int, std::vector<int>> &measure_to_trackers = msg_ptr->refine_data.measure_to_trackers;
    std::map<int, std::vector<int>> &tracker_to_measures = msg_ptr->refine_data.tracker_to_measures;                                            
    auto &erase_objs = msg_ptr->refine_data.erase_objs;
    auto &obj_refine = msg_ptr->objects_refine;
    auto &obj_trk = msg_ptr->objects_history;
    auto &obj_last = msg_ptr->objects;
    msg_ptr->objects_denoise_del.clear();
    for(size_t i = 0; i<obj_refine.size(); i++){
        if (std::find(erase_objs.begin(), erase_objs.end(), i) != erase_objs.end()){
            continue;
        }
        auto &obj_ri = obj_refine.at(i);
        if(obj_ri->type != ObjectType::UNKNOW){
            continue;
        }
        if (measure_to_trackers.find(i) == measure_to_trackers.end()) {
            obj_ri->object_state.noise_state = NoiseState::NOISE_SUSPECTED;
            continue;
        }
        std::vector<int> &trk_ids = measure_to_trackers[i];
        std::vector<int> trk_rb_ids;
        trk_rb_ids.reserve(trk_ids.size());
        for(size_t tii = 0; tii<trk_ids.size(); tii++){
            int trk_i = trk_ids.at(tii);
            auto &obj_ti = obj_trk.at(trk_i).back();
            if(obj_ti->type == ObjectType::UNKNOW){
               trk_rb_ids.emplace_back(trk_i);
            }
        }
        if(trk_rb_ids.size()==0){
            obj_ri->object_state.noise_state = NoiseState::NOISE_SUSPECTED;
            continue;
        } else if(trk_rb_ids.size()==1){
            int trk_id = trk_rb_ids.at(0);
            NoiseState noise_state = calclabelFromTracker(trk_id,obj_trk,obj_ri);
            // std::cout<<"trk_rb_ids.size()==1    "<<"trk_id:"<<trk_id<<"  noise_state:"<<kNoiseState2NameMap.at(noise_state)<<std::endl;
            obj_ri->object_state.noise_state = noise_state;
            if(noise_state == NoiseState::NOISE_NOISE){
                erase_objs.push_back(i);//to del obj idx
                Object::Ptr obj_ptr(new Object);
                obj_ptr->clone(*obj_ri);
                msg_ptr->objects_denoise_del.push_back(obj_ptr);
            }

        }else{
            std::vector<NoiseState> noise_states;
            noise_states.reserve(trk_rb_ids.size());
            for (size_t trk_ii = 0; trk_ii<trk_rb_ids.size(); trk_ii++){
                auto &trk_id = trk_rb_ids.at(trk_ii);
                NoiseState noise_state_cur = calclabelFromTracker(trk_id,obj_trk,obj_ri);
                // std::cout<<trk_ii<<" - trk_rb_ids.size()>1    "<<"trk_id:"<<trk_id<<"  noise_state:"<<kNoiseState2NameMap.at(noise_state_cur)<<std::endl;
                noise_states.push_back(noise_state_cur);
            }
            NoiseState noise_state;
            noise_state = multiTrkObjState(noise_states);
            // std::cout<<"Result: trk_rb_ids.size()>1    "<<"  noise_state:"<<kNoiseState2NameMap.at(noise_state)<<std::endl;
            obj_ri->object_state.noise_state = noise_state;
            if(noise_state == NoiseState::NOISE_NOISE){
                erase_objs.push_back(i);//to del obj idx
                Object::Ptr obj_ptr(new Object);
                obj_ptr->clone(*obj_ri);
                msg_ptr->objects_denoise_del.push_back(obj_ptr);
            }
        }
    }
}

void printTrackid(const LidarFrameMsg::Ptr &msg_ptr) {
    int rii = 0;
    for(auto &ri:msg_ptr->objects_refine){
        rii++;
        // std::cout << "msg_ptr->objects_refine trackid ("<<rii<<"): "<<ri->tracker_id << std::endl;
    }
    int i = 0;
    for(auto &current_tracker:msg_ptr->objects_history){
        i++;
        int j = 0;
        for(auto &ci:current_tracker){
            j++;
            // std::cout << "objects_history trackid ("<<i<<", "<<j<<"): "<<current_tracker.back()->tracker_id << std::endl;
        }
    }
}

void setAllNoise(const LidarFrameMsg::Ptr &msg_ptr) {
    auto &obj_refine = msg_ptr->objects_refine;
    for (auto i = 0; i<obj_refine.size();i++){
        auto &obj = obj_refine[i];
        if(obj->type != ObjectType::UNKNOW){
            continue;
        }
        obj->noise_state_obj = NoiseState::NOISE_NOISE;
    }
}
void DenoiseFrame::recalcSuspectedbyWeight(const LidarFrameMsg::Ptr &msg_ptr) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto &cloud_ptr = msg_ptr->scan_ptr;
    auto &obj_refine = msg_ptr->objects_refine;
    auto &erase_objs = msg_ptr->refine_data.erase_objs;
    for (auto i = 0; i<obj_refine.size();i++){
        auto &obj = obj_refine.at(i);
        if(obj->type != ObjectType::UNKNOW){
            continue;
        }
        if (std::find(erase_objs.begin(), erase_objs.end(), i) != erase_objs.end()){
            continue;
        }
        double x = obj->center(0);
        double y = obj->center(1);
        if (!isInRange(msg_ptr, x, y)) {
            continue;
        }
        if(obj->object_state.noise_state != NoiseState::NOISE_SUSPECTED){
            continue;
        }
        double grid_weight = obj->denoise_grid_weight;
        double obj_weight = obj->denoise_obj_weight;
        double invalid_threshold = 0.5;
        double valid_threshold = 0.7;
        obj->object_state.noise_state = weightedSum(grid_weight,obj_weight, invalid_threshold, valid_threshold);
        obj->denoise_state_frame = obj->object_state.noise_state;
        if(obj->object_state.noise_state == NoiseState::NOISE_NOISE){
            erase_objs.push_back(i);//to del obj idx
            Object::Ptr obj_ptr(new Object);
            obj_ptr->clone(*obj);
            msg_ptr->objects_denoise_del.push_back(obj_ptr);
        }
    }
}
void DenoiseFrame::perception(const LidarFrameMsg::Ptr &msg_ptr) {
    TRY_CATCH
    {
        // printTrackid(msg_ptr);
        calcLabelFromGridObj(msg_ptr);
        // buildMapMeasureTracker(msg_ptr, erase_objs);
        setLabelFromHistory(msg_ptr);
        
        recalcSuspectedbyWeight(msg_ptr);
    }
    END_TRY_CATCH
}


} // namespace robosense

