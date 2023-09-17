#include "trigger_info_pub.h"

namespace robosense {

TriggerManager::TriggerManager()
{

}

void TriggerManager::init(const Rs_YAMLReader &configParse){
    
}

void TriggerManager::perception(const LidarFrameMsg::Ptr &msg_ptr) {
    auto &trigger_vec = msg_ptr->trigger_vec;
    auto &trigger_info = msg_ptr->trigger_info;
    trigger_info.clear();
    auto &tmp_del_noise = msg_ptr->objects_denoise_del;
    //1. noise check
    trigger_vec[0] = check_noise(tmp_del_noise.size());
    if(trigger_vec[0] != TriggerType::NORMAL){
        trigger_info.push_back("Lidar");
        trigger_info.push_back("noise");
    }
    //2. yaw check
    trigger_vec[1] = check_yaw(msg_ptr);
    if(trigger_vec[1] != TriggerType::NORMAL){
        trigger_info.push_back("Object");
        trigger_info.push_back("unstable yaw");
    }

    //3. traffic check
    trigger_vec[2] = check_traffic(msg_ptr);
    if(trigger_vec[2] != TriggerType::NORMAL){
        trigger_info.push_back("Traffic");
        trigger_info.push_back("vehicle congestion");
    }

    //4. truck&bus check
    trigger_vec[3] = check_oversize_vehicle(msg_ptr);
    if(trigger_vec[3] != TriggerType::NORMAL){
        trigger_info.push_back("Object");
        trigger_info.push_back("oversize vehicle");
    }
    
}


TriggerType TriggerManager::check_noise(const int size_del_noise){
    if(size_del_noise){
        return TriggerType::ENABLE_NOISE;
    }
    else
        return TriggerType::NORMAL;
}


TriggerType TriggerManager::check_traffic(const LidarFrameMsg::Ptr &msg_ptr){
    auto &obj_vec = msg_ptr->objects;
    int obj_num =0;
    for (size_t i = 0; i < obj_vec.size(); ++i) {
        for (size_t j = i + 1; j < obj_vec.size(); ++j) {
            float distance =  std::sqrt(std::pow(obj_vec[i]->center.x()- obj_vec[j]->center.x(), 2) + std::pow(obj_vec[i]->center.y() - obj_vec[j]->center.x(), 2));;
            if (distance < distanceThreshold) {
                obj_num++;
            }
        }
    }

    if(obj_num > traffic_thre){
        return TriggerType::ENABLE_TRAFFIC;
    }
    else
        return TriggerType::NORMAL;
}

TriggerType TriggerManager::check_oversize_vehicle(const LidarFrameMsg::Ptr &msg_ptr){
    auto &obj_vec = msg_ptr->objects;
    int oversize_obj_num =0;
    for (auto &target : obj_vec) {
       if(target->type == ObjectType::TRUCK || target->type == ObjectType::BUS){
           oversize_obj_num += 1;
       }
    }
    if(oversize_obj_num > oversize_thre){
        return TriggerType::ENABLE_OVERSIZE;
    }
    else
        return TriggerType::NORMAL;
}

double TriggerManager::diff_yaw(const double yaw_a, const double yaw_b){
    double temp_angle;
    if (yaw_a * yaw_b >= 0) {
        temp_angle = fabs(yaw_a - yaw_b);
    } else {
        temp_angle = fabs(yaw_a) + fabs(yaw_b);
        if(temp_angle > M_PI/2 && temp_angle < M_PI){
            temp_angle = fabs(M_PI - temp_angle);
        }
    }
    if (temp_angle > M_PI)
        temp_angle = fabs(M_PI - temp_angle);
    return temp_angle;
}

TriggerType TriggerManager::check_yaw(const LidarFrameMsg::Ptr &msg_ptr){
    auto &objs_history = msg_ptr->objects_history;
    std::vector<int> yaw_candidates;
    for(int idx =0; idx < objs_history.size(); idx++){
        auto recentObj = objs_history[idx].back();
        int history_size = objs_history[idx].size();
        if(recentObj->type == ObjectType::UNKNOW || recentObj->type == ObjectType::PED){
            continue;
        }
        if (objs_history[idx].size() > 5) {
            history_size = 5;
        }
        double yawMean = 0;
        int yawNum = 0;
        double velocityYawMean = 0;
        int velocityYawNum = 0;
        double disYawMean = 0;
        int disYawNum = 0;
        for (int buffer_i = history_size - 1; buffer_i > 0; buffer_i--) {  
            auto &tmp_obj = objs_history[idx].at(buffer_i);
            //if (tmp_obj->type == ObjectType::UNKNOW || tmp_obj->type == ObjectType::PED) 
            //    continue;
            double angle = atan2(tmp_obj->velocity(1), tmp_obj->velocity(0)); 
            double yaw = atan2(tmp_obj->direction(1), tmp_obj->direction(0));
            double dif_yaw_angle = diff_yaw(angle, yaw);
            if(tmp_obj->velocity.norm() > 10 && dif_yaw_angle > M_PI/10 ){
                yawNum += 1;
                yawMean = (yawMean * (yawNum - 1) + dif_yaw_angle) / yawNum;
            }
            auto &his_obj = objs_history[idx].at(buffer_i -1);
            double his_obj_angle = atan2((his_obj->velocity(1)), his_obj->velocity(0));
            double his_obj_yaw = atan2((his_obj->direction(1)), his_obj->direction(0));
            if(tmp_obj->velocity.norm() > 5){
                velocityYawNum += 1;
                velocityYawMean = (velocityYawMean * (velocityYawNum - 1) + diff_yaw(his_obj_angle,angle)) / velocityYawNum;
            }
        }
        if(yawMean > M_PI/10 || velocityYawMean > M_PI/2){
            yaw_candidates.push_back(idx);
        }
    }
    if(yaw_candidates.size() > 0){
        return TriggerType::ENABLE_YAW;
    }else{
        return TriggerType::NORMAL;
    }    


}
     
}
    