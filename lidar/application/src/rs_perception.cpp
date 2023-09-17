#include "rs_perception.h"
#include "common/include/tic_toc.h"
#include "common/include/config/rs_yamlReader.h"

#include <ros/ros.h>
#include <ros/callback_queue.h>

namespace robosense {

bool TicToc::enable_timer_ = false;

LidarPerception::LidarPerception()
    : private_nh_("~")
    , nh_ptr(new ros::NodeHandle)
    , exit_flag_(true)
{
    init();
}

LidarPerception::~LidarPerception(){
    std::lock_guard<std::mutex> lock(mutex_);
    exit_flag_ = true;
}

void LidarPerception::init() {
    std::string perception_vehicle_type, perception_config_path, proj;
    private_nh_.getParam("vehicle_type", perception_vehicle_type);
    if (perception_vehicle_type == "bus") {
        perception_vehicle_type = "leishen";
    }
    private_nh_.getParam("perception_config_path", perception_config_path);
    bool ret = private_nh_.getParam("proj", proj);
    if (!ret) {
        ROS_WARN_STREAM("proj is not set! use default scenes: beijing");
        is_parser_map_info_ = true;
        proj = "beijing";
    }
    ROS_INFO_STREAM("perception_config_path:" << perception_config_path);

    // for local running
    std::string::size_type pos = perception_config_path.find("src");
    if(pos != std::string::npos)
    {
        std::string base_path = perception_config_path.substr(0, pos);   
        perception_config_path = base_path + "install/share/rs_perception_node/config";
        ROS_INFO_STREAM("perception_config_path_new:" << perception_config_path);
    }
    std::string vehicle_config_path = perception_config_path + "/../../config/vehicle/vehicle_config.txt";
    std::string vehicle_type = Rs_YAMLReader::getVehicleType(vehicle_config_path);
    ROS_INFO_STREAM("vehicle_type:" << vehicle_type);

    if ((perception_vehicle_type == "zvision" && vehicle_type == "jinlv") 
        || (perception_vehicle_type == "leishen" && vehicle_type == "m2")) {
        ROS_ERROR("vehicle type and sensor type do not match, please check the configuration file! vehicle type [%s], sensor type [%s]", vehicle_type.c_str(), perception_vehicle_type.c_str());
        exit(0);
    }

    std::string vehicle_type_config_file = std::string("/vehicle_type_config/") + vehicle_type + "/" +  perception_vehicle_type  + "/" + proj + ".perception.yaml";
    Rs_YAMLReader yamlReader(perception_config_path ,vehicle_type_config_file);

    is_lock_localization_ = yamlReader.getValue<bool>("is_lock_localization");
    is_inter_localization = yamlReader.getValue<bool>("is_inter_localization");
    is_showTime_local_lidar = yamlReader.getValue<bool>("is_showTime_local_lidar");
    TicToc::enable_timer_ = yamlReader.getValue<bool>("is_print_timecost");

    sensor_name_ = yamlReader.getValue<std::string>("sensor_name");
    input_cloud_topic_ = yamlReader.getValue<std::string>("input_cloud_topic");
    output_objects_topic_ = yamlReader.getValue<std::string>("output_objects_topic");

    rule_segment_enable = yamlReader.getValue<bool>("rule_segment.enable");
    downsampling_ratio = yamlReader.getValue<float>("rule_segment.downsampling_ratio");

    lidar_perception_manager_ptr_.reset(new LidarPerceptionManager);
    lidar_perception_manager_ptr_->init(yamlReader);
}

// create thread distributor for pointCloudCallback and othersCallback
void LidarPerception::asyncSpinner() {
    ros::SubscribeOptions ops;
    ros::CallbackQueue pointcloud_callback_queue;
    ops.init<sensor_msgs::PointCloud2>(input_cloud_topic_, 1, boost::bind(&LidarPerception::pointCloudCallback, this, _1));
    ops.transport_hints = ros::TransportHints();
    ops.callback_queue = &pointcloud_callback_queue;
    sub_pointcloud_ = nh_ptr->subscribe(ops);

    ros::AsyncSpinner s(1, &pointcloud_callback_queue);
    s.start();

    ros::spin();
}

void LidarPerception::perception() {
    sub_gnss_ = nh_ptr->subscribe("/localization/global", 200, &LidarPerception::updataGNSSCallback, this);
    sub_rain_mode_ = nh_ptr->subscribe("/sensor/rainmode", 1, &LidarPerception::rainModeCallback, this);

    pub_objects_ = nh_ptr->advertise<std_msgs::String>(output_objects_topic_, 1);

    pub_msg_app_ = nh_ptr->advertise<autopilot_msgs::BinaryData>("/perception/lidar/point_cloud_app", 1, true);
    pub_points_app_ = nh_ptr->advertise<pcl::PointCloud<pcl::PointXYZI>>("/perception/lidar/show_app_pointcloud", 1, true);
    pub_msg_cloud_ = nh_ptr->advertise<autopilot_msgs::BinaryData>("/perception/lidar/point_cloud_aicloud", 1, true);

    pub_msg_timer_ = nh_ptr->createTimer(ros::Duration(0.9), &LidarPerception::pointsAndMsgPubEvent, this, false, false);
    pub_trigger_ = nh_ptr->advertise<autopilot_msgs::BinaryData>("/perception/lidar/trigger_info", 1, true);
    asyncSpinner();
}
void LidarPerception::pointsAndMsgPubEvent(const ros::TimerEvent &) {
    int loop_num = 20;
    while (loop_num--) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!exit_flag_ && msg_ptr_->msg_pub_flag)
                break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if(exit_flag_) return;

    static int seq = 0;
    ros::Time ts;
    ts.fromSec(msg_ptr_->timestamp);
    // MogoPointCloud proto header
    rule_segement::MogoPointCloud PointCloudPB_msg, AiCloudPB_msg;
    PointCloudPB_msg.mutable_header()->set_seq(seq++);
    PointCloudPB_msg.mutable_header()->mutable_stamp()->set_sec(ts.sec);
    PointCloudPB_msg.mutable_header()->mutable_stamp()->set_nsec(ts.nsec);
    PointCloudPB_msg.mutable_header()->set_frame_id("base_link");
    // msg_ptr_->frame_id
    PointCloudPB_msg.mutable_header()->set_module_name("rule_segment");
    // PointCloud proto localization
    PointCloudPB_msg.set_self_longitude(local_current_.longitude());
    PointCloudPB_msg.set_self_latitude(local_current_.latitude());
    PointCloudPB_msg.set_self_altitude(local_current_.altitude());
    PointCloudPB_msg.set_self_roll(local_current_.roll());
    PointCloudPB_msg.set_self_pitch(local_current_.pitch());
    PointCloudPB_msg.set_self_yaw(local_current_.yaw());

    AiCloudPB_msg = PointCloudPB_msg;
    pcl::PointCloud<pcl::PointXYZI>::Ptr back_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    lidar_perception_manager_ptr_->getBackgroundCloudpoints(msg_ptr_, back_cloud_ptr);
    lidar_perception_manager_ptr_->BackgroundDownsampling(back_cloud_ptr, downsampling_ratio);
    lidar_perception_manager_ptr_->toProtoMsg(PointCloudPB_msg, back_cloud_ptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pc_ptr->header.frame_id = "base_link";
    int new_size_i;
    new_size_i = PointCloudPB_msg.add_data_size();
    for (int i = 0; i < new_size_i; i += 4) {
        pcl::PointXYZI point;
        point.x = PointCloudPB_msg.add_data(i) / 100.0;
        point.y = PointCloudPB_msg.add_data(i + 1) / 100.0;
        point.z = PointCloudPB_msg.add_data(i + 2) / 100.0;
        point.intensity = PointCloudPB_msg.add_data(i + 3);
        pc_ptr->points.emplace_back(point);
    }
    pub_points_app_.publish(pc_ptr);

    localization::Localization newlocal_current;
    {
        std::lock_guard<std::mutex> lockmap(local_mutex_);
        newlocal_current = global_localizations_.back();
    }

    double newGpsTimestamp = newlocal_current.header().stamp().sec() + newlocal_current.header().stamp().nsec() * 1e-9;

    PointCloudPB_msg.set_newgpstimestamp(newGpsTimestamp);
    proto_msg_publisher_.publish(pub_msg_app_, PointCloudPB_msg);

    lidar_perception_manager_ptr_->BackgroundDownsampling(back_cloud_ptr, downsampling_ratio);
    lidar_perception_manager_ptr_->toProtoMsg(AiCloudPB_msg, back_cloud_ptr);
    AiCloudPB_msg.set_newgpstimestamp(newGpsTimestamp);
    proto_msg_publisher_.publish(pub_msg_cloud_, AiCloudPB_msg);
}

void LidarPerception::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &pts_msg) {
    TRY_CATCH
    {
        TicToc timer("One Frame of Lidar Processing");
        if (pts_msg->data.size() == 0) {
            ROS_INFO_STREAM(" Input cloud point is empty!");
            return;
        }
        PointCloudPtr pointcloud(new PointCloud);
        pcl::fromROSMsg(*pts_msg, *pointcloud);

        double timestamp = pts_msg->header.stamp.toSec();
        ros::Time ts;
        ts.fromSec(timestamp);
        double lidar_current_timestamp = ts.sec + ts.nsec * 1e-9;
        if (!updateLidarToImu(lidar_current_timestamp, local_current_)) {
            ROS_ERROR_STREAM("Fail to get localization for lidar measurement.");
            return;
        }

        LidarFrameMsg::Ptr lidar_msg_ptr(new LidarFrameMsg);
        lidar_msg_ptr->frame_id = "/middle_lidar";
        lidar_msg_ptr->timestamp = timestamp;
        lidar_msg_ptr->sys_timestamp = ros::Time::now().toSec();
        lidar_msg_ptr->scan_ptr = pointcloud;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            lidar_msg_ptr->is_rain_mode = (rain_mode_flag_ == 1) ? true : false;
            
            msg_ptr_ = lidar_msg_ptr;
            exit_flag_ = false;
        }
        if(rule_segment_enable && !pub_msg_timer_.hasStarted()) pub_msg_timer_.start();

        lidar_perception_manager_ptr_->perception(lidar_msg_ptr, local_current_, is_parser_map_info_);
        exit_flag_ = true;
        enable_trigger = (lidar_msg_ptr->trigger_info.size() > 0) ? true : false; 
        std_msgs::String lidar_frame;
        perception::TrackedObjects out_objects = parserObjFromMsg(lidar_msg_ptr);
        out_objects.SerializeToString(&lidar_frame.data);
        pub_objects_.publish(lidar_frame);     
        if(enable_trigger){
            parserTriggerFromMsg(lidar_msg_ptr);
        }
  
    }
    END_TRY_CATCH
}

void LidarPerception::updataGNSSCallback(const autopilot_msgs::BinaryDataConstPtr &msg) {
    std::lock_guard<std::mutex> lock(local_mutex_); 
    while (global_localizations_.size() >= 200) {
        if (is_lock_localization_)
            return;
        global_localizations_.pop_front();
    }
    localization::Localization global_localization;
    RosToProto(*msg, global_localization);

    global_localizations_.emplace_back(global_localization);
}

void LidarPerception::rainModeCallback(const std_msgs::Int32::ConstPtr &msg) {
    if (rain_mode_flag_ != msg->data && msg->data == 1) {
        ROS_INFO("\033[32m =======>> Rain mode opened! <<======= \033[0m");
    } else if (rain_mode_flag_ != msg->data && msg->data == 0) {
        ROS_INFO("\033[32m =======>> Rain mode closed! <<======= \033[0m");
    } else {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_); 
    rain_mode_flag_ = msg->data;
}

bool LidarPerception::updateLidarToImu(const double &timestamp, localization::Localization &localization) {
    double local_stamp = 0;
    double lidar_stamp = timestamp + 0.05; // reduce timestamp by time delay of sensor data transmission and perception consuming.
    bool is_local_valid = false;

    {
        std::lock_guard<std::mutex> lock(local_mutex_);
        if (global_localizations_.empty()) {
            ROS_ERROR_STREAM("Localization message NOT received.");
            return false;
        }
        if (is_lock_localization_) {
            localization = global_localizations_.front();
            return true;
        }

        for (auto it = global_localizations_.rbegin(); it != global_localizations_.rend(); it++) {
            local_stamp = (*it).header().stamp().sec() + (*it).header().stamp().nsec() * 1e-9;
            if (local_stamp > lidar_stamp)
                continue;
            else {
                localization = *it;
                is_local_valid = true;
                break;
            }
        }
    }

    if (!is_local_valid) {
        ROS_ERROR_STREAM("Localization message too new cause invalid.");
        return false;
    }
    if (is_inter_localization) {
        if (used_localizations_.size() > 10)
            used_localizations_.pop_front();
        used_localizations_.emplace_back(localization);
        if (abs(local_stamp - last_local_stamp_) < RS_DBL_EPSILON && used_localizations_.size() > 3) {
            localization = used_localizations_.back();
            used_localizations_.pop_back();
            bool is_stamp_valid = true;
            double last_used_local_stamp = 0;
            std::vector<localization::Localization> interpolated_localization_vec;
            for (auto it = used_localizations_.rbegin(); it != used_localizations_.rend(); it++) {
                double used_local_stamp = (*it).header().stamp().sec() + (*it).header().stamp().nsec() * 1e-9;
                if (abs(used_local_stamp - last_used_local_stamp) < RS_DBL_EPSILON) {
                    is_stamp_valid = false;
                    ROS_WARN_STREAM("The same timestamp from localization is exist, stamp = " << std::setprecision(18) << used_local_stamp);
                    break;
                }
                last_used_local_stamp = used_local_stamp;

                interpolated_localization_vec.emplace_back(*it);
                if (interpolated_localization_vec.size() > 2)
                    break;
            }
            if (is_stamp_valid) {
                ROS_WARN_STREAM("The location info is abnormal, the new location is calculated by interpolation.");
                localization = interpolateNewLocalization(interpolated_localization_vec, lidar_stamp);
            }
            used_localizations_.emplace_back(localization);
        }
        last_local_stamp_ = local_stamp;
    }

    if (is_showTime_local_lidar)
        ROS_INFO_STREAM("Time distance " << std::setprecision(3) << abs(lidar_stamp - local_stamp)
                                         << " between Pointcloud: " << std::setprecision(18) << lidar_stamp
                                         << " and localization: " << std::setprecision(18) << local_stamp
                                         << ", system time: " << std::setprecision(18) << ros::Time::now().toSec());
    if (abs(lidar_stamp - local_stamp) > 0.3) {
        ROS_ERROR_STREAM("Time distance " << std::setprecision(3) << abs(lidar_stamp - local_stamp)
                                          << " between  Pointcloud: " << std::setprecision(18) << lidar_stamp
                                          << " and localization: " << std::setprecision(18) << local_stamp << " is too long."
                                          << " system time: " << std::setprecision(18) << ros::Time::now().toSec());
        return false;
    }
    return true;
}

localization::Localization LidarPerception::interpolateNewLocalization(const std::vector<localization::Localization> &local, double &stamp){
    double local_stamp0 = local.at(0).header().stamp().sec() + local.at(0).header().stamp().nsec() * 1e-9;
    double local_stamp1 = local.at(1).header().stamp().sec() + local.at(1).header().stamp().nsec() * 1e-9;
    double local_stamp2 = local.at(2).header().stamp().sec() + local.at(2).header().stamp().nsec() * 1e-9;

    double k1 = ((stamp - local_stamp1) * (stamp - local_stamp0)) / ((local_stamp2 - local_stamp1) * (local_stamp2 - local_stamp0));
    double k2 = ((stamp - local_stamp2) * (stamp - local_stamp0)) / ((local_stamp1 - local_stamp2) * (local_stamp1 - local_stamp0));
    double k3 = ((stamp - local_stamp2) * (stamp - local_stamp1)) / ((local_stamp0 - local_stamp2) * (local_stamp0 - local_stamp1));

    double new_longitude = k1 * local.at(2).longitude() + k2 * local.at(1).longitude() + k3 * local.at(0).longitude();
    double new_latitude = k1 * local.at(2).latitude() + k2 * local.at(1).latitude() + k3 * local.at(0).latitude();
    double new_position_x = k1 * local.at(2).position().x() + k2 * local.at(1).position().x() + k3 * local.at(0).position().x();
    double new_position_y = k1 * local.at(2).position().y() + k2 * local.at(1).position().y() + k3 * local.at(0).position().y();
    double new_position_z = k1 * local.at(2).position().z() + k2 * local.at(1).position().z() + k3 * local.at(0).position().z();
    double yaw0 = (local.at(0).yaw() > 0 && local.at(0).yaw() < 3) ? local.at(0).yaw() + 360 : local.at(0).yaw();
    double yaw1 = (local.at(1).yaw() > 0 && local.at(1).yaw() < 3) ? local.at(1).yaw() + 360 : local.at(1).yaw();
    double yaw2 = (local.at(2).yaw() > 0 && local.at(2).yaw() < 3) ? local.at(2).yaw() + 360 : local.at(2).yaw();
    double new_yaw = k1 * yaw2 + k2 * yaw1 + k3 * yaw0;
    if (new_yaw > 360) new_yaw -= 360;

    localization::Localization interpolated_localization;
    interpolated_localization.mutable_header()->mutable_stamp()->set_sec(floor(stamp));
    interpolated_localization.mutable_header()->mutable_stamp()->set_nsec(floor((stamp - floor(stamp))*1e9));
    interpolated_localization.set_utm_zone(local.at(0).utm_zone());
    interpolated_localization.set_longitude(new_longitude);
    interpolated_localization.set_latitude(new_latitude);
    interpolated_localization.mutable_position()->set_x(new_position_x);
    interpolated_localization.mutable_position()->set_y(new_position_y);
    interpolated_localization.mutable_position()->set_z(new_position_z);
    interpolated_localization.set_yaw(new_yaw);

    return std::move(interpolated_localization);
}

void LidarPerception::parserGroundFromMsg(const LidarFrameMsg::Ptr msg_ptr, localization::Localization local_current) {
    
    static int seq = 0; 
    ros::Time ts;
    ts.fromSec(msg_ptr->timestamp);
    ground_map::GroundMap ground_msg;
    ground_map::GroundCoeff *ground_coe;
    ground_msg.mutable_header()->set_seq(seq++);
    ground_msg.mutable_header()->mutable_stamp()->set_sec(ts.sec);
    ground_msg.mutable_header()->mutable_stamp()->set_nsec(ts.nsec);
    ground_msg.mutable_header()->set_frame_id("base_link");
    ground_msg.mutable_header()->set_module_name("ground_map");
    ground_msg.set_self_longitude(local_current.longitude());
    ground_msg.set_self_latitude(local_current.latitude());
    ground_msg.set_self_altitude(local_current.altitude());
    ground_msg.set_self_roll(local_current.roll());
    ground_msg.set_self_pitch(local_current.pitch());
    ground_msg.set_self_yaw(local_current.yaw());
    for (size_t i = 0; i < msg_ptr->semantic_map.size(); i++) {
        auto coefficent = msg_ptr->semantic_map[i];
        ground_coe = ground_msg.add_coefficients();
        ground_coe->set_start_ring(coefficent.start_ring);
        ground_coe->set_end_ring(coefficent.end_ring);       
        ground_coe->set_a(coefficent.ground_coe[0]);
        ground_coe->set_b(coefficent.ground_coe[1]);
        ground_coe->set_c(coefficent.ground_coe[2]);
        ground_coe->set_d(coefficent.ground_coe[3]);
        ground_coe->set_normal_x(coefficent.ground_normal[0]);
        ground_coe->set_normal_y(coefficent.ground_normal[1]);
        ground_coe->set_normal_z(coefficent.ground_normal[2]);

    }
    proto_ground_publisher_.publish(pub_ground_ ,ground_msg);
}

void LidarPerception::parserTriggerFromMsg(const LidarFrameMsg::Ptr msg_ptr){
    static int seq = 0; 
    ros::Time ts;
    ts.fromSec(msg_ptr->timestamp);
    ros::Time ts_start;
    ts_start.fromSec(msg_ptr->timestamp - 3.0);
    ros::Time ts_end;
    ts_end.fromSec(msg_ptr->timestamp + 5.0);
    trigger_info::TriggerInfo trigger_msg;
    trigger_info::Label *trigger_label;
    trigger_msg.mutable_header()->set_seq(seq++);
    trigger_msg.mutable_header()->mutable_stamp()->set_sec(ts.sec);
    trigger_msg.mutable_header()->mutable_stamp()->set_nsec(ts.nsec);
    trigger_msg.mutable_header()->set_frame_id("base_link");
    trigger_msg.mutable_header()->set_module_name("Bus-Lidar");
    trigger_msg.mutable_timestamp_start()->set_sec(ts_start.sec);
    trigger_msg.mutable_timestamp_start()->set_nsec(ts_start.nsec);
    trigger_msg.mutable_timestamp_end()->set_sec(ts_end.sec);
    trigger_msg.mutable_timestamp_end()->set_nsec(ts_end.nsec);
    for (size_t i = 0; i < msg_ptr->trigger_info.size(); i=i+2) {
        auto label = msg_ptr->trigger_info[i];
        auto sublabel = msg_ptr->trigger_info[i+1];
        trigger_label = trigger_msg.add_trigger_label();
        trigger_label->set_primary_label(label);
        trigger_label->set_secondary_label(sublabel);       
    }
    trigger_msg.set_case_num(20);
    trigger_msg.set_level(1);
    proto_trigger_publisher_.publish(pub_trigger_ ,trigger_msg);

}

perception::TrackedObjects LidarPerception::parserObjFromMsg(const LidarFrameMsg::Ptr &msg_ptr) {
    perception::TrackedObjects result_objs;
    perception::TrackedObject *obj;

    msg_ptr->transAxis(AxisType::GLOBAL_AXIS);
    std::vector<Object::Ptr> objects = msg_ptr->objects;
    std::vector<Object::Ptr> objects_vehicle = msg_ptr->objects_vehicle;
    static int seq = 0;

    result_objs.mutable_header()->set_seq(seq++);
    ros::Time ts;
    ts.fromSec(msg_ptr->timestamp);
    result_objs.mutable_header()->mutable_stamp()->set_sec(ts.sec);
    result_objs.mutable_header()->mutable_stamp()->set_nsec(ts.nsec);
    result_objs.mutable_header()->set_frame_id("base_link"); // msg_ptr->frame_id
    result_objs.mutable_header()->set_module_name("rs_perception");
    result_objs.set_sensor_name(sensor_name_);

    for (size_t i = 0; i < objects.size(); i++) {
        // if (msg_ptr->denoise_grid.enable && TrackingSetting::params_ptr_->is_delete_in_output) {
        //     if (objects[i]->object_state.noise_state == NoiseState::NOISE_NOISE)
        //         continue;
        // }
        obj = result_objs.add_objs();
        ROS_DEBUG_STREAM(HDENOISE_G << "beforeAxis Object"
                                    << "trkid:" << objects[i]->tracker_id << "noisestate:" << (int)objects[i]->object_state.noise_state << " ToMap:" << kNoiseState2NameMap.at(objects[i]->object_state.noise_state));
        ROS_DEBUG_STREAM(HDENOISE_G << "beforeAxis ObjVhi"
                                    << "trkid:" << objects_vehicle[i]->tracker_id << "noisestate:" << (int)objects_vehicle[i]->object_state.noise_state << " ToMap:" << kNoiseState2NameMap.at(objects_vehicle[i]->object_state.noise_state));
        convertObjectBaseGlobalAxis(objects[i], msg_ptr->timestamp, obj);
        // std::cout << "pub track: " << objects[i]->center(0) << ", " << objects[i]->center(1) << std::endl;
        // std::cout << "pub global: " << obj->obj().center().x() << ", " << obj->obj().center().y() << std::endl;
        convertObjectBaseVehicleAxis(objects_vehicle[i], msg_ptr->timestamp, obj);
        // std::cout << "pub vehicle: " << obj->obj().center().x() << ", " << obj->obj().center().y() << std::endl;
    }
    return result_objs;
}

void LidarPerception::convertObjectBaseGlobalAxis(const Object::Ptr &obj_ptr, const double &ts, perception::TrackedObject *obj) {
    perception::TrackedObject *obj_p = obj; //->mutable_obj();
    double yaw = atan2(obj_ptr->direction(1), obj_ptr->direction(0));
    {
        obj_p->mutable_velocity()->set_x(obj_ptr->velocity(0));
        obj_p->mutable_velocity()->set_y(obj_ptr->velocity(1));
        obj_p->mutable_velocity()->set_z(obj_ptr->velocity(2));
    }
    if ((obj_ptr->velocity.norm() > 1.38 && obj_ptr->type == ObjectType::UNKNOW) 
        || (obj_ptr->velocity.norm() > 5.66 && obj_ptr->type != ObjectType::UNKNOW)) {
        yaw = atan2(obj_ptr->velocity(1), obj_ptr->velocity(0)); // atan2(obj_ptr->direction(1), obj_ptr->direction(0));
    }
    // 0~2*PI->East:0, Counterclockwise
    if (yaw > 2 * M_PI) {
        yaw -= 2 * M_PI;
    } else if (yaw < 0)
        yaw += 2 * M_PI;
    obj->set_longitude_p(obj_ptr->center(0));
    obj->set_latitude_p(obj_ptr->center(1));
    obj->set_yaw(yaw);
}

void LidarPerception::convertObjectBaseVehicleAxis(const Object::Ptr &obj_ptr, const double &ts, perception::TrackedObject *obj) {
    perception::Object *obj_p = obj->mutable_obj();
    double angle = atan2(obj_ptr->direction(1), obj_ptr->direction(0));
    obj_p->set_id(obj_ptr->tracker_id);
    obj_p->set_sensor_name(sensor_name_); //???
    obj_p->set_type(kObjectType2MogoType.at(obj_ptr->type));
    obj_p->set_time_stamp(ts);

    obj_p->set_exist_confidence(obj_ptr->exist_confidence);
    obj_p->set_type_confidence(obj_ptr->type_confidence);
    obj_p->set_detect_state(kDetectState2MogoState.at(obj_ptr->object_state.detect_state));
    obj_p->set_noise_state(kNoiseState2MogoState.at(obj_ptr->object_state.noise_state));
    obj_p->set_group_type(kGroupType2MogoType.at(obj_ptr->group));
    obj_p->set_road_status(kRoadType2MogoType.at(obj_ptr->status));
    ROS_DEBUG_STREAM(HDENOISE_G << "trackid:" << obj->obj().id()
                                << " noisestate:" << (int)obj->obj().noise_state()
                                << " oriNoiseState:" << (int)obj_ptr->object_state.noise_state);
    obj_p->set_angle(angle);
    obj_p->set_x_distance(obj_ptr->center(0));
    obj_p->set_y_distance(obj_ptr->center(1));
    obj_p->mutable_center()->set_x(obj_ptr->center(0));
    obj_p->mutable_center()->set_y(obj_ptr->center(1));
    obj_p->mutable_center()->set_z(obj_ptr->center(2));
    obj_p->mutable_size()->set_x(obj_ptr->size(0));
    obj_p->mutable_size()->set_y(obj_ptr->size(1));
    obj_p->mutable_size()->set_z(obj_ptr->size(2));
    // add polygon
    if (obj_ptr->type == ObjectType::UNKNOW) {
        geometry::Point *contour;
        std::vector<Eigen::Vector3d> tmp_polygon;
        if (obj_ptr->type_confidence == 0) {
            tmp_polygon.resize(4);
            obj_ptr->compute_bndbox_points(tmp_polygon);
        } else {
            tmp_polygon = obj_ptr->polygons;
        }
        for (size_t n = 0; n < tmp_polygon.size(); n++) {
            contour = obj_p->add_contour();
            contour->set_x(tmp_polygon[n][0]);
            contour->set_y(tmp_polygon[n][1]);
            contour->set_z(0);
        }
    } else {
        const auto &center = obj_ptr->center;
        const auto &size = obj_ptr->size;
        const auto &direction = obj_ptr->direction;
        RotateBox box(center, size, direction);

        std::vector<Eigen::Vector3d> corners;
        box.corners(corners);

        geometry::Point *contour;
        for (int i = 0; i < 4; ++i) {
            contour = obj_p->add_contour();
            contour->set_x(corners[i].x());
            contour->set_y(corners[i].y());
            contour->set_z(0);
        }
    }
    obj_p->mutable_velocity()->set_x(obj_ptr->velocity(0));
    obj_p->mutable_velocity()->set_y(obj_ptr->velocity(1));
    obj_p->mutable_velocity()->set_z(obj_ptr->velocity(2));
    
    obj_p->mutable_acceleration()->set_x(obj_ptr->acceleration(0));
    obj_p->mutable_acceleration()->set_y(obj_ptr->acceleration(1));
    obj_p->mutable_acceleration()->set_z(0);
    
    obj->set_yaw_rate(obj_ptr->angle_velocity);
}

} // namespace robosense
