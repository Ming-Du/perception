#pragma once 

#include "rs_perception_manager.h"

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include "autopilot_msgs/ros_proto.h"
#include "common/proto/mogo_point_cloud.pb.h"
#include "common/proto/trigger_info.pb.h"
#include <pcl/filters/voxel_grid.h>

namespace robosense{

class LidarPerception {
public:
    using Ptr = std::shared_ptr<LidarPerception>;

    LidarPerception();
    ~LidarPerception();

    void perception();

private:
    void init();
    void asyncSpinner();
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &pts_msg);
    void pointCloudmapCallback(const sensor_msgs::PointCloud2ConstPtr &pts_msg);
    void groundmapCallback(const autopilot_msgs::BinaryDataConstPtr &msg);
    void updataGNSSCallback(const autopilot_msgs::BinaryDataConstPtr &msg);
    void rainModeCallback(const std_msgs::Int32::ConstPtr &msg);

    perception::TrackedObjects parserObjFromMsg(const LidarFrameMsg::Ptr &msg_ptr);
    localization::Localization interpolateNewLocalization(const std::vector<localization::Localization> &local, double &stamp);

    bool updateLidarToImu(const double &timestamp, localization::Localization &localization);
    void convertObjectBaseGlobalAxis(const Object::Ptr &obj_ptr, const double &ts, perception::TrackedObject *obj);
    void convertObjectBaseVehicleAxis(const Object::Ptr &obj_ptr, const double &ts, perception::TrackedObject *obj);
    void pointsAndMsgPubEvent(const ros::TimerEvent &);
    void ProtoPub(LidarFrameMsg::Ptr lidar_msg_ptr, localization::Localization local_current);
    void parserGroundFromMsg(LidarFrameMsg::Ptr lidar_msg_ptr, localization::Localization local_current);
    void parserTriggerFromMsg(const LidarFrameMsg::Ptr msg_ptr);
private:
    ros::NodeHandle private_nh_;
    ros::NodeHandlePtr nh_ptr;

    ros::Subscriber sub_gnss_;
    ros::Subscriber sub_pointcloud_;
    ros::Subscriber sub_ground_map_;
    ros::Subscriber sub_rain_mode_;

    ros::Timer pub_msg_timer_;

    ros::Publisher pub_objects_;
    ros::Publisher pub_msg_app_;
    ros::Publisher pub_points_app_;
    ros::Publisher pub_msg_cloud_;
    ProtoPublisher<rule_segement::MogoPointCloud> proto_msg_publisher_;
    ros::Publisher pub_ground_;
    ros::Publisher pub_trigger_;
    ProtoPublisher<ground_map::GroundMap> proto_ground_publisher_;
    ProtoPublisher<trigger_info::TriggerInfo> proto_trigger_publisher_;
    LidarPerceptionManager::Ptr lidar_perception_manager_ptr_;
    ground_map::GroundMap falcon_ground;
    std::list<localization::Localization> global_localizations_;
    std::list<localization::Localization> used_localizations_;

    bool is_send_pointcloud = false;
    bool is_process_semanticmap = false;
    bool enable_trigger = false;
    std::mutex mutex_;
    std::mutex local_mutex_;

    bool exit_flag_;
    LidarFrameMsg::Ptr msg_ptr_;
    localization::Localization local_current_;

    // param
    std::string input_cloud_topic_;
    std::string output_objects_topic_;

    int rain_mode_flag_ = -1;
    bool is_parser_map_info_ = false;
    bool is_lock_localization_ = false;
    bool is_inter_localization = false;
    bool is_showTime_local_lidar = false;
    bool rule_segment_enable = false;
    float downsampling_ratio = 0.5;
    double last_local_stamp_ = 0;
    std::string sensor_name_;

};

typedef std::shared_ptr<LidarPerception> LidarPerceptionPtr;
typedef std::shared_ptr<const LidarPerception> LidarPerceptionConstPtr;

}   // namespace robosense

