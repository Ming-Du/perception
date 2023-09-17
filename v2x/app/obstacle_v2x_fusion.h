/**
@file: obstacle_v2x_fusion.h
@brief: collect bsm and rsm messages
 **/

#pragma once

#include "autopilot_msgs/ros_proto.h"
#include "common/proto/BSM_PB.pb.h"
#include "common/proto/BasicSafetyMessage_PB.pb.h"
#include "common/proto/RSI_PB.pb.h"
#include "common/proto/RSI_RTE_PB.pb.h"
#include "common/proto/RSM_PB.pb.h"
#include "common/proto/RSM_PNT_PB.pb.h"
#include "common/proto/SSM_PB.pb.h"
#include "common/proto/SSM_PNT_PB.pb.h"
#include "common/proto/localization.pb.h"
#include "common/proto/object.pb.h"
#include "common/proto/obuObject.pb.h"
#include "perception/base/sensor_manager/sensor_manager.h"
#include "proto/v2x_fusion_component.pb.h"
#include "common/include/log.h"
#include "common/include/pb_utils.h"
#include "perception/v2x/common/self_veh_filter/self_veh_filter.h"
#include  "perception/v2x/common/rviz_display/v2x_display.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <list>
#include <mutex>
#include <string>
#include <unordered_map>

namespace perception {
namespace v2x {

class V2xFusion {
 public:
  V2xFusion(const std::shared_ptr<ros::NodeHandle>& node);
  ~V2xFusion() = default;
  // @brief: main entry func
  // @author:liuxinyu
  bool Init();
  std::string Name() const { return "V2xFusion"; }
  // @brief: all v2x message sets
  // @author:liuxinyu
  void CollectionV2xObj();

 private:
  void BsmCallback(
      const autopilot_msgs::BinaryDataConstPtr& msg);  // const std_msgs::StringConstPtr & msg
  void V2iRsmCallback(const autopilot_msgs::BinaryDataConstPtr& msg);
  void V2nRsmCallback(const autopilot_msgs::BinaryDataConstPtr& msg);
  void V2nRsiCallback(const autopilot_msgs::BinaryDataConstPtr& msg);
  void SsmCallback(const autopilot_msgs::BinaryDataConstPtr& msg);
  void LocalizationCallback(const autopilot_msgs::BinaryDataConstPtr& msg);
  bool UpdateV2xObjToImu(localization::Localization& localization);
  bool IsHex(std::string str);
  void InitClassIndex();
  bool QueryNearestLoc(const double& timestamp, localization::Localization& localization);
  bool BsmObj2ObuObjectConvertor(std::unordered_map<int, ::common::BSM_PB>& bsm_objects,
                                 perception::ObuObjects& obu_objects,
                                 perception::ObjectSource& source);
  bool RsmObj2ObuObjectConvertor(std::unordered_map<int, ::common::RSM_PNT_PB>& rsm_objects,
                                 perception::ObuObjects& obu_objects,
                                 perception::ObjectSource& source);
  bool SsmObj2ObuObjectConvertor(std::unordered_map<int, ::common::SSM_PNT_PB>& ssm_objects,
                                 perception::ObuObjects& obu_objects,
                                 perception::ObjectSource& source);
  bool RsiObj2ObuObjectConvertor(std::unordered_map<int, ::common::RSI_RTE_PB>& rsi_objects,
                                 perception::ObuObjects& obu_objects,
                                 perception::ObjectSource& source);

  template <class ProtoMessage>
  void V2xRosToProto(const autopilot_msgs::BinaryData& ros_msg, ProtoMessage& proto_msg) {
    if (ros_msg.name == proto_msg.GetTypeName()) {
      common::DeserializeProto(proto_msg, ros_msg.data);
    } else {
      ROS_ERROR("V2xRosToProto: %s -> %s", ros_msg.name.c_str(), proto_msg.GetTypeName().c_str());
    }
  }

  template <class T>
  void TimeCheck(T &objs, double loc_ts, double thr) {
    if(objs.empty()){
      return;
    }
    if (objs.size() == 0) {
      return;
    }
    for (auto iter = objs.begin(); iter != objs.end();) {
      double ts_obj =
          iter->second.header().stamp().sec() + iter->second.header().stamp().nsec() * 1e-9;
      double diff_time = abs(loc_ts - ts_obj);
      if (diff_time > thr) {
        double candidate_id = iter->first;
        iter++;
        objs.erase(candidate_id);
      } else {
        iter++;
      }
    }
  }

 private:
  std::shared_ptr<ros::NodeHandle> node_ = nullptr;
  std::vector<ros::Subscriber> v2x_subscribers_;
  ros::Subscriber localization_subscriber_;
  ros::Publisher v2x_objects_pub_;
  std::unordered_map<int, ::common::BSM_PB> bsm_objects_;
  std::unordered_map<int, ::common::RSM_PNT_PB> rsm_objects_;
  std::unordered_map<int, ::common::RSM_PNT_PB> v2n_rsm_objects_;
  std::unordered_map<int, ::common::SSM_PNT_PB> v2v_ssm_objects_;
  std::unordered_map<int, ::common::RSI_RTE_PB> v2n_rsi_objects_;
  std::unordered_map<int, ::common::SSM_PNT_PB> v2i_ssm_objects_;
  std::mutex bsm_mutex_, v2x_rsm_mutex_, v2n_rsm_mutex_, ssm_mutex_, rsi_mutex_;
  bool use_filter_ego_car_ = false;  // filter ego car
  std::shared_ptr<perception::v2x::SelfVehIdentBase> self_veh_ident_ptr_ = nullptr;

  std::list<localization::Localization> global_localizations_;
  std::map<int32_t, perception::ObjectType> class_index_;
  bool use_time_filter_;  // filter is or not from cloud data
  double latest_time_;

  bool pub_bsm_ = false;
  bool pub_rsm_ = false;
  bool pub_v2n_rsm_ = false;
  bool pub_v2v_ssm_ = false;
  bool pub_v2n_rsi_ = false;
  bool pub_v2i_ssm_ = false;

  double TimeCheckOfStoreV2ISSM_ = 0.1;
  double TimeCheckOfStoreV2IRSM_ = 0.1;
  double TimeCheckOfStoreV2NRSM_ = 2.0;
  double TimeCheckOfStoreV2NRSI_ = 2.0;
  double TimeCheckOfStoreV2VBSM_ = 0.1;
  double TimeCheckOfLocalization_ = 0.1;

  // rviz
  bool pub_v2x_rviz_ = false;
  ros::Publisher v2x_rte_rviz_pub_;
  ros::Publisher v2x_pnt_rviz_pub_;
  perception::v2x::V2xDisplay v2x_display_;
};


}  // namespace v2x
}  // namespace perception