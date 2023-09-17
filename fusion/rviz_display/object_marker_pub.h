//
// Created by tiger on 23-5-6.
//

#ifndef CATKIN_WS_OBJECT_MARKER_PUB_H
#define CATKIN_WS_OBJECT_MARKER_PUB_H
#include <iostream>
#include <string>
#include "base_marker_pub.h"
#include "common/proto/localization.pb.h"
#include <unordered_map>

namespace perception {
namespace fusion {

//typedef struct ObhectMarker_ {
//  ROS_VISUALIZATION_MARKER marker;
//  int action = 0;
//}ObjectMarker;

class ObjectMarkerPub : public BaseMarkerPub{
public:
  using Ptr = std::shared_ptr<ObjectMarkerPub>;

  // TrajMarkerPub() = default;

  void init(const MarkerPubOptions& options) override {
    if (options.node_ptr == nullptr) {
      return;
    }
    options_ = options;
  }

  virtual std::vector<ROS_VISUALIZATION_MARKER>& display(const TrackedObjects& tracked_objects) override;
  std::vector<ROS_VISUALIZATION_MARKER>& display_ego_text(
      const localization::Localization localization);

  void TrackInfo(const TrackedObject& obj, ROS_VISUALIZATION_MARKER &marker);
  void LabelInfo(const TrackedObject &tmp_obj, ROS_VISUALIZATION_MARKER &marker);
  void PolygonInfo(const TrackedObject &tmp_obj, ROS_VISUALIZATION_MARKER &tmp_maker);
  void ArrowInfo(const TrackedObject& tmp_obj, ROS_VISUALIZATION_MARKER &tmp_maker, bool is_arrow, double alpha = 1.);
  void TrajectoryInfo(const TrackedObject &tmp_obj, ROS_VISUALIZATION_MARKER &marker);
  void TrajectoryCubeInfo(const TrackedObject &tmp_obj, ROS_VISUALIZATION_MARKER &marker);
  bool FindKey(std::vector<TrackedObject> &obj_vec, int key);
  bool FindDelIndex(std::vector<int> del_mkid_index, int mkid);
  void UpdateRviz(std::vector<TrackedObject> &obj_vec);
  std::vector<ROS_VISUALIZATION_MARKER>& display_app(const TrackedObjects& tracked_objects, const localization::Localization localization);
//  void DisplayAllInfo(const TrackedObject& tmp_obj, int index, bool is_arrow, double alpha = 1.);
//  void VelDirectionInfo(const TrackedObject &tmp_obj);
  std::string name() override { return "DetailObject"; }

  Eigen::Quaterniond RPYToQuaterniond(const float& roll = 0.0,
                                      const float& pitch = 0.0,
                                      const float& yaw = 0.0);
  int mk_id;
  std::unordered_map<int, std::vector<int>> id_key_;
  std::unordered_map<int, std::string> delete_key_;
  std::unordered_map<std::string, std::vector<int>> delete_map_;
  std::vector<int> del_mkid_index_; //delete
  int delete_key_nums_ = 0; //need to delete marker
  int update_key_nums_ = 0; //need to update marker
private:
  localization::Localization localization_;
};
}
}


#endif //CATKIN_WS_OBJECT_MARKER_PUB_H
