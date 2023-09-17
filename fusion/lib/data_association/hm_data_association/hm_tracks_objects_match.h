#pragma once

#include <string>
#include <vector>

#include "lib/data_association/hm_data_association/track_object_distance.h"
#include "lib/interface/base_data_association.h"
#include "perception/common/graph/gated_hungarian_bigraph_matcher.h"
#include"base/configmanager.h"

namespace perception {
namespace fusion {

class HMTrackersObjectsAssociation : public BaseDataAssociation {
 public:
  HMTrackersObjectsAssociation() {}
  ~HMTrackersObjectsAssociation() {}
  HMTrackersObjectsAssociation(const HMTrackersObjectsAssociation&) = delete;
  HMTrackersObjectsAssociation& operator=(const HMTrackersObjectsAssociation&) = delete;

  bool Init() override {
    track_object_distance_.set_distance_thresh(static_cast<float>(s_match_distance_thresh_));

    bool ret_x = ConfigManager::Instance()->get_value("idassign_devposition_x",
                                                      &idassign_diff_px);
    bool ret_y = ConfigManager::Instance()->get_value("idassign_devposition_y",
                                               &idassign_diff_py);
    if (!ret_x || !ret_y) {
      idassign_diff_px = 10.0;
      idassign_diff_py = 10.0;
    }
    return true;
  }

  bool Associate(const AssociationOptions& options,
                 SensorFramePtr sensor_measurements,
                 ScenePtr scene,
                 AssociationResult* association_result) override;

  std::string Name() const override { return "HMTrackersObjectsAssociation"; }

 private:
  void ComputeAssociationDistanceMat(const std::vector<TrackPtr>& fusion_tracks,
                                     const std::vector<SensorObjectPtr>& sensor_objects,
                                     const Eigen::Vector3d& ref_point,
                                     const std::vector<size_t>& unassigned_tracks,
                                     const std::vector<size_t>& unassigned_measurements,
                                     std::vector<std::vector<double>>* association_mat,
                                     bool iscamera);

  void IdAssign(const std::vector<TrackPtr>& fusion_tracks,
                const std::vector<SensorObjectPtr>& sensor_objects,
                std::vector<TrackMeasurmentPair>* assignments,
                std::vector<size_t>* unassigned_fusion_tracks,
                std::vector<size_t>* unassigned_sensor_objects,
                bool do_nothing = false,
                bool post = false);

  void PostIdAssign(const std::vector<TrackPtr>& fusion_tracks,
                    const std::vector<SensorObjectPtr>& sensor_objects,
                    const std::vector<size_t>& unassigned_fusion_tracks,
                    const std::vector<size_t>& unassigned_sensor_objects,
                    std::vector<TrackMeasurmentPair>* post_assignments);

  bool MinimizeAssignment(const std::vector<std::vector<double>>& association_mat,
                          const std::vector<size_t>& track_ind_l2g,
                          const std::vector<size_t>& measurement_ind_l2g,
                          std::vector<TrackMeasurmentPair>* assignments,
                          std::vector<size_t>* unassigned_tracks,
                          std::vector<size_t>* unassigned_measurements);

  void ComputeDistance(const std::vector<TrackPtr>& fusion_tracks,
                       const std::vector<SensorObjectPtr>& sensor_objects,
                       const std::vector<size_t>& unassigned_fusion_track,
                       const std::vector<int>& track_ind_g2l,
                       const std::vector<int>& measurement_ind_g2l,
                       const std::vector<size_t>& measurement_ind_l2g,
                       const std::vector<std::vector<double>>& association_mat,
                       AssociationResult* association_result);

  void GenerateUnassignedData(size_t track_num,
                              size_t objects_num,
                              const std::vector<TrackMeasurmentPair>& assignments,
                              std::vector<size_t>* unassigned_tracks,
                              std::vector<size_t>* unassigned_objects);
  //@lijian1,
  //@描述：该函数主要是从距离代价矩阵中选择出每一个track与那些测量值代价最小的这个测量值的位置（也可以是其他测量值的唯一标识）
  //@输入1：association_mat
  //@输出1：minDistanceSensorObjIndex
  std::vector<int> SelectMinMatchDistanceforTrack(
      std::vector<std::vector<double>>& association_mat);
  //@lijian1,
  //@描述：该函数作用是修改距离代价矩阵，依据是若多个track匹配到了同一个测量值，则根据track离自车的距离远近，修改调整距离代价
  //@输入1：track数组
  //@输入2：最小代价位置记录数组
  //@输入3：未匹配的track位置数组
  //@输入4：代价矩阵associtation
  //@输出1：代价矩阵
  void AdjustAssociationMat(const std::vector<TrackPtr>& fusion_tracks,
                            const std::vector<size_t>& unassigned_tracks,
                            const std::vector<int>& mindistanceindex,
                            std::vector<std::vector<double>>* association_mat);

 private:
  common::GatedHungarianMatcher<float> optimizer_;
  TrackObjectDistance track_object_distance_;
  static double s_match_distance_thresh_;
  static double s_match_distance_bound_;
  static double s_association_center_dist_threshold_;
  double idassign_diff_px, idassign_diff_py;
};

}  // namespace fusion
}  // namespace perception
