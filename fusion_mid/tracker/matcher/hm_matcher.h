#pragma once

#include <vector>

#include "tracker/base/frame.h"
#include "perception/common/graph/gated_hungarian_bigraph_matcher.h"
#include "perception/lib/registerer/registerer.h"
#include "perception/fusion_mid/tracker/interface/base_matcher.h"
#include "perception/fusion_mid/proto/hm_matcher_conf.pb.h"
#include "perception/base/perception_gflags.h"
#include "common/include/pb_utils.h"


namespace perception {
namespace mid_fusion {

class HMMatcher : public BaseMatcher {
 public:
  HMMatcher();
  virtual ~HMMatcher();
  bool Init() override;
  // @brief match mid_fusion objects to tracks
  // @params[IN] mid_fusion_tracks: global tracks
  // @params[IN] mid_fusion_frame: current mid_fusion frame
  // @params[IN] options: matcher options for future use
  // @params[OUT] assignments: matched pair of tracks and measurements
  // @params[OUT] unassigned_tracks: unmatched tracks
  // @params[OUT] unassigned_objects: unmatched objects
  // @return nothing
  bool Match(const std::vector<MidFusionTrackPtr> &mid_fusion_tracks,
             const mid_fusion::Frame &detected_frame,
             const TrackObjectMatcherOptions &options,
             std::vector<TrackObjectPair> *assignments,
             std::vector<size_t> *unassigned_tracks,
             std::vector<size_t> *unassigned_objects) override;

 protected:
  bool RefinedTrack(const mid_fusion::ObjectPtr &track_object, double track_timestamp,
                    const mid_fusion::ObjectPtr &mid_fusion_object,
                    double mid_fusion_timestamp) override;

 private:
  common::GatedHungarianMatcher<double> hungarian_matcher_;
  void TrackObjectPropertyMatch(const std::vector<MidFusionTrackPtr> &mid_fusion_tracks,
                                const mid_fusion::Frame &detected_frame,
                                std::vector<TrackObjectPair> *assignments,
                                std::vector<size_t> *unassigned_tracks,
                                std::vector<size_t> *unassigned_objects);
  void ComputeAssociationMat(const std::vector<MidFusionTrackPtr> &mid_fusion_tracks,
                             const mid_fusion::Frame &detected_frame,
                             const std::vector<size_t> &unassigned_tracks,
                             const std::vector<size_t> &unassigned_objects,
                             std::vector<std::vector<double>> *association_mat);
  double DistanceBetweenObs(const mid_fusion::ObjectPtr &obs1, double timestamp1,
                            const mid_fusion::ObjectPtr &obs2, double timestamp2);
};

}  // namespace mid_fusion
}  // namespace perception
