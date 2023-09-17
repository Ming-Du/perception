#pragma once

#include <string>
#include <utility>
#include <vector>
#include <ros/ros.h>


#include "perception/fusion_mid/tracker/base/frame.h"
#include "perception/lib/registerer/registerer.h"
#include "perception/fusion_mid/tracker/common/mid_fusion_track.h"

namespace perception {
namespace mid_fusion {
typedef std::pair<size_t, size_t> TrackObjectPair;

struct TrackObjectMatcherOptions {
  Eigen::Vector3d *ref_point = nullptr;
};

class BaseMatcher {
 public:
  BaseMatcher() : name_("BaseMatcher") {}
  virtual ~BaseMatcher() {}
  virtual bool Init() { return true; }
  // @brief match mid_fusion objects to tracks
  // @params[IN] mid_fusion_tracks: global tracks
  // @params[IN] detected_frame: current detected frame
  // @params[IN] options: matcher options for future use
  // @params[OUT] assignments: matched pair of tracks and measurements
  // @params[OUT] unassigned_tracks: unmatched tracks
  // @params[OUT] unassigned_objects: unmatched objects
  // @return nothing
  virtual bool Match(const std::vector<MidFusionTrackPtr> &mid_fusion_tracks,
                     const mid_fusion::Frame &detected_frame,
                     const TrackObjectMatcherOptions &options,
                     std::vector<TrackObjectPair> *assignments,
                     std::vector<size_t> *unassigned_tracks,
                     std::vector<size_t> *unassigned_objects) {
    return true;
  }
  virtual void IDMatch(const std::vector<MidFusionTrackPtr> &mid_fusion_tracks,
                       const mid_fusion::Frame &detected_frame,
                       std::vector<TrackObjectPair> *assignments,
                       std::vector<size_t> *unassigned_tracks,
                       std::vector<size_t> *unassigned_objects);
  static void SetMaxMatchDistance(double dist);
  static double GetMaxMatchDistance();
  static void SetBoundMatchDistance(double dist);
  static double GetBoundMatchDistance();
  virtual std::string Name() { return name_; }

 protected:
  std::string name_;
  static double s_max_match_distance_;
  static double s_bound_match_distance_;
  virtual bool RefinedTrack(const mid_fusion::ObjectPtr &track_object,
                            double track_timestamp,
                            const mid_fusion::ObjectPtr &mid_fusion_object,
                            double mid_fusion_timestamp);
  // FRIEND_TEST(BaseMatcherTest, base_matcher_test);

};

PERCEPTION_REGISTER_REGISTERER(BaseMatcher);
#define PERCEPTION_REGISTER_MATCHER(name) \
  PERCEPTION_REGISTER_CLASS(BaseMatcher, name)

}  // namespace mid_fusion
}  // namespace perception
