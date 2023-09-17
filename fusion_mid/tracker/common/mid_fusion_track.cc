#include "perception/fusion_mid/tracker/common/mid_fusion_track.h"
#include "perception/fusion_mid/tracker/filter/adaptive_kalman_filter.h"

const double PI = 3.1415926535898;
const int MAX_OBJ_IDX = 2147483647;
const int ORIGIN_CHENGTECH_MAX_ID_NUM = 255;

namespace perception {
namespace mid_fusion {

// Static member variable
int MidFusionTrack::s_current_idx_ = 0;
int MidFusionTrack::s_tracked_times_threshold_ = 3;
bool MidFusionTrack::s_use_filter_ = false;
std::string MidFusionTrack::s_chosen_filter_ =  // NOLINT
    "AdaptiveKalmanFilter";

MidFusionTrack::MidFusionTrack(const mid_fusion::ObjectPtr& obs, const double timestamp) {
  s_current_idx_ %= MAX_OBJ_IDX;
  // obs_id_ = s_current_idx_++;
  obs_id_ = obs->track_id;
  obs_detected_.reset(new Object);
  *obs_detected_ = *obs;
  obs_.reset(new Object);
  *obs_ = *obs;
  timestamp_ = timestamp;
  tracked_times_ = 1;
  tracking_time_ = 0.0;
  is_dead_ = false;

  // Or use register class instead.
  filter_.reset(new AdaptiveKalmanFilter);
  filter_->Init(*obs);
}

void MidFusionTrack::UpdataDetectedObs(const mid_fusion::ObjectPtr& obs_detected,
                                const double timestamp) {
  obs_detected->ToString();
  *obs_detected_ = *obs_detected;
  *obs_ = *obs_detected;
  double time_diff = timestamp - timestamp_;
  if (s_use_filter_) {
    Eigen::VectorXd state;
    state = filter_->UpdateWithObject(*obs_detected_, time_diff);
    ROS_DEBUG_STREAM("UpdataDetectedObs: center(" << state(0) << "," << state(1) << ")velocity("
                                                 << state(2) << "," << state(3) << ")");
    obs_->center(0) = static_cast<float>(state(0));
    obs_->center(1) = static_cast<float>(state(1));
    obs_->velocity(0) = static_cast<float>(state(2));
    obs_->velocity(1) = static_cast<float>(state(3));
    Eigen::Matrix4d covariance_matrix = filter_->GetCovarianceMatrix();
    obs_->center_uncertainty(0) = static_cast<float>(covariance_matrix(0, 0));
    obs_->center_uncertainty(1) = static_cast<float>(covariance_matrix(1, 1));
    obs_->velocity_uncertainty(0) = static_cast<float>(covariance_matrix(2, 2));
    obs_->velocity_uncertainty(1) = static_cast<float>(covariance_matrix(3, 3));
  }
  tracking_time_ += time_diff;
  timestamp_ = timestamp;
  ++tracked_times_;
}

void MidFusionTrack::SetObsNullptr() {
  obs_detected_ = nullptr;
  obs_ = nullptr;
}

mid_fusion::ObjectPtr MidFusionTrack::GetDetectedObs() { return obs_detected_; }

mid_fusion::ObjectPtr MidFusionTrack::GetObs() { return obs_; }

int MidFusionTrack::GetObsId() const { return obs_id_; }

double MidFusionTrack::GetTimestamp() { return timestamp_; }

double MidFusionTrack::GetTrackingTime() { return tracking_time_; }
}  // namespace mid_fusion
}  // namespace perception
