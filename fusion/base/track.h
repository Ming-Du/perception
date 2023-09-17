#pragma once
#include <map>
#include <memory>
#include <string>

#include "gtest/gtest_prod.h"

#include "base/sensor_object.h"

namespace perception {
namespace fusion {

typedef std::map<std::string, SensorObjectPtr> SensorId2ObjectMap;

class Track {
 public:
  Track();
  virtual ~Track() = default;

  Track(const Track&) = delete;
  Track& operator=(const Track&) = delete;

  // static members initialization
  inline static void SetMaxLidarInvisiblePeriod(double period) {
    s_max_lidar_invisible_period_ = period;
  }
  inline static void SetMaxRadarInvisiblePeriod(double period) {
    s_max_radar_invisible_period_ = period;
  }
  inline static void SetMaxCameraInvisiblePeriod(double period) {
    s_max_camera_invisible_period_ = period;
  }
  // Modify @jiangnan:add falcon lidar
  inline static void SetMaxFalconLidarInvisiblePeriod(double period) {
    s_max_falcon_lidar_invisible_period_ = period;
  }
  // Modify(@liuxinyu): obu_test
  inline static void SetMaxObuInvisiblePeriod(double period) {
    s_max_obu_invisible_period_ = period;
  }
  inline static void SetMaxVidarInvisiblePeriod(double period) {
    s_max_vidar_invisible_period_ = period;
  }

  bool Initialize(SensorObjectPtr obj, bool is_background = false);

  void Reset();

  SensorObjectConstPtr GetSensorObject(const std::string& sensor_id) const;
  SensorObjectConstPtr GetLatestLidarObject() const;
  SensorObjectConstPtr GetLatestRadarObject() const;
  SensorObjectConstPtr GetLatestCameraObject() const;
  // Modify @jiangnan:add falcon lidar
  SensorObjectConstPtr GetLatestFalconLidarObject() const;
  // Modify(@liuxinyu): obu_test
  SensorObjectConstPtr GetLatestObuObject() const;
  SensorObjectConstPtr GetLatestVidarObject() const;

  inline FusedObjectPtr GetFusedObject() { return fused_object_; }
  inline SensorId2ObjectMap& GetLidarObjects() { return lidar_objects_; }

  inline const SensorId2ObjectMap& GetLidarObjects() const { return lidar_objects_; }

  inline SensorId2ObjectMap& GetRadarObjects() { return radar_objects_; }

  inline const SensorId2ObjectMap& GetRadarObjects() const { return radar_objects_; }

  inline SensorId2ObjectMap& GetCameraObjects() { return camera_objects_; }

  inline const SensorId2ObjectMap& GetCameraObjects() const { return camera_objects_; }
  // Modify @jiangnan: add falcon lidar
  inline const SensorId2ObjectMap& GetFalconLidarObjects() const { return falcon_lidar_objects_; }

  // Modify(@liuxinyu): obu_test
  inline SensorId2ObjectMap& GetObuObjects() { return obu_objects_; }

  inline const SensorId2ObjectMap& GetObuObjects() const { return obu_objects_; }
  inline SensorId2ObjectMap& GetVidarObjects() { return vidar_objects_; }

  inline const SensorId2ObjectMap& GetVidarObjects() const { return vidar_objects_; }

  inline int GetTrackId() const { return fused_object_->GetBaseObject()->track_id; }

  inline double GetTrackingPeriod() const { return tracking_period_; }

  inline size_t GetTrackedTimes() const { return tracked_times_; }
  inline int GetMatchTimes() const {return match_times_;}
  inline void addMatchTimes(){++match_times_;}
  inline void AddTrackedTimes() { ++tracked_times_; }

  inline void SetIsPublish() {is_publish_ = true;}

  inline bool GetIsPublish() { return is_publish_; }

  inline double GetExistenceProb() const { return existence_prob_; }

  inline void SetExistenceProb(double prob) { existence_prob_ = prob; }

  inline double GetToicProb() const { return toic_prob_; }

  inline void SetToicProb(double prob) { toic_prob_ = prob; }
  inline bool IsBackground() const { return is_background_; }

  inline bool IsAlive() const { return is_alive_; }

  bool IsVisible(const std::string& sensor_id) const;
  bool IsLidarVisible() const;
  bool IsRadarVisible() const;
  bool IsCameraVisible() const;
  // Modify @jiangnan:add falcon lidar
  bool IsFalconLidarVisible() const;
  // Modify(@liuxinyu): obu_test
  bool IsObuVisible() const;
  bool IsVidarVisible() const;  // ming.du
  bool IsPredicted() const;
//  bool IsMultiOverlapTrack() const;
  void SetAlive(
      bool is_alive);  // set track alive status, and so we can control track if should be deleted
  void SetPredictionState(bool is_predicted);
//  void SetMultiOverlapTrack(bool flag);

  static size_t GenerateNewTrackId();

  void UpdateWithSensorObject(const SensorObjectPtr& obj);

  void UpdateWithoutSensorObject(const std::string& sensor_id, double measurement_timestamp);

  std::string DebugString() const;
 protected:
  // update state
  void UpdateSupplementState(const SensorObjectPtr& src_object = nullptr);
  void UpdateUnfusedState(const SensorObjectPtr& src_object);

  SensorObjectConstPtr GetLatestSensorObject(const SensorId2ObjectMap& objects) const;
  void UpdateSensorObject(SensorId2ObjectMap* objects, const SensorObjectPtr& obj);
  void UpdateSensorObjectWithoutMeasurement(SensorId2ObjectMap* objects,
                                            const std::string& sensor_id,
                                            double measurement_timestamp,
                                            double max_invisible_period);
  void UpdateSensorObjectWithMeasurement(SensorId2ObjectMap* objects,
                                         const std::string& sensor_id,
                                         double measurement_timestamp,
                                         double max_invisible_period);
  void UpdateWithSensorObjectForBackground(const SensorObjectPtr& obj);
  void UpdateWithoutSensorObjectForBackground(const std::string& sensor_id,
                                              double measurement_timestamp);

 public:
  static int prediction_count_;

 protected:
  SensorId2ObjectMap lidar_objects_;
  SensorId2ObjectMap radar_objects_;
  SensorId2ObjectMap camera_objects_;
  SensorId2ObjectMap falcon_lidar_objects_;  // modify @jiangnan:add falcon lidar
  // Modify(@liuxinyu): obu_test
  SensorId2ObjectMap obu_objects_;
  SensorId2ObjectMap vidar_objects_;  // add ming.du
  FusedObjectPtr fused_object_ = nullptr;

  double tracking_period_ = 0.0;
  double existence_prob_ = 0.0;
  double toic_prob_ = 0.0;

  bool is_background_ = false;
  bool is_alive_ = true;
  bool is_predicted_ = false;
  int prediction_time_ = 0;
//  bool is_multi_overlap_track_ = false;
  int match_times_ = 0;//非雷神激光匹配的次数统计
  size_t tracked_times_ = 0;
  bool is_publish_ = 0;

 private:
  FRIEND_TEST(TrackTest, test);

  static size_t s_track_idx_;
  static double s_max_lidar_invisible_period_;
  static double s_max_radar_invisible_period_;
  static double s_max_camera_invisible_period_;
  static double s_max_falcon_lidar_invisible_period_;  // Modify @jiangnan : add falcon lidar
  static double s_max_obu_invisible_period_;
  static double s_max_vidar_invisible_period_;
};

typedef std::shared_ptr<Track> TrackPtr;
typedef std::shared_ptr<const Track> TrackConstPtr;


//typedef struct ExtendTrack_ {
//  TrackPtr track;
//  int merge;//0 delete own  1 delete relative 2 update own 3 update other
//  double area;
//  std::vector<TrackPtr> relative_tracks;
//  int multi_overlap_track;
//  void Check() {
//    if (relative_tracks.size() > 1) multi_overlap_track = 1;
//  }
//  void Init() {
//    merge = 0;
//    multi_overlap_track = 0;
//    area = 0;
//    track = nullptr;
//  }
//}ExtendTrack;

}  // namespace fusion
}  // namespace perception
