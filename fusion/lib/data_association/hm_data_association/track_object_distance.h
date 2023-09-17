#pragma once

#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/StdVector"

#include "base/fusion_log.h"
#include "base/sensor_data_manager.h"
#include "base/sensor_frame.h"
#include "base/sensor_object.h"
#include "base/track.h"
#include "common/camera_util.h"
#include "lib/data_association/hm_data_association/probabilities.h"
#include "lib/data_association/hm_data_association/projection_cache.h"
#include "lib/data_association/hm_data_association/track_object_similarity.h"
#include "perception/base/sensor_manager/sensor_manager.h"

namespace perception {
namespace fusion {

struct TrackObjectDistanceOptions {
  Eigen::Vector3d* ref_point = nullptr;
};  // struct TrackedObjectDistanceOptions

class TrackObjectDistance {
 public:
  TrackObjectDistance() = default;
  ~TrackObjectDistance() = default;
  TrackObjectDistance(const TrackObjectDistance&) = delete;
  TrackObjectDistance operator=(const TrackObjectDistance&) = delete;

  void set_distance_thresh(const float distance_thresh) { distance_thresh_ = distance_thresh; }
  void ResetProjectionCache(std::string sensor_id, double timestamp) {
    projection_cache_.Reset(sensor_id, timestamp);
  }
  // @brief: compute the distance between input fused track and sensor object
  // @params [in] fused_track: maintained fused track
  // @params [in] sensor_object: sensor observation
  // @params [in] options: options of track object distanace computation
  // @return track object distance
  float Compute(const TrackPtr& fused_track,
                const SensorObjectPtr& sensor_object,
                const TrackObjectDistanceOptions& options);
  // @brief: calculate the similarity between velodyne64 observation and
  // camera observation
  // @return the similarity which belongs to [0, 1]. When velodyne64
  // observation is similar to the camera one, the similarity would
  // close to 1. Otherwise, it would close to 0.
  // @NOTE: original method name is compute_velodyne64_camera_dist_score
  double ComputeLidarCameraSimilarity(const SensorObjectConstPtr& lidar,
                                      const SensorObjectConstPtr& camera,
                                      const bool measurement_is_lidar);
  // @brief: calculate the similarity between radar observation and
  // camera observation
  // @retrun the similarity which belongs to [0, 1]. When radar
  // observation is similar to the camera one, the similarity would
  // close to 1. Otherwise, it would close to 0.
  // @NOTE: original method name is compute_radar_camera_dist_score
  double ComputeRadarCameraSimilarity(const SensorObjectConstPtr& radar,
                                      const SensorObjectConstPtr& camera);

 protected:
  // @brief: compute the distance between lidar observation and
  // lidar observation
  // @return distance of lidar vs. lidar
  float ComputeLidarLidar(const SensorObjectConstPtr& fused_object,
                          const SensorObjectPtr& sensor_object,
                          const Eigen::Vector3d& ref_pos,
                          int range = 3);
  // @brief: compute the distance between lidar observation and
  // radar observation
  // @return distance of lidar vs. radar
  float ComputeLidarRadar(const SensorObjectConstPtr& fused_object,
                          const SensorObjectPtr& sensor_object,
                          const Eigen::Vector3d& ref_pos,
                          int range = 3);
  // @brief: compute the distance between radar observation and
  // radar observation
  // @return distance of radar vs. radar
  float ComputeRadarRadar(const SensorObjectConstPtr& fused_object,
                          const SensorObjectPtr& sensor_object,
                          const Eigen::Vector3d& ref_pos,
                          int range = 3);
  // @brief: compute the distance between lidar observation and
  // camera observation
  // @return distance of lidar vs. camera
  float ComputeLidarCamera(const SensorObjectConstPtr& lidar,
                           const SensorObjectConstPtr& camera,
                           const bool measurement_is_lidar,
                           const bool is_track_id_consistent);
  /**
   * use camera2D depth to optimize distance between lidar and camera
   * @param lidar
   * @param camera
   * @param is_track_id_consistent
   * @return
   */
  float ComputeLidarCamera2D(const SensorObjectConstPtr& lidar,
                       const SensorObjectConstPtr& camera,
                       const bool is_track_id_consistent);
  // Modify(@liuxinyu): obu_test
  float ComputeLidarObu(const SensorObjectConstPtr& fused_object,
                        const SensorObjectPtr& sensor_object,
                        const Eigen::Vector3d& ref_pos,
                        int range = 3);
  float ComputeObuObu(const SensorObjectConstPtr& fused_object,
                      const SensorObjectPtr& sensor_object,
                      const Eigen::Vector3d& ref_pos,
                      int range = 3);
  float ComputeRadarObu(const SensorObjectConstPtr& fused_object,
                      const SensorObjectPtr& sensor_object,
                      const Eigen::Vector3d& ref_pos,
                      int range = 3);
  float ComputeVidarObu(const SensorObjectConstPtr& fused_object,
                      const SensorObjectPtr& sensor_object,
                      const Eigen::Vector3d& ref_pos,
                      double center_distance_thresh,
                      int range = 3);
  // @brief: compute the distance between radar observation and
  // camera observation
  // @return distance of radar vs. camera
  float ComputeRadarCamera(const SensorObjectConstPtr& radar, const SensorObjectConstPtr& camera);

  float ComputeLidarVidar(const SensorObjectConstPtr& fused_object,
                          const SensorObjectPtr& sensor_object,
                          const Eigen::Vector3d& ref_pos,
                          double center_distance_thresh,
                          int range = 3);
  float ComputeRadarVidar(const SensorObjectConstPtr& fused_object,
                          const SensorObjectPtr& sensor_object,
                          const Eigen::Vector3d& ref_pos,
                          double center_distance_thresh,
                          int range = 3);
  float ComputeObuVidar(const SensorObjectConstPtr& fused_object,
                        const SensorObjectPtr& sensor_object,
                        const Eigen::Vector3d& ref_pos,
                        double center_distance_thresh,
                        int range = 3);
  float ComputeVidarVidar(const SensorObjectConstPtr& fused_object,
                          const SensorObjectPtr& sensor_object,
                          const Eigen::Vector3d& ref_pos,
                          double center_distance_thresh,
                          int range = 3);

  float ComputeSensor2Sensor(const SensorObjectConstPtr& fused_object,
                             const SensorObjectPtr& sensor_object,
                             const Eigen::Vector3d& ref_pos,
                             double center_distance_thresh,
                             int range = 3);
  float ComputeSensorCamera(const SensorObjectConstPtr& sensor, const SensorObjectConstPtr& camera);

  float ComputeLidarFused(const SensorObjectPtr& sensor_object,
                          Eigen::Vector3d* ref_point = nullptr);
  float ComputeCamera2DFused(const SensorObjectPtr& sensor_object,
                             Eigen::Vector3d* ref_point = nullptr);
  float ComputeObuFused(const SensorObjectPtr& sensor_object, Eigen::Vector3d* ref_point = nullptr);
  float ComputeRadarFused(const SensorObjectPtr& sensor_object,
                          Eigen::Vector3d* ref_point = nullptr);
  float ComputeFalconLidarFused(const SensorObjectPtr& sensor_object,
                                Eigen::Vector3d* ref_point = nullptr);
  float ComputeVidarFused(const SensorObjectPtr& sensor_object,
                          Eigen::Vector3d* ref_point = nullptr);
  // @brief: compute the distance between camera observation and
  // camera observation
  // @return distance of camera vs. camera
  float ComputeCameraCamera(const SensorObjectPtr& fused_camera,
                            const SensorObjectPtr& sensor_camera);
  // Modify(@liuxinyu): obu_test
  float ComputeObuCamera(const SensorObjectConstPtr& obu, const SensorObjectConstPtr& camera);
  double ComputePolygonIOU(const SensorObjectConstPtr& fused_object,
                           const SensorObjectPtr& sensor_object);

  // @brief: compute 3d distance between fused object and sensor object
  // @return 3d distance betwene fused object and sensor object
  float ComputePolygonDistance3d(const SensorObjectConstPtr& fused_object,
                                 const SensorObjectPtr& sensor_object,
                                 const Eigen::Vector3d& ref_pos,
                                 int range);
  // @brief: compute euclidean distance
  // @return euclidean distance of input pts
  float ComputeEuclideanDistance(const Eigen::Vector3d& des, const Eigen::Vector3d& src);
  // @brief: compute polygon center
  // @params [out] center: center of input polygon
  // @return true if get center successfully, otherwise return false
  bool ComputePolygonCenter(const fusion::PolygonDType& polygon, Eigen::Vector3d* center);
  // @brief: compute polygon center
  // @params [out] center: center of input polygon
  // @return true if get center successfully, otherwise return false
  bool ComputePolygonCenter(const fusion::PolygonDType& polygon,
                            const Eigen::Vector3d& ref_pos,
                            int range,
                            Eigen::Vector3d* center);
  
  //Modify @ jiangnan
  float ComputeTypeProbality(const ObjectType& fused_type, const ObjectType& sensor_type);

  // Modify @jiangnan: computer  distance  between lidar and radar
  float ComputerPointLinesegmentDistance(const fusion::PointD& pointA,
                                         const fusion::PointD& pointB,
                                         const fusion::PointD& pointO);
float ComputeVidarandOtherDistance(const SensorObjectConstPtr& fused_object,
                             const SensorObjectPtr& sensor_object,
                             const Eigen::Vector3d& ref_pos,
                             double center_distance_thresh,
                             int range = 3);
 private:
  base::BaseCameraModelPtr QueryCameraModel(const SensorObjectConstPtr& camera);
  base::BaseCameraDistortionModelPtr QueryCameraDistortionModel(const SensorObjectConstPtr& camera);
  bool QueryWorld2CameraPose(const SensorObjectConstPtr& camera, Eigen::Matrix4d* pose);
  bool QueryLidar2WorldPose(const SensorObjectConstPtr& lidar, Eigen::Matrix4d* pose);
  void QueryProjectedVeloCtOnCamera(const SensorObjectConstPtr& velodyne64,
                                    const SensorObjectConstPtr& camera,
                                    const Eigen::Matrix4d& lidar2camera_pose,
                                    Eigen::MatrixXd* polygon_image);
  bool QueryPolygonDCenter(const fusion::ObjectConstPtr& object,
                           const Eigen::Vector3d& ref_pos,
                           const int range,
                           Eigen::Vector3d* polygon_ct);
  void GetModified2DRadarBoxVertices(const std::vector<Eigen::Vector3d>& radar_box_vertices,
                                     const SensorObjectConstPtr& camera,
                                     const base::BaseCameraModelPtr& camera_intrinsic,
                                     const Eigen::Matrix4d& world2camera_pose,
                                     std::vector<Eigen::Vector2d>* radar_box2d_vertices);
  ProjectionCacheObject* BuildProjectionCacheObject(const SensorObjectConstPtr& lidar,
                                                    const SensorObjectConstPtr& camera,
                                                    const base::BaseCameraDistortionModelPtr& camera_model,
                                                    const std::string& measurement_sensor_id,
                                                    double measurement_timestamp,
                                                    const std::string& projection_sensor_id,
                                                    double projection_timestamp);
  ProjectionCacheObject* QueryProjectionCacheObject(const SensorObjectConstPtr& lidar,
                                                    const SensorObjectConstPtr& camera,
                                                    const base::BaseCameraDistortionModelPtr& camera_model,
                                                    const bool measurement_is_lidar);
  bool IsTrackIdConsistent(const SensorObjectConstPtr& object1,
                           const SensorObjectConstPtr& object2);
  bool LidarCameraCenterDistanceExceedDynamicThreshold(const SensorObjectConstPtr& lidar,
                                                       const SensorObjectConstPtr& camera);

  void MakeSensorMap(const TrackPtr& fused_track);
  ProjectionCache projection_cache_;
  float distance_thresh_ = 4.5f;
  const float vc_similarity2distance_penalize_thresh_ = 0.07f;
  const float vc_diff2distance_scale_factor_ = 0.02f;  // 0.8f;     //syf-to debug
  const float rc_similarity2distance_penalize_thresh_ = 0.1f;
  const float rc_x_similarity_params_2_welsh_loss_scale_ = 0.5f;
  const Eigen::Vector2d rc_min_box_size_ = Eigen::Vector2d(25, 25);

  XSimilarityParams rc_x_similarity_params_ = XSimilarityParams();
  YSimilarityParams rc_y_similarity_params_ = YSimilarityParams();
  HSimilarityParams rc_h_similarity_params_ = HSimilarityParams();
  WSimilarityParams rc_w_similarity_params_ = WSimilarityParams();
  LocSimilarityParams rc_loc_similarity_params_ = LocSimilarityParams();
  XSimilarityParams rc_x_similarity_params_2_ = XSimilarityParams();

 private:
  double s_lidar2lidar_association_center_dist_threshold_ = 10.0;
  double s_lidar2radar_association_center_dist_threshold_ = 10.0;
  double s_radar2radar_association_center_dist_threshold_ = 10.0;
  size_t s_lidar2camera_projection_downsample_target_pts_num_ = 100;
  size_t s_lidar2camera_projection_vertices_check_pts_num_ = 20;
  double s_lidar2obu_association_center_dist_threshold_ = 10.0;
  double s_radar2obu_association_center_dist_threshold_ = 10.0;
  double s_lidar2vidar_association_center_dist_threshold_ = 15.0;
  double s_radar2vidar_association_center_dist_threshold_ = 15.0;
  double s_vidar2obu_association_center_dist_threshold_ = 15.0;
  double s_vidar2vidar_association_center_dist_threshold_ = 10.0;

  double s_lidar2lidar_coefficient_dist_threshold_ = 35.0;

  double s_lidar2camera3d_association_center_dist_threshold_ = 10.0;

  std::unordered_map<std::string, SensorObjectConstPtr> sensor_map_;
  TrackPtr fused_track_;
};  // class TrackObjectDistance

}  // namespace fusion
}  // namespace perception
