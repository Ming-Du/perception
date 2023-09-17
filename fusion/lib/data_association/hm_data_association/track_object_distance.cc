
#include "track_object_distance.h"

#include <algorithm>
#include <boost/format.hpp>
#include <limits>
#include <map>
#include <utility>

#include "base/point.h"
#include "chi_squared_cdf_1_0.0500_0.999900.h"
#include "chi_squared_cdf_2_0.0500_0.999900.h"
#include "common/iou.h"
#include "fstream"
#include "perception/base/camera.h"
#include "perception/common/geometry/camera_homography.h"
#include "common/proto/sensor_meta.pb.h"

namespace perception {
namespace fusion {

void TrackObjectDistance::GetModified2DRadarBoxVertices(
    const std::vector<Eigen::Vector3d>& radar_box_vertices,
    const SensorObjectConstPtr& camera,
    const base::BaseCameraModelPtr& camera_intrinsic,
    const Eigen::Matrix4d& world2camera_pose,
    std::vector<Eigen::Vector2d>* radar_box2d_vertices) {
  const double camera_height = camera->GetBaseObject()->size(2);
  std::vector<Eigen::Vector3d> modified_radar_box_vertices = radar_box_vertices;
  for (size_t i = 0; i < 4; ++i) {
    modified_radar_box_vertices[i + 4].z() = modified_radar_box_vertices[i].z() + camera_height;
  }
  radar_box2d_vertices->reserve(radar_box_vertices.size());
  for (const auto& box_vertex_3d : modified_radar_box_vertices) {
    Eigen::Vector4d local_box_vertex = world2camera_pose * box_vertex_3d.homogeneous();
    Eigen::Vector2f temp_vertex = camera_intrinsic->Project(local_box_vertex.head(3).cast<float>());
    radar_box2d_vertices->push_back(temp_vertex.cast<double>());
  }
}

base::BaseCameraModelPtr TrackObjectDistance::QueryCameraModel(const SensorObjectConstPtr& camera) {
  return SensorDataManager::Instance()->GetCameraIntrinsic(camera->GetSensorId());
}
base::BaseCameraDistortionModelPtr TrackObjectDistance::QueryCameraDistortionModel(const SensorObjectConstPtr& camera) {
  return SensorDataManager::Instance()->GetCameraIntrinsicDistortion(camera->GetSensorId());
}
bool TrackObjectDistance::QueryWorld2CameraPose(const SensorObjectConstPtr& camera,
                                                Eigen::Matrix4d* pose) {
  Eigen::Affine3d camera2world_pose;
  bool status = SensorDataManager::Instance()->GetPose(camera->GetSensorId(),
                                                       camera->GetTimestamp(), &camera2world_pose);
  if (!status) {
    return false;
  }
  (*pose) = camera2world_pose.matrix().inverse();
  return true;
}

bool TrackObjectDistance::QueryLidar2WorldPose(const SensorObjectConstPtr& lidar,
                                               Eigen::Matrix4d* pose) {
  Eigen::Affine3d velo2world_pose;
  if (!lidar->GetRelatedFramePose(&velo2world_pose)) {
    return false;
  }
  (*pose) = velo2world_pose.matrix();
  return true;
}

ProjectionCacheObject* TrackObjectDistance::BuildProjectionCacheObject(
    const SensorObjectConstPtr& lidar,
    const SensorObjectConstPtr& camera,
    const base::BaseCameraDistortionModelPtr& camera_model,
    const std::string& measurement_sensor_id,
    double measurement_timestamp,
    const std::string& projection_sensor_id,
    double projection_timestamp) {
  // 1. get lidar2camera_pose
  Eigen::Matrix4d world2camera_pose;
  if (!QueryWorld2CameraPose(camera, &world2camera_pose)) {
    return nullptr;
  }
  Eigen::Matrix4d lidar2world_pose;
  if (!QueryLidar2WorldPose(lidar, &lidar2world_pose)) {
    return nullptr;
  }
  Eigen::Matrix4d lidar2camera_pose =
      static_cast<Eigen::Matrix<double, 4, 4, 0, 4, 4>>(world2camera_pose * lidar2world_pose);
  // 2. compute offset
  double time_diff = camera->GetTimestamp() - lidar->GetTimestamp();
  Eigen::Vector3d offset = lidar->GetBaseObject()->velocity.cast<double>() * time_diff;
  // 3. build projection cache
  const fusion::PointFCloud& cloud = lidar->GetBaseObject()->lidar_supplement.cloud;
  double width = static_cast<double>(camera_model->get_width());
  double height = static_cast<double>(camera_model->get_height());
  const int lidar_object_id = lidar->GetBaseObject()->id;
  ProjectionCacheObject* cache_object =
      projection_cache_.BuildObject(measurement_sensor_id, measurement_timestamp,
                                    projection_sensor_id, projection_timestamp, lidar_object_id);
  if (cache_object == nullptr) {
    ROS_ERROR("BuildProjectionCacheObject: Failed to build projection cache object!");
    return nullptr;
  }
  size_t start_ind = projection_cache_.GetPoint2dsSize();
  size_t end_ind = projection_cache_.GetPoint2dsSize();
  float xmin = std::numeric_limits<float>::max();
  float ymin = std::numeric_limits<float>::max();
  float xmax = -std::numeric_limits<float>::max();
  float ymax = -std::numeric_limits<float>::max();
  // 4. check whether all lidar's 8 3d vertices would projected outside frustum,
  // if not, build projection object of its cloud and cache it
  // else, build empty projection object and cache it
  bool is_all_lidar_3d_vertices_outside_frustum = false;
  if (cloud.size() > s_lidar2camera_projection_vertices_check_pts_num_) {
    is_all_lidar_3d_vertices_outside_frustum = true;
    std::vector<Eigen::Vector3d> lidar_box_vertices;
    GetObjectEightVertices(lidar->GetBaseObject(), &lidar_box_vertices);
    for (size_t i = 0; i < lidar_box_vertices.size(); ++i) {
      Eigen::Vector3d& vt = lidar_box_vertices[i];
      Eigen::Vector4d project_vt = static_cast<Eigen::Matrix<double, 4, 1, 0, 4, 1>>(
          world2camera_pose *
          Eigen::Vector4d(vt(0) + offset(0), vt(1) + offset(1), vt(2) + offset(2), 1.0));
      if (project_vt(2) <= 0) {
        continue;
      }
      Eigen::Vector2f project_vt2f = camera_model->Project(
          Eigen::Vector3f(static_cast<float>(project_vt(0)), static_cast<float>(project_vt(1)),
                          static_cast<float>(project_vt(2))));
      if (!IsPtInFrustum(project_vt2f, width, height)) {
        continue;
      }
      is_all_lidar_3d_vertices_outside_frustum = false;
      break;
    }
  }
  // 5. if not all lidar 3d vertices outside frustum, build projection object
  // of its cloud and cache it, else build & cache an empty one.
  if (!is_all_lidar_3d_vertices_outside_frustum) {
    // 5.1 check whehter downsampling needed
    size_t every_n = 1;
    if (cloud.size() > s_lidar2camera_projection_downsample_target_pts_num_) {
      every_n = cloud.size() / s_lidar2camera_projection_downsample_target_pts_num_;
    }
    for (size_t i = 0; i < cloud.size(); ++i) {
      if ((every_n > 1) && (i % every_n != 0)) {
        continue;
      }
      const fusion::PointF& pt = cloud.at(i);
      Eigen::Vector4d project_pt = static_cast<Eigen::Matrix<double, 4, 1, 0, 4, 1>>(
          lidar2camera_pose *
          Eigen::Vector4d(pt.x + offset(0), pt.y + offset(1), pt.z + offset(2), 1.0));
      if (project_pt(2) <= 0) {
        continue;
      }
      Eigen::Vector2f project_pt2f = camera_model->Project(
          Eigen::Vector3f(static_cast<float>(project_pt(0)), static_cast<float>(project_pt(1)),
                          static_cast<float>(project_pt(2))));
      if (!IsPtInFrustum(project_pt2f, width, height)) {
        continue;
      }
      if (project_pt2f.x() < xmin) {
        xmin = project_pt2f.x();
      }
      if (project_pt2f.y() < ymin) {
        ymin = project_pt2f.y();
      }
      if (project_pt2f.x() > xmax) {
        xmax = project_pt2f.x();
      }
      if (project_pt2f.y() > ymax) {
        ymax = project_pt2f.y();
      }
      projection_cache_.AddPoint(project_pt2f);
    }
  }
  end_ind = projection_cache_.GetPoint2dsSize();
  cache_object->SetStartInd(start_ind);
  cache_object->SetEndInd(end_ind);
  fusion::BBox2DF box = fusion::BBox2DF(xmin, ymin, xmax, ymax);
  cache_object->SetBox(box);
  return cache_object;
}

ProjectionCacheObject* TrackObjectDistance::QueryProjectionCacheObject(
    const SensorObjectConstPtr& lidar,
    const SensorObjectConstPtr& camera,
    const base::BaseCameraDistortionModelPtr& camera_model,
    const bool measurement_is_lidar) {
  // 1. try to query existed projection cache object
  const std::string& measurement_sensor_id =
      measurement_is_lidar ? lidar->GetSensorId() : camera->GetSensorId();
  const double measurement_timestamp =
      measurement_is_lidar ? lidar->GetTimestamp() : camera->GetTimestamp();
  const std::string& projection_sensor_id =
      measurement_is_lidar ? camera->GetSensorId() : lidar->GetSensorId();
  const double projection_timestamp =
      measurement_is_lidar ? camera->GetTimestamp() : lidar->GetTimestamp();
  const int lidar_object_id = lidar->GetBaseObject()->id;
  ProjectionCacheObject* cache_object =
      projection_cache_.QueryObject(measurement_sensor_id, measurement_timestamp,
                                    projection_sensor_id, projection_timestamp, lidar_object_id);
  if (cache_object != nullptr) {
    return cache_object;
  }  // 2. if query failed, build projection and cache it
  return BuildProjectionCacheObject(lidar, camera, camera_model, measurement_sensor_id,
                                    measurement_timestamp, projection_sensor_id,
                                    projection_timestamp);
}
void TrackObjectDistance::QueryProjectedVeloCtOnCamera(const SensorObjectConstPtr& velodyne64,
                                                       const SensorObjectConstPtr& camera,
                                                       const Eigen::Matrix4d& lidar2camera_pose,
                                                       Eigen::MatrixXd* polygon_image) {
  /*double time_diff = camera->GetTimestamp() - velodyne64->GetTimestamp();
  Eigen::Vector3d offset =
      velodyne64->GetBaseObject()->velocity.cast<double>() * time_diff;*/
  const Eigen::Vector3d& velo_ct = velodyne64->GetBaseObject()->center_ego;
  const Eigen::Vector3f& velosize = velodyne64->GetBaseObject()->size;
  PointCloud<PointD> polygon = velodyne64->GetBaseObject()->polygon_ego;
  Eigen::MatrixXd polygononimage(4, 3);
  double down = velo_ct[2] - velosize[2] * 0.5f;
  double up = velo_ct[2] + velosize[2] * 0.5f;
  std::sort(polygon.begin(), polygon.end(),
            [&](const perception::fusion::PointD& p1, const perception::fusion::PointD& p2) {
              return p1.y <= p2.y;
            });
  Eigen::Vector4d pt_c(velo_ct[0], velo_ct[1], velo_ct[2], 1.0);
  Eigen::Vector4d pt_lu(polygon[0].x, polygon[polygon.size() - 1].y, up, 1.0);
  Eigen::Vector4d pt_rd(polygon[polygon.size() - 1].x, polygon[0].y, down, 1.0);
  polygononimage.col(0) =
      static_cast<Eigen::Matrix<double, 4, 1, 0, 4, 1>>(lidar2camera_pose * pt_c);
  polygononimage.col(1) =
      static_cast<Eigen::Matrix<double, 4, 1, 0, 4, 1>>(lidar2camera_pose * pt_lu);
  polygononimage.col(2) =
      static_cast<Eigen::Matrix<double, 4, 1, 0, 4, 1>>(lidar2camera_pose * pt_rd);
  *polygon_image = polygononimage;
}

// @brief: compute the distance between lidar observation and
// camera observation
// @return distance of lidar vs. camera
float TrackObjectDistance::ComputeLidarCamera2D(const SensorObjectConstPtr& lidar,
                                              const SensorObjectConstPtr& camera,
                                              const bool is_track_id_consistent) {

  if (camera->GetBaseObject()->camera_supplement.has_3d == false) {
    // std::cout << " camera id = " << camera->GetBaseObject()->id << " no depth" << std::endl;
    return (std::numeric_limits<float>::max)();
  }
  // 计算两个similarity， 一个是zoy平面的iou，一个是中心点到lidar polygon的最近距离
  double center_distance = (lidar->GetBaseObject()->center - camera->GetBaseObject()->center).head(2).norm();
  if (center_distance > s_lidar2camera3d_association_center_dist_threshold_) {
    // std::cout << " camera id = " << camera->GetBaseObject()->id << " center_distance is = " << center_distance << std::endl;
    return (std::numeric_limits<float>::max)();
  }
  fusion::ObjectConstPtr lidar_obj, camera3d_obj;
  lidar_obj = lidar->GetBaseObject();
  camera3d_obj = camera->GetBaseObject();
  Eigen::Vector3f predict_velocity = lidar_obj->velocity;

  // 1.中心点到lidar polygon的最近距离-distance
  fusion::PointD PointO;
  PointO.x = camera3d_obj->position[0];
  PointO.y = camera3d_obj->position[1];
  PointO.z = camera3d_obj->position[2];
  double size_y = camera3d_obj->size[1];
  double size_z = camera3d_obj->size[2];

  double time_diff = camera->GetTimestamp() - lidar->GetTimestamp();

  float distance = s_lidar2camera3d_association_center_dist_threshold_;
  float distance_temp = s_lidar2camera3d_association_center_dist_threshold_;

  int nCount = lidar_obj->polygon.size();

  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::min();

  for (int idx = 0; idx < nCount; idx++) {

    fusion::PointD PointA, PointB;
    PointA.x = lidar_obj->polygon_utm.at(idx).x + predict_velocity[0] * time_diff;//utm坐标系，因为用的速度是绝对速度
    PointA.y = lidar_obj->polygon_utm.at(idx).y + predict_velocity[1] * time_diff;
    PointB.x = lidar_obj->polygon_utm.at((idx + 1) % nCount).x + predict_velocity[0] * time_diff;
    PointB.y = lidar_obj->polygon_utm.at((idx + 1) % nCount).y + predict_velocity[1] * time_diff;
    distance_temp = ComputerPointLinesegmentDistance(PointA, PointB, PointO);

    if (distance_temp < distance) {
      distance = distance_temp;
    }
    if (min_y > PointA.y) {
      min_y = PointA.y;
    }
    if (max_y < PointA.y) {
      max_y = PointA.y;
    }
  }

  // 2.计算zoy平面iou-iou_score
  PointCloud<PointD> camera_zoy;
  camera_zoy.push_back(PointD(PointO.z+size_z/2.0, PointO.y+size_y/2.0, 0.0, 0.0));
  camera_zoy.push_back(PointD(PointO.z+size_z/2.0, PointO.y-size_y/2.0, 0.0, 0.0));
  camera_zoy.push_back(PointD(PointO.z-size_z/2.0, PointO.y-size_y/2.0, 0.0, 0.0));
  camera_zoy.push_back(PointD(PointO.z-size_z/2.0, PointO.y+size_y/2.0, 0.0, 0.0));

  PointCloud<PointD> lidar_zoy;
  lidar_zoy.push_back(PointD(lidar_obj->position[2]+lidar_obj->size[2]/2.0, max_y, 0.0, 0.0));
  lidar_zoy.push_back(PointD(lidar_obj->position[2]+lidar_obj->size[2]/2.0, min_y, 0.0, 0.0));
  lidar_zoy.push_back(PointD(lidar_obj->position[2]-lidar_obj->size[2]/2.0, min_y, 0.0, 0.0));
  lidar_zoy.push_back(PointD(lidar_obj->position[2]-lidar_obj->size[2]/2.0, max_y, 0.0, 0.0));

  IOU iou;

  double iou_score = iou.Compute(0, 0, camera_zoy, lidar_zoy);

  // 3.将两个距离糅合成一个
  double dis_similarity = 1.0 - distance/s_lidar2camera3d_association_center_dist_threshold_;
  std::vector<double> multiple_similarities = {
      dis_similarity, iou_score
  };
  double fused_similarity = FuseMultipleProbabilities(multiple_similarities);

  distance = distance_thresh_ * static_cast<float>(1.0 - fused_similarity) /
             (1.0f - rc_similarity2distance_penalize_thresh_);//随便写写
  // std::cout << "track id = " << fused_track_->GetTrackId() << ", camera id = " << camera3d_obj->id
  //           << ", dis = " << distance << ", iou_score = " << iou_score << ", dis_similarity = " << dis_similarity
  //           << ", fused_similarity = " << fused_similarity << std::endl;
  return distance;
}

bool TrackObjectDistance::QueryPolygonDCenter(const fusion::ObjectConstPtr& object,
                                              const Eigen::Vector3d& ref_pos,
                                              const int range,
                                              Eigen::Vector3d* polygon_ct) {
  if (object == nullptr) {
    return false;
  }
  const fusion::PolygonDType& polygon = object->polygon_utm;
  if (!ComputePolygonCenter(polygon, polygon_ct)) {
    return false;
  }
  return true;
}

bool TrackObjectDistance::IsTrackIdConsistent(const SensorObjectConstPtr& object1,
                                              const SensorObjectConstPtr& object2) {
  if (object1 == nullptr || object2 == nullptr) {
    return false;
  }
  if (object1->GetBaseObject()->track_id == object2->GetBaseObject()->track_id) {
    return true;
  }
  return false;
}

bool TrackObjectDistance::LidarCameraCenterDistanceExceedDynamicThreshold(
    const SensorObjectConstPtr& lidar,
    const SensorObjectConstPtr& camera) {
  double center_distance =
      (lidar->GetBaseObject()->center - camera->GetBaseObject()->center).head(2).norm();
  double local_distance = 60;
  const fusion::PointFCloud& cloud = lidar->GetBaseObject()->lidar_supplement.cloud;
  if (cloud.size() > 0) {
    const fusion::PointF& pt = cloud.at(0);
    local_distance = std::sqrt(pt.x * pt.x + pt.y * pt.y);
  }
  double dynamic_threshold = 5 + 0.15 * local_distance;  // no points , dynamic_threshold is 14
  if (center_distance > dynamic_threshold) {
    return true;
  }
  return false;
}

void TrackObjectDistance::MakeSensorMap(const TrackPtr& fused_track) {
  sensor_map_["lidar"] = fused_track->GetLatestLidarObject();

  sensor_map_["radar"] = fused_track->GetLatestRadarObject();

  sensor_map_["camera"] = fused_track->GetLatestCameraObject();
  // Modify @jiangnan : add falcon lidar
  sensor_map_["falcon"] = fused_track->GetLatestFalconLidarObject();
  // Modify(@liuxinyu): obu_test
  sensor_map_["obu"] = fused_track->GetLatestObuObject();
  sensor_map_["vidar"] = fused_track->GetLatestVidarObject();
  // init distance threshold
  s_lidar2vidar_association_center_dist_threshold_ = 15;
}

// @brief: compute the distance between input fused track and sensor object
// @return track object distance
float TrackObjectDistance::Compute(const TrackPtr& fused_track,
                                   const SensorObjectPtr& sensor_object,
                                   const TrackObjectDistanceOptions& options) {
  FusedObjectPtr fused_object = fused_track->GetFusedObject();
  fused_track_ = fused_track;
  if (fused_object == nullptr) {
    ROS_ERROR("Compute: fused object is nullptr!");
    return (std::numeric_limits<float>::max)();
  }
  Eigen::Vector3d* ref_point = options.ref_point;
  if (ref_point == nullptr) {
    ROS_ERROR("Compute: reference point is nullptr!");
    return (std::numeric_limits<float>::max)();
  }
  float distance = (std::numeric_limits<float>::max)();
  float min_distance = (std::numeric_limits<float>::max)();
  // create fused history sensor map
  MakeSensorMap(fused_track);
  if (IsLidar(sensor_object)) {
    min_distance = ComputeLidarFused(sensor_object, ref_point);
  } else if (IsFalconLidar(sensor_object)) {
    min_distance = ComputeFalconLidarFused(sensor_object, ref_point);
  } else if (IsRadar(sensor_object)) {
    min_distance = ComputeRadarFused(sensor_object, ref_point);
  } else if (IsCamera(sensor_object)) {
    min_distance = ComputeCamera2DFused(sensor_object, ref_point);
  } else if (IsObu(sensor_object)) {
    min_distance = ComputeObuFused(sensor_object, ref_point);
  } else if (IsVidar(sensor_object)) {
    // not fused with camera
    min_distance = ComputeVidarFused(sensor_object, ref_point);
  } else {
    ROS_ERROR("Compute: fused sensor type is not support!");
  }
  return min_distance;
}

float TrackObjectDistance::ComputeLidarFused(const SensorObjectPtr& sensor_object,
                                             Eigen::Vector3d* ref_point) {
  float distance = (std::numeric_limits<float>::max)();
  float min_distance = (std::numeric_limits<float>::max)();
  if (sensor_map_["lidar"] != nullptr) {
    distance = ComputeLidarLidar(sensor_map_["lidar"], sensor_object, *ref_point);
    min_distance = std::min(distance, min_distance);
  }
  if (sensor_map_["falcon"] != nullptr) {
    distance = ComputeLidarLidar(sensor_map_["falcon"], sensor_object, *ref_point);
    min_distance = std::min(distance, min_distance);
  }
  if (sensor_map_["obu"] != nullptr) {
    distance = ComputeLidarObu(sensor_map_["obu"], sensor_object, *ref_point);
    min_distance = std::min(distance, min_distance);
  }
  if (sensor_map_["vidar"] != nullptr) {
        distance = ComputeLidarVidar(sensor_map_["vidar"], sensor_object, *ref_point,
                                 s_lidar2vidar_association_center_dist_threshold_);
    min_distance = std::min(distance, min_distance);
  }
  // only the radar tracker: calculate cost distance between lidar measurement and latest radar object
  if (sensor_map_["radar"] != nullptr && sensor_map_["lidar"] == nullptr && sensor_map_["falcon"] == nullptr) {
    distance = ComputeLidarRadar(sensor_map_["radar"], sensor_object, *ref_point);
    min_distance = std::min(distance, min_distance);
  }
  return min_distance;
}

float TrackObjectDistance::ComputeFalconLidarFused(const SensorObjectPtr& sensor_object,
                                                   Eigen::Vector3d* ref_point) {
  float distance = (std::numeric_limits<float>::max)();
  float min_distance = (std::numeric_limits<float>::max)();
  if (sensor_map_["lidar"] != nullptr) {
    distance = ComputeLidarLidar(sensor_map_["lidar"], sensor_object, *ref_point);
    min_distance = std::min(distance, min_distance);
  }
  if (sensor_map_["falcon"] != nullptr) {
    distance = ComputeLidarLidar(sensor_map_["falcon"], sensor_object, *ref_point);
    min_distance = std::min(distance, min_distance);
  }
  if (sensor_map_["vidar"] != nullptr) {
        distance = ComputeLidarVidar(sensor_map_["vidar"], sensor_object, *ref_point,
                                 s_lidar2vidar_association_center_dist_threshold_);
    min_distance = std::min(distance, min_distance);
  }
  if (sensor_map_["obu"] != nullptr) {
    distance = ComputeLidarObu(sensor_map_["obu"], sensor_object, *ref_point);
    min_distance = std::min(distance, min_distance);
  }
  return min_distance;
}

float TrackObjectDistance::ComputeCamera2DFused(const SensorObjectPtr& sensor_object,
                                                Eigen::Vector3d* ref_point) {
  float distance = (std::numeric_limits<float>::max)();
  float min_distance = (std::numeric_limits<float>::max)();
  if (fused_track_->GetFusedObject()->GetBaseObject()->center_ego.x() <= 1)
    return min_distance;

  bool islidarcomputed =
      false;  //@lijian,如果已经和lidar的目标计算了投影匹配关系则不再和图达通的，二者选其一
  if (sensor_map_["lidar"] != nullptr) {
    bool is_camera_track_id_consistent = IsTrackIdConsistent(sensor_map_["camera"], sensor_object);
    distance = ComputeLidarCamera(sensor_map_["lidar"], sensor_object, false,
                                  is_camera_track_id_consistent);
    min_distance = std::min(distance, min_distance);
    islidarcomputed = true;
  }
  if (sensor_map_["falcon"] != nullptr && !islidarcomputed) {
    bool is_camera_track_id_consistent = IsTrackIdConsistent(sensor_map_["camera"], sensor_object);
    distance = ComputeLidarCamera(sensor_map_["falcon"], sensor_object, false,
                                  is_camera_track_id_consistent);
    min_distance = std::min(distance, min_distance);
  }
  return min_distance;
}

float TrackObjectDistance::ComputeObuFused(const SensorObjectPtr& sensor_object,
                                           Eigen::Vector3d* ref_point) {
  float distance = (std::numeric_limits<float>::max)();
  float min_distance = (std::numeric_limits<float>::max)();
  if (sensor_map_["falcon"] != nullptr) {
    distance = ComputeLidarObu(sensor_map_["falcon"], sensor_object, *ref_point);
    min_distance = std::min(distance, min_distance);
  }
  if (sensor_map_["lidar"] != nullptr) {
    distance = ComputeLidarObu(sensor_map_["lidar"], sensor_object, *ref_point);
    min_distance = std::min(distance, min_distance);
  }
  if (sensor_map_["obu"] != nullptr) {
    distance = ComputeObuObu(sensor_map_["obu"], sensor_object, *ref_point);
    min_distance = std::min(distance, min_distance);
  }
  if (sensor_map_["vidar"] != nullptr) {
    distance = ComputeVidarObu(sensor_map_["vidar"], sensor_object, *ref_point, 
                                          s_vidar2obu_association_center_dist_threshold_);
    min_distance = std::min(distance, min_distance);
  }
  if (sensor_map_["radar"] != nullptr) {
    distance = ComputeRadarObu(sensor_map_["radar"], sensor_object, *ref_point);
    min_distance = std::min(distance, min_distance);
  }
  return min_distance;
}

float TrackObjectDistance::ComputeRadarFused(const SensorObjectPtr& sensor_object,
                                             Eigen::Vector3d* ref_point) {
  float distance = (std::numeric_limits<float>::max)();
  float min_distance = (std::numeric_limits<float>::max)();
  if (sensor_map_["lidar"] != nullptr) {
    distance = ComputeLidarRadar(sensor_map_["lidar"], sensor_object, *ref_point);
    min_distance = std::min(distance, min_distance);
  }
  if (sensor_map_["falcon"] != nullptr) {
    distance = ComputeLidarRadar(sensor_map_["falcon"], sensor_object, *ref_point);
    min_distance = std::min(distance, min_distance);
  }
  if (sensor_map_["obu"] != nullptr) {
    // 复用LidarRadar计算方式
    distance = ComputeLidarRadar(sensor_map_["obu"], sensor_object, *ref_point); 
    min_distance = std::min(distance, min_distance);
  }
  if (sensor_map_["vidar"] != nullptr) {
    distance = ComputeLidarRadar(sensor_map_["vidar"], sensor_object, *ref_point);
    min_distance = std::min(distance, min_distance);
  }
  if (sensor_map_["radar"] != nullptr) {
    distance = ComputeRadarRadar(sensor_map_["radar"], sensor_object, *ref_point);
    min_distance = std::min(distance, min_distance);
  }

 // Exception radar filtering : abnormal position;
  float x_dis = sensor_object->GetBaseObject()->center_ego.x();
  if (min_distance > 4 || (x_dis > 0 && min_distance > 2.5))
    return 4.0;

  // Modify @jiangnan: add distance cost  (use speed difference  between  
  // radar measurement velocity and last velocity)
  float speed_diff = 0.0;
  const SensorObjectConstPtr last_radar_object = sensor_map_["radar"];
  float last_speed = 0.0;
  if (last_radar_object != nullptr) {
    // use last radar_object velocity
    speed_diff = (last_radar_object->GetBaseObject()->velocity -
                  sensor_object->GetBaseObject()->velocity).norm();
  } else {
    // use last fusion_object velocity
    speed_diff = (fused_track_->GetFusedObject()->GetBaseObject()->velocity -
                  sensor_object->GetBaseObject()->velocity).norm();
  }

  // Exception radar filtering : abnormal velocity;
  if (speed_diff > 6.0)
    return 4.0;

  float center_distance = min_distance;
  // center_density [0,1]: describes the probability of  center_point_distance
  float center_density = 1 / (1 + (exp(2.0 * (center_distance - 3.0))));

  // speed_density [0,1]:
  float speed_density =  1 / (1 + (exp(2.0 * (speed_diff - 3.0))));
 // compute synthesis probability
  float result_pro_density = 0.65 * center_density + 0.35 * speed_density;
 // calculate the cost distance by probability
  float result_distance = 4 - result_pro_density;

  return result_distance;
}

float TrackObjectDistance::ComputeVidarFused(const SensorObjectPtr& sensor_object,
                                             Eigen::Vector3d* ref_point) {
  float distance = (std::numeric_limits<float>::max)();
  float min_distance = (std::numeric_limits<float>::max)();
  if (sensor_map_["lidar"] != nullptr) {
    distance = ComputeLidarVidar(sensor_map_["lidar"], sensor_object, *ref_point,
                                 s_lidar2vidar_association_center_dist_threshold_);
    min_distance = std::min(distance, min_distance);
  }
  if (sensor_map_["falcon"] != nullptr) {
    distance = ComputeLidarVidar(sensor_map_["falcon"], sensor_object, *ref_point,
                                 s_lidar2vidar_association_center_dist_threshold_);
    min_distance = std::min(distance, min_distance);
  }
    if (sensor_map_["vidar"] != nullptr) {
    distance = ComputeVidarVidar(sensor_map_["vidar"], sensor_object, *ref_point,
                                 s_lidar2vidar_association_center_dist_threshold_);
    min_distance = std::min(distance, min_distance);
  }
  if (min_distance >= distance_thresh_) {
    if (sensor_map_["radar"] != nullptr) {
      distance = ComputeLidarRadar(sensor_map_["radar"], sensor_object, *ref_point);
      min_distance = std::min(distance, min_distance);
    }
  }

  if (sensor_map_["obu"] != nullptr) {
    // 复用LidarVidar计算方式
    distance = ComputeLidarVidar(sensor_map_["obu"], sensor_object, *ref_point,
                                s_vidar2obu_association_center_dist_threshold_);
    min_distance = std::min(distance, min_distance);
  }

  return min_distance;
}

// @brief: compute the distance between velodyne64 observation and
// velodyne64 observation
// @return the distance of velodyne64 vs. velodyne64
float TrackObjectDistance::ComputeLidarLidar(const SensorObjectConstPtr& fused_object,
                                             const SensorObjectPtr& sensor_object,
                                             const Eigen::Vector3d& ref_pos,
                                             int range) {
  double center_distance =
      (sensor_object->GetBaseObject()->center - fused_object->GetBaseObject()->center)
          .head(2)
          .norm();
  if (center_distance > s_lidar2lidar_association_center_dist_threshold_) {
    ROS_DEBUG_STREAM(
        "ComputeLidarLidar: center distance exceed lidar2lidar tight "
        "threshold: "
        << "center_dist@" << center_distance << ", "
        << "tight_threh@" << s_lidar2lidar_association_center_dist_threshold_);
    return (std::numeric_limits<float>::max)();
  }
  center_distance =
      ComputePolygonDistance3d(fused_object, sensor_object, ref_pos, range);

  // center_density [0,1]: describes the probability of  center_point_distance
  float center_density = 1 / (1 + (exp(2.0 * (center_distance - 3.0))));

  // iou_density [0,1]: describes the probability of IoU
  float iou_density = ComputePolygonIOU(fused_object, sensor_object);

  //type_density [0,1]:
  float type_density =
      ComputeTypeProbality(fused_object->GetBaseObject()->type, sensor_object->GetBaseObject()->type);

  float iou_factor = 0.45;
  float center_factor = 0.45;

  // set the coefficient according to the distance
  float abs_distence = std::sqrt(std::pow(sensor_object->GetBaseObject()->center[0], 2) +
                                 std::pow(sensor_object->GetBaseObject()->center[1], 2));
  if (abs_distence < s_lidar2lidar_coefficient_dist_threshold_) {
    iou_factor = 0.55;
    center_factor = 0.35;
  }

  float result_pro_density = center_factor * center_density + iou_factor * iou_density + 0.1 * type_density;

  //map probability values to the distance space 
  float distance = ((1 - result_pro_density) * 4) * 1.35443;

  return distance;
}

// Modify @jiangnan :;computer match distance  between polygon and  radar center
float TrackObjectDistance::ComputeLidarRadar(const SensorObjectConstPtr& fused_object,
                                             const SensorObjectPtr& sensor_object,
                                             const Eigen::Vector3d& ref_pos,
                                             int range) {
  double center_distance =
      (sensor_object->GetBaseObject()->center - fused_object->GetBaseObject()->center)
          .head(2)
          .norm();
  if (center_distance > s_lidar2radar_association_center_dist_threshold_) {
    ROS_DEBUG_STREAM(
        "ComputeLidarRadar: center distance exceed lidar2radar tight "
        "threshold: "
        << "center_dist@" << center_distance << ", "
        << "tight_threh@" << s_lidar2radar_association_center_dist_threshold_);
    return (std::numeric_limits<float>::max)();
  }
  fusion::ObjectConstPtr lidar_obj, radar_obj;
  Eigen::Vector3f predict_velocity= Eigen::Vector3f(1.0f, 0.0f, 0.0f);
  if (IsRadar(sensor_object)) {
    lidar_obj = fused_object->GetBaseObject();
    radar_obj = sensor_object->GetBaseObject();
    predict_velocity = lidar_obj->velocity;
  } else {
    lidar_obj = sensor_object->GetBaseObject();
    radar_obj = fused_object->GetBaseObject();
  }

  fusion::PointD PointO;
  PointO.x = radar_obj->position[0];
  PointO.y = radar_obj->position[1];
  PointO.z = 0.0;

  double fusion_timestamp = fused_object->GetTimestamp();
  double sensor_timestamp = sensor_object->GetTimestamp();
  double time_diff = sensor_timestamp - fusion_timestamp;

  float distance = s_lidar2radar_association_center_dist_threshold_;
  float distance_temp = s_lidar2radar_association_center_dist_threshold_;
                     
  for (size_t idx = 0; idx < fused_object->GetBaseObject()->polygon.size(); ++idx) {
    if (idx == fused_object->GetBaseObject()->polygon_utm.size() - 1) {
      fusion::PointD PointA, PointB;
      PointA.x = lidar_obj->polygon_utm.at(idx).x + predict_velocity[0] * time_diff;
      PointA.y = lidar_obj->polygon_utm.at(idx).y + predict_velocity[1] * time_diff;
      PointB.x = lidar_obj->polygon_utm.at(0).x + predict_velocity[0] * time_diff;
      PointB.y = lidar_obj->polygon_utm.at(0).y + predict_velocity[1] * time_diff;
      distance_temp = ComputerPointLinesegmentDistance(PointA, PointB, PointO);
    } else {
      fusion::PointD PointA, PointB;
      PointA.x = lidar_obj->polygon_utm.at(idx).x + predict_velocity[0] * time_diff;
      PointA.y = lidar_obj->polygon_utm.at(idx).y + predict_velocity[1] * time_diff;
      PointB.x = lidar_obj->polygon_utm.at(idx + 1).x + predict_velocity[0] * time_diff;
      PointB.y = lidar_obj->polygon_utm.at(idx + 1).y + predict_velocity[1] * time_diff;
      distance_temp = ComputerPointLinesegmentDistance(PointA, PointB, PointO);
    }
    if (distance_temp < distance)
      distance = distance_temp;
  }
  return distance;
}

float TrackObjectDistance::ComputeLidarObu(const SensorObjectConstPtr& fused_object,
                                           const SensorObjectPtr& sensor_object,
                                           const Eigen::Vector3d& ref_pos,
                                           int range) {
  double center_distance =
      (sensor_object->GetBaseObject()->center - fused_object->GetBaseObject()->center)
          .head(2)
          .norm();
  if (center_distance > s_lidar2obu_association_center_dist_threshold_) {
    ROS_DEBUG_STREAM("ComputeLidarObu: center distance exceed lidar2obu tight threshold: "
                     << "center_dist@" << center_distance << ", "
                     << "tight_threh@" << s_lidar2obu_association_center_dist_threshold_);
    return (std::numeric_limits<float>::max)();
  }
  float distance = ComputePolygonDistance3d(fused_object, sensor_object, ref_pos, range);
  ROS_DEBUG_STREAM("ComputeLidarObu: distance: " << distance);
  return distance;
}

float TrackObjectDistance::ComputeObuObu(const SensorObjectConstPtr& fused_object,
                                         const SensorObjectPtr& sensor_object,
                                         const Eigen::Vector3d& ref_pos,
                                         int range) {
  double center_distance =
      (sensor_object->GetBaseObject()->center - fused_object->GetBaseObject()->center)
          .head(2)
          .norm();
  if (center_distance > s_lidar2obu_association_center_dist_threshold_) {
    ROS_DEBUG_STREAM("center distance exceed lidar2obu tight threshold: "
                     << "center_dist@" << center_distance << ", "
                     << "tight_threh@" << s_lidar2obu_association_center_dist_threshold_);
    return (std::numeric_limits<float>::max)();
  }
  float distance = ComputePolygonDistance3d(fused_object, sensor_object, ref_pos, range);
  ROS_DEBUG_STREAM("ComputeObuObu: distance: " << distance);
  return distance;
}

float TrackObjectDistance::ComputeRadarObu(const SensorObjectConstPtr& fused_object,
                                           const SensorObjectPtr& sensor_object,
                                           const Eigen::Vector3d& ref_pos,
                                           int range) {
  double center_distance =
      (sensor_object->GetBaseObject()->center - fused_object->GetBaseObject()->center)
          .head(2)
          .norm();
  if (center_distance > s_radar2obu_association_center_dist_threshold_) {
    ROS_DEBUG_STREAM("ComputeRadarObu: center distance exceed radar2obu tight threshold: "
                     << "center_dist@" << center_distance << ", "
                     << "tight_threh@" << s_radar2obu_association_center_dist_threshold_);
    return (std::numeric_limits<float>::max)();
  }
  float distance = ComputePolygonDistance3d(fused_object, sensor_object, ref_pos, range);
  ROS_DEBUG_STREAM("ComputeRadarObu: distance: " << distance);
  return distance;
}

float TrackObjectDistance::ComputeVidarObu(const SensorObjectConstPtr& fused_object,
                                          const SensorObjectPtr& sensor_object,
                                          const Eigen::Vector3d& ref_pos,
                                          double center_distance_thresh,
                                          int range) { 
  float distance =
      ComputeVidarandOtherDistance(fused_object, sensor_object, ref_pos, center_distance_thresh);
  return distance;
}

float TrackObjectDistance::ComputeRadarRadar(const SensorObjectConstPtr& fused_object,
                                             const SensorObjectPtr& sensor_object,
                                             const Eigen::Vector3d& ref_pos,
                                             int range) {
  double center_distance =
      (sensor_object->GetBaseObject()->center - fused_object->GetBaseObject()->center)
          .head(2)
          .norm();
  if (center_distance > s_radar2radar_association_center_dist_threshold_) {
    ROS_DEBUG_STREAM(
        "ComputeRadarRadar: center distance exceed radar2radar tight "
        "threshold: "
        << "center_dist@" << center_distance << ", "
        << "tight_threh@" << s_radar2radar_association_center_dist_threshold_);
    return (std::numeric_limits<float>::max)();
  }
  // float distance = ComputePolygonDistance3d(fused_object, sensor_object, ref_pos, range);

  const fusion::ObjectConstPtr& obj_f = fused_object->GetBaseObject();
  Eigen::Vector3d fused_poly_center(0, 0, 0);

  const fusion::ObjectConstPtr obj_s = sensor_object->GetBaseObject();
  Eigen::Vector3d sensor_poly_center(obj_s->position[0], obj_s->position[1], obj_s->position[2]);

  double fusion_timestamp = fused_object->GetTimestamp();
  double sensor_timestamp = sensor_object->GetTimestamp();
  double time_diff = sensor_timestamp - fusion_timestamp;
  fused_poly_center[0] = obj_f->position[0] + obj_f->velocity(0) * time_diff;
  fused_poly_center[1] = obj_f->position[1] + obj_f->velocity(1) * time_diff;
  float distance = ComputeEuclideanDistance(fused_poly_center, sensor_poly_center);

  ROS_DEBUG_STREAM("ComputeRadarRadar: distance: " << distance);
  return distance;
}

float TrackObjectDistance::ComputeLidarVidar(const SensorObjectConstPtr& fused_object,
                                             const SensorObjectPtr& sensor_object,
                                             const Eigen::Vector3d& ref_pos,
                                             double center_distance_thresh,
                                             int range) {
  float distance =
      ComputeVidarandOtherDistance(fused_object, sensor_object, ref_pos, center_distance_thresh);
  return distance;
}

float TrackObjectDistance::ComputeObuVidar(const SensorObjectConstPtr& fused_object,
                                           const SensorObjectPtr& sensor_object,
                                           const Eigen::Vector3d& ref_pos,
                                           double center_distance_thresh,
                                           int range) {
  float distance =
      ComputeVidarandOtherDistance(fused_object, sensor_object, ref_pos, center_distance_thresh);
  return distance;
}

float TrackObjectDistance::ComputeRadarVidar(const SensorObjectConstPtr& fused_object,
                                             const SensorObjectPtr& sensor_object,
                                             const Eigen::Vector3d& ref_pos,
                                             double center_distance_thresh,
                                             int range) {
  float distance =
      ComputeVidarandOtherDistance(fused_object, sensor_object, ref_pos, center_distance_thresh);
  return distance;
}

float TrackObjectDistance::ComputeVidarVidar(const SensorObjectConstPtr& fused_object,
                                             const SensorObjectPtr& sensor_object,
                                             const Eigen::Vector3d& ref_pos,
                                             double center_distance_thresh,
                                             int range) {
  float distance =
      ComputeVidarandOtherDistance(fused_object, sensor_object, ref_pos, center_distance_thresh);
  return distance;
}
float TrackObjectDistance::ComputeVidarandOtherDistance(const SensorObjectConstPtr& fused_object,
                                                        const SensorObjectPtr& sensor_object,
                                                        const Eigen::Vector3d& ref_pos,
                                                        double center_distance_thresh,
                                                        int range) {
  double center_distance =
      (sensor_object->GetBaseObject()->position - fused_object->GetBaseObject()->position)
          .head(2)
          .norm();
  if (center_distance > center_distance_thresh) {
    ROS_DEBUG_STREAM("ComputeSensor2Sensor: center distance exceed Sensor2Sensor tight threshold: "
                     << "center_dist@" << center_distance << ", "
                     << "tight_threh@" << center_distance_thresh);
    return (std::numeric_limits<float>::max)();
  }

  const fusion::ObjectConstPtr& obj_f = fused_object->GetBaseObject();
  Eigen::Vector3d fused_poly_center(0, 0, 0);

  const fusion::ObjectConstPtr obj_s = sensor_object->GetBaseObject();
  Eigen::Vector3d sensor_poly_center(obj_s->position[0], obj_s->position[1], obj_s->position[2]);

  double fusion_timestamp = fused_object->GetTimestamp();
  double sensor_timestamp = sensor_object->GetTimestamp();
  double time_diff = sensor_timestamp - fusion_timestamp;
  fused_poly_center[0] = obj_f->position[0] + obj_f->velocity(0) * time_diff;
  fused_poly_center[1] = obj_f->position[1] + obj_f->velocity(1) * time_diff;
  float distance = ComputeEuclideanDistance(fused_poly_center, sensor_poly_center);
  return distance;
}
//@lijian,计算传感器之间的匹配代价（vidar和lidar的）
float TrackObjectDistance::ComputeSensor2Sensor(const SensorObjectConstPtr& fused_object,
                                                const SensorObjectPtr& sensor_object,
                                                const Eigen::Vector3d& ref_pos,
                                                double center_distance_thresh,
                                                int range) {
  double center_distance =
      (sensor_object->GetBaseObject()->position - fused_object->GetBaseObject()->position)
          .head(2)
          .norm();
  if (center_distance > center_distance_thresh) {
    ROS_WARN_STREAM("ComputeSensor2Sensor: center distance exceed Sensor2Sensor tight threshold: "
                    << "center_dist@" << center_distance << ", "
                    << "tight_threh@" << center_distance_thresh);
    return (std::numeric_limits<float>::max)();
  }
  float distance = ComputePolygonDistance3d(fused_object, sensor_object, ref_pos, range);
  ROS_DEBUG_STREAM("ComputeSensor2Sensor: distance: " << distance);
  return distance;
}

float TrackObjectDistance::ComputeSensorCamera(const SensorObjectConstPtr& sensor,
                                               const SensorObjectConstPtr& camera) {
  float distance = (std::numeric_limits<float>::max)();
  // 1. get camera model and pose
  base::BaseCameraModelPtr camera_model = QueryCameraModel(camera);
  if (camera_model == nullptr) {
    ROS_ERROR_STREAM("ComputeSensorCamera: Failed to get camera model for "
                     << camera->GetSensorId());
    return distance;
  }
  Eigen::Matrix4d world2camera_pose;
  if (!QueryWorld2CameraPose(camera, &world2camera_pose)) {
    return distance;
  }
  // get camera useful information
  const fusion::BBox2DF& camera_bbox = camera->GetBaseObject()->camera_supplement.box;
  const fusion::Point2DF camera_bbox_ct = camera_bbox.Center();
  const Eigen::Vector2d box2d_ct = Eigen::Vector2d(camera_bbox_ct.x, camera_bbox_ct.y);
  Eigen::Vector2d box2d_size =
      Eigen::Vector2d(camera_bbox.xmax - camera_bbox.xmin, camera_bbox.ymax - camera_bbox.ymin);
  box2d_size = box2d_size.cwiseMax(rc_min_box_size_);
  double width = static_cast<double>(camera_model->get_width());
  double height = static_cast<double>(camera_model->get_height());
  // get radar useful information
  double time_diff = camera->GetTimestamp() - sensor->GetTimestamp();
  Eigen::Vector3d offset = sensor->GetBaseObject()->velocity.cast<double>() * time_diff;
  offset.setZero();
  Eigen::Vector3d sensor_ct = sensor->GetBaseObject()->center + offset;
  Eigen::Vector4d local_pt = static_cast<Eigen::Matrix<double, 4, 1, 0, 4, 1>>(
      world2camera_pose * Eigen::Vector4d(sensor_ct[0], sensor_ct[1], sensor_ct[2], 1.0));
  std::vector<Eigen::Vector3d> sensor_box_vertices;
  GetObjectEightVertices(sensor->GetBaseObject(), &sensor_box_vertices);
  std::vector<Eigen::Vector2d> sensor_box2d_vertices;
  GetModified2DRadarBoxVertices(sensor_box_vertices, camera, camera_model, world2camera_pose,
                                &sensor_box2d_vertices);
  // compute similarity
  double fused_similarity = 0.0;
  if (local_pt[2] > 0) {
    Eigen::Vector3f pt3f;
    pt3f << camera_model->Project(Eigen::Vector3f(static_cast<float>(local_pt[0]),
                                                  static_cast<float>(local_pt[1]),
                                                  static_cast<float>(local_pt[2]))),
        static_cast<float>(local_pt[2]);
    Eigen::Vector3d pt3d = pt3f.cast<double>();
    if (IsPtInFrustum(pt3d, width, height)) {
      // compute similarity on x direction
      double x_similarity = ComputeRadarCameraXSimilarity(pt3d.x(), box2d_ct.x(), box2d_size.x(),
                                                          rc_x_similarity_params_);
      // compute similarity on y direction
      double y_similarity = ComputeRadarCameraYSimilarity(pt3d.y(), box2d_ct.y(), box2d_size.y(),
                                                          rc_y_similarity_params_);
      // compute similarity on height
      // use camera car height to modify the radar location
      // double h_similarity = ComputeRadarCameraHSimilarity(
      //     radar, camera, box2d_size.y(), radar_box2d_vertices,
      //     rc_h_similarity_params_);
      // compute similarity on width
      // double w_similarity = ComputeRadarCameraWSimilarity(
      //     radar, width, box2d_size.x(), radar_box2d_vertices,
      //     rc_w_similarity_params_);
      // compute similarity on offset 3d
      double loc_similarity = ComputeRadarCameraLocSimilarity(sensor_ct, camera, world2camera_pose,
                                                              rc_loc_similarity_params_);
      // compute similarity on velocity
      double velocity_similarity = ComputeRadarCameraVelocitySimilarity(sensor, camera);
      // fuse similarity
      std::vector<double> multiple_similarities = {
          x_similarity, y_similarity, loc_similarity, velocity_similarity
          // height_similarity, width_similarity,
      };
      fused_similarity = FuseMultipleProbabilities(multiple_similarities);
    }
  }
  distance = distance_thresh_ * static_cast<float>(1.0 - fused_similarity) /
             (1.0f - rc_similarity2distance_penalize_thresh_);
  //  ROS_DEBUG_STREAM("ComputeRadarCamera distance: " << distance);
  return distance;
}

// @brief: compute the distance between lidar observation and
// camera observation
// @return distance of lidar vs. camera
float TrackObjectDistance::ComputeLidarCamera(const SensorObjectConstPtr& lidar,
                                              const SensorObjectConstPtr& camera,
                                              const bool measurement_is_lidar,
                                              const bool is_track_id_consistent) {
  float distance = distance_thresh_;
  // 1. get camera intrinsic and pose
  base::BaseCameraDistortionModelPtr camera_model = QueryCameraDistortionModel(camera);
  // base::BaseCameraModelPtr camera_model = QueryCameraModel(camera);
  if (camera_model == nullptr) {
    ROS_ERROR_STREAM("ComputeLidarCamera: Failed to get camera model for "
                     << camera->GetSensorId());
    return distance;
  }
  Eigen::Matrix4d world2camera_pose;
  if (!QueryWorld2CameraPose(camera, &world2camera_pose)) {
    ROS_ERROR("ComputeLidarCamera: Failed to query camera pose!");
    return distance;
  }
  Eigen::Matrix4d lidar2world_pose;
  if (!QueryLidar2WorldPose(lidar, &lidar2world_pose)) {
    ROS_ERROR("ComputeLidarCamera: Failed to query lidar pose!");
    return distance;
  }
  Eigen::Matrix4d lidar2camera_pose =
      static_cast<Eigen::Matrix<double, 4, 4, 0, 4, 4>>(world2camera_pose * lidar2world_pose);
  // 2. compute distance of camera vs. lidar observation
  // const fusion::PointFCloud& cloud =
  //     lidar->GetBaseObject()->lidar_supplement.cloud;
  const fusion::BBox2DF& camera_bbox = camera->GetBaseObject()->camera_supplement.box;
  const fusion::Point2DF camera_bbox_ct = camera_bbox.Center();  // bbox center exit here-syf
  const Eigen::Vector2f box2f_ct = Eigen::Vector2f(camera_bbox_ct.x, camera_bbox_ct.y);

  //@lijian,投影lidar的polygon
  Eigen::MatrixXd polygon_image(4, 3);
  ObjectPtr trackobj = fused_track_->GetFusedObject()->GetBaseObject();
  if (trackobj->centers_image.empty()) {
    QueryProjectedVeloCtOnCamera(lidar, camera, lidar2camera_pose, &polygon_image);
    Eigen::Vector2f pt_ct = camera_model->Project(Eigen::Vector3f(
        static_cast<float>(polygon_image(0, 0)), static_cast<float>(polygon_image(1, 0)),
        static_cast<float>(polygon_image(2, 0))));
    Eigen::Vector2f pt_lu = camera_model->Project(Eigen::Vector3f(
        static_cast<float>(polygon_image(0, 1)), static_cast<float>(polygon_image(1, 1)),
        static_cast<float>(polygon_image(2, 1))));
    Eigen::Vector2f pt_rd = camera_model->Project(Eigen::Vector3f(
        static_cast<float>(polygon_image(0, 2)), static_cast<float>(polygon_image(1, 2)),
        static_cast<float>(polygon_image(2, 2))));
    if (pt_ct[0] < 0 || pt_ct[0] > 1920 || pt_ct[1] <0 || pt_ct[1] > 1080)
      return distance;
    if (pt_lu[0] < 0 || pt_lu[0] > 1920 || pt_lu[1] <0 || pt_lu[1] > 1080)
      return distance;
    if (pt_rd[0] < 0 || pt_rd[0] > 1920 || pt_rd[1] <0 || pt_rd[1] > 1080)
      return distance;
    trackobj->centers_image.emplace_back(pt_ct);
    trackobj->centers_image.emplace_back(pt_lu);
    trackobj->centers_image.emplace_back(pt_rd);
  }
  Eigen::Vector2f pt_ct_f = trackobj->centers_image[0];
  Eigen::Vector2f pt_lu_f = trackobj->centers_image[1];
  Eigen::Vector2f pt_rd_f = trackobj->centers_image[2];
  // 1.判断lidar点是否在camera里
  if (pt_ct_f[0] > camera_bbox.xmax || pt_ct_f[0] < camera_bbox.xmin ||
      pt_ct_f[1] > camera_bbox.ymax || pt_ct_f[1] < camera_bbox.ymin) {
    distance = distance_thresh_;
  } else {
    // 2.计算center距离
    Eigen::Vector2f ct_diff = pt_ct_f - box2f_ct;
    distance = ct_diff.norm() * vc_diff2distance_scale_factor_;
    if (distance < distance_thresh_) {
      // 3.距离足够近，计算IOU
      float xmin = static_cast<float>(pt_lu_f[0]);
      float xmax = static_cast<float>(pt_rd_f[0]);
      float ymin = static_cast<float>(pt_lu_f[1]);
      float ymax = static_cast<float>(pt_rd_f[1]);
      double w = std::fabs(std::min(xmax, camera_bbox.xmax) - std::max(xmin, camera_bbox.xmin));
      double h = std::fabs(std::min(ymax, camera_bbox.ymax) - std::max(ymin, camera_bbox.ymin));
      double area1 = (xmax - xmin) * (ymax - ymin);
      double area2 = (camera_bbox.xmax - camera_bbox.xmin) * (camera_bbox.ymax - camera_bbox.ymin);
      //如果面积差太大就说明不是一个物体
      float percent = area1/area2;
      if(percent < 0.3 || percent > 3){
          distance = std::numeric_limits<float>::max();
          return distance;
      }
      if (area1 > area2)
        area1 = area2;
      double iou = w * h / area1;
      if (iou < 0.7) {
        distance = std::numeric_limits<float>::max();
      }
    }
  }
  // }
  return distance;
}
// @brief: compute the distance between radar observation and
// camera observation
// @return distance of radar vs. camera
float TrackObjectDistance::ComputeRadarCamera(const SensorObjectConstPtr& radar,
                                              const SensorObjectConstPtr& camera) {
  float distance = distance_thresh_;
  // 1. get camera model and pose
  base::BaseCameraModelPtr camera_model = QueryCameraModel(camera);
  if (camera_model == nullptr) {
    ROS_ERROR_STREAM("ComputeRadarCamera: Failed to get camera model for "
                     << camera->GetSensorId());
    return distance;
  }
  Eigen::Matrix4d world2camera_pose;
  if (!QueryWorld2CameraPose(camera, &world2camera_pose)) {
    return distance;
  }
  // get camera useful information
  const fusion::BBox2DF& camera_bbox = camera->GetBaseObject()->camera_supplement.box;
  const fusion::Point2DF camera_bbox_ct = camera_bbox.Center();
  const Eigen::Vector2d box2d_ct = Eigen::Vector2d(camera_bbox_ct.x, camera_bbox_ct.y);
  Eigen::Vector2d box2d_size =
      Eigen::Vector2d(camera_bbox.xmax - camera_bbox.xmin, camera_bbox.ymax - camera_bbox.ymin);
  box2d_size = box2d_size.cwiseMax(rc_min_box_size_);
  double width = static_cast<double>(camera_model->get_width());
  double height = static_cast<double>(camera_model->get_height());
  // get radar useful information
  double time_diff = camera->GetTimestamp() - radar->GetTimestamp();
  Eigen::Vector3d offset = radar->GetBaseObject()->velocity.cast<double>() * time_diff;
  offset.setZero();
  Eigen::Vector3d radar_ct = radar->GetBaseObject()->center + offset;
  Eigen::Vector4d local_pt = static_cast<Eigen::Matrix<double, 4, 1, 0, 4, 1>>(
      world2camera_pose * Eigen::Vector4d(radar_ct[0], radar_ct[1], radar_ct[2], 1.0));
  std::vector<Eigen::Vector3d> radar_box_vertices;
  GetObjectEightVertices(radar->GetBaseObject(), &radar_box_vertices);
  std::vector<Eigen::Vector2d> radar_box2d_vertices;
  GetModified2DRadarBoxVertices(radar_box_vertices, camera, camera_model, world2camera_pose,
                                &radar_box2d_vertices);
  // compute similarity
  double fused_similarity = 0.0;
  if (local_pt[2] > 0) {
    Eigen::Vector3f pt3f;
    pt3f << camera_model->Project(Eigen::Vector3f(static_cast<float>(local_pt[0]),
                                                  static_cast<float>(local_pt[1]),
                                                  static_cast<float>(local_pt[2]))),
        static_cast<float>(local_pt[2]);
    Eigen::Vector3d pt3d = pt3f.cast<double>();
    if (IsPtInFrustum(pt3d, width, height)) {
      // compute similarity on x direction
      double x_similarity = ComputeRadarCameraXSimilarity(pt3d.x(), box2d_ct.x(), box2d_size.x(),
                                                          rc_x_similarity_params_);
      // compute similarity on y direction
      double y_similarity = ComputeRadarCameraYSimilarity(pt3d.y(), box2d_ct.y(), box2d_size.y(),
                                                          rc_y_similarity_params_);
      // compute similarity on height
      // use camera car height to modify the radar location
      // double h_similarity = ComputeRadarCameraHSimilarity(
      //     radar, camera, box2d_size.y(), radar_box2d_vertices,
      //     rc_h_similarity_params_);
      // compute similarity on width
      // double w_similarity = ComputeRadarCameraWSimilarity(
      //     radar, width, box2d_size.x(), radar_box2d_vertices,
      //     rc_w_similarity_params_);
      // compute similarity on offset 3d
      double loc_similarity = ComputeRadarCameraLocSimilarity(radar_ct, camera, world2camera_pose,
                                                              rc_loc_similarity_params_);
      // compute similarity on velocity
      double velocity_similarity = ComputeRadarCameraVelocitySimilarity(radar, camera);
      // fuse similarity
      std::vector<double> multiple_similarities = {
          x_similarity, y_similarity, loc_similarity, velocity_similarity
          // height_similarity, width_similarity,
      };
      fused_similarity = FuseMultipleProbabilities(multiple_similarities);
    }
  }
  distance = distance_thresh_ * static_cast<float>(1.0 - fused_similarity) /
             (1.0f - rc_similarity2distance_penalize_thresh_);
  ROS_DEBUG_STREAM("ComputeRadarCamera: distance: " << distance);
  return distance;
}

float TrackObjectDistance::ComputeObuCamera(const SensorObjectConstPtr& obu,
                                            const SensorObjectConstPtr& camera) {
  // TODO(@liuxinyu): 相似性复用的RADAR计算方法
  float distance = distance_thresh_;
  // 1. get camera model and pose
  base::BaseCameraModelPtr camera_model = QueryCameraModel(camera);
  if (camera_model == nullptr) {
    ROS_ERROR_STREAM("ComputeObuCamera: Failed to get camera model for " << camera->GetSensorId());
    return distance;
  }
  Eigen::Matrix4d world2camera_pose;
  if (!QueryWorld2CameraPose(camera, &world2camera_pose)) {
    return distance;
  }
  // get camera useful information
  const fusion::BBox2DF& camera_bbox = camera->GetBaseObject()->camera_supplement.box;
  const fusion::Point2DF camera_bbox_ct = camera_bbox.Center();
  const Eigen::Vector2d box2d_ct = Eigen::Vector2d(camera_bbox_ct.x, camera_bbox_ct.y);
  Eigen::Vector2d box2d_size =
      Eigen::Vector2d(camera_bbox.xmax - camera_bbox.xmin, camera_bbox.ymax - camera_bbox.ymin);
  box2d_size = box2d_size.cwiseMax(rc_min_box_size_);
  double width = static_cast<double>(camera_model->get_width());
  double height = static_cast<double>(camera_model->get_height());
  // get obu useful information
  double time_diff = camera->GetTimestamp() - obu->GetTimestamp();
  Eigen::Vector3d offset = obu->GetBaseObject()->velocity.cast<double>() * time_diff;
  offset.setZero();
  Eigen::Vector3d obu_ct = obu->GetBaseObject()->center + offset;
  Eigen::Vector4d local_pt = static_cast<Eigen::Matrix<double, 4, 1, 0, 4, 1>>(
      world2camera_pose * Eigen::Vector4d(obu_ct[0], obu_ct[1], obu_ct[2], 1.0));
  std::vector<Eigen::Vector3d> obu_box_vertices;
  GetObjectEightVertices(obu->GetBaseObject(), &obu_box_vertices);
  std::vector<Eigen::Vector2d> obu_box2d_vertices;
  GetModified2DRadarBoxVertices(obu_box_vertices, camera, camera_model, world2camera_pose,
                                &obu_box2d_vertices);
  // compute similarity
  double fused_similarity = 0.0;
  if (local_pt[2] > 0) {
    Eigen::Vector3f pt3f;
    pt3f << camera_model->Project(Eigen::Vector3f(static_cast<float>(local_pt[0]),
                                                  static_cast<float>(local_pt[1]),
                                                  static_cast<float>(local_pt[2]))),
        static_cast<float>(local_pt[2]);
    Eigen::Vector3d pt3d = pt3f.cast<double>();
    if (IsPtInFrustum(pt3d, width, height)) {
      // compute similarity on x direction
      double x_similarity = ComputeRadarCameraXSimilarity(pt3d.x(), box2d_ct.x(), box2d_size.x(),
                                                          rc_x_similarity_params_);
      // compute similarity on y direction
      double y_similarity = ComputeRadarCameraYSimilarity(pt3d.y(), box2d_ct.y(), box2d_size.y(),
                                                          rc_y_similarity_params_);
      // compute similarity on height
      // use camera car height to modify the obu location
      // double h_similarity = ComputeRadarCameraHSimilarity(
      //     obu, camera, box2d_size.y(), obu_box2d_vertices,
      //     rc_h_similarity_params_);
      // compute similarity on width
      // double w_similarity = ComputeRadarCameraWSimilarity(
      //     obu, width, box2d_size.x(), obu_box2d_vertices,
      //     rc_w_similarity_params_);
      // compute similarity on offset 3d
      double loc_similarity = ComputeRadarCameraLocSimilarity(obu_ct, camera, world2camera_pose,
                                                              rc_loc_similarity_params_);
      // compute similarity on velocity
      double velocity_similarity = ComputeRadarCameraVelocitySimilarity(obu, camera);
      // fuse similarity
      std::vector<double> multiple_similarities = {
          x_similarity, y_similarity, loc_similarity, velocity_similarity
          // height_similarity, width_similarity,
      };
      fused_similarity = FuseMultipleProbabilities(multiple_similarities);
    }
  }
  distance = distance_thresh_ * static_cast<float>(1.0 - fused_similarity) /
             (1.0f - rc_similarity2distance_penalize_thresh_);
  ROS_DEBUG_STREAM("ComputeObuCamera: distance: " << distance);
  return distance;
}

// @brief: compute the distance between camera observation and
// camera observation
// @return the distance of camera vs. camera
float TrackObjectDistance::ComputeCameraCamera(const SensorObjectPtr& fused_camera,
                                               const SensorObjectPtr& sensor_camera) {
  return (std::numeric_limits<float>::max());
}

// @brief: calculate the similarity between velodyne64 observation and
// camera observation
// @return the similarity which belongs to [0, 1]. When velodyne64
// observation is similar to the camera one, the similarity would
// close to 1. Otherwise, it would close to 0.
// @key idea:
// 1. get camera intrinsic and pose
// 2. compute similarity between camera's box and velodyne64's pts
// within camera's frustum
// @NOTE: original method name is compute_velodyne64_camera_dist_score
double TrackObjectDistance::ComputeLidarCameraSimilarity(const SensorObjectConstPtr& lidar,
                                                         const SensorObjectConstPtr& camera,
                                                         const bool measurement_is_lidar) {
  double similarity = 0.0;
  // 1. get camera intrinsic and pose
  base::BaseCameraDistortionModelPtr camera_model = QueryCameraDistortionModel(camera);
  if (camera_model == nullptr) {
    ROS_ERROR_STREAM("ComputeLidarCameraSimilarity: Failed to get camera model for "
                     << camera->GetSensorId());
    return similarity;
  }
  Eigen::Matrix4d world2camera_pose;
  if (!QueryWorld2CameraPose(camera, &world2camera_pose)) {
    return similarity;
  }
  Eigen::Matrix4d lidar2world_pose;
  if (!QueryLidar2WorldPose(lidar, &lidar2world_pose)) {
    return similarity;
  }
  // Eigen::Matrix4d lidar2camera_pose = world2camera_pose * lidar2world_pose;
  // 2. compute similarity of camera vs. velodyne64 observation
  const fusion::PointFCloud& cloud = lidar->GetBaseObject()->lidar_supplement.cloud;
  const fusion::BBox2DF& camera_bbox = camera->GetBaseObject()->camera_supplement.box;
  if (cloud.size() > 0) {
    ProjectionCacheObject* cache_object =
        QueryProjectionCacheObject(lidar, camera, camera_model, measurement_is_lidar);
    if (cache_object == nullptr) {
      return similarity;
    }
    similarity = ComputePtsBoxSimilarity(&projection_cache_, cache_object, camera_bbox);
  }
  return similarity;
}

// @brief: calculate the similarity between radar observation and
// camera observation
// @return the similarity which belongs to [0, 1]. When radar
// observation is similar to the camera one, the similarity would
// close to 1. Otherwise, it would close to 0.
// @NOTE: original method name is compute_radar_camera_dist_score
// @TODO: THIS METHOD SHOULD RETURN 0, IF RADAR IS IN FRONT OF CAMERA DETECTION
double TrackObjectDistance::ComputeRadarCameraSimilarity(const SensorObjectConstPtr& radar,
                                                         const SensorObjectConstPtr& camera) {
  double similarity = 0.0;
  // 1. get camera intrinsic and pose
  base::BaseCameraModelPtr camera_model = QueryCameraModel(camera);
  if (camera_model == nullptr) {
    ROS_ERROR_STREAM("ComputeRadarCameraSimilarity: Failed to get camera model for "
                     << camera->GetSensorId());
    return similarity;
  }
  Eigen::Matrix4d world2camera_pose;
  if (!QueryWorld2CameraPose(camera, &world2camera_pose)) {
    return similarity;
  }
  // 2. get information of camera
  const fusion::BBox2DF& camera_bbox = camera->GetBaseObject()->camera_supplement.box;
  const fusion::Point2DF camera_bbox_ct = camera_bbox.Center();
  const Eigen::Vector2d box2d_ct = Eigen::Vector2d(camera_bbox_ct.x, camera_bbox_ct.y);
  Eigen::Vector2d box2d_size =
      Eigen::Vector2d(camera_bbox.xmax - camera_bbox.xmin, camera_bbox.ymax - camera_bbox.ymin);
  box2d_size = box2d_size.cwiseMax(rc_min_box_size_);
  double width = static_cast<double>(camera_model->get_width());
  double height = static_cast<double>(camera_model->get_height());
  // 3. get information of radar
  Eigen::Vector3d radar_ct = radar->GetBaseObject()->center;
  Eigen::Vector4d local_pt = static_cast<Eigen::Matrix<double, 4, 1, 0, 4, 1>>(
      world2camera_pose * Eigen::Vector4d(radar_ct[0], radar_ct[1], radar_ct[2], 1.0));
  // 4. similarity computation
  Eigen::Vector3d camera_ct = camera->GetBaseObject()->center;
  Eigen::Vector3d camera_ct_c = (world2camera_pose * camera_ct.homogeneous()).head(3);
  double depth_diff = local_pt.z() - camera_ct_c.z();
  const double depth_diff_th = 0.1;
  depth_diff /= camera_ct_c.z();
  if (local_pt[2] > 0 && depth_diff > -depth_diff_th) {
    Eigen::Vector3f pt3f;
    pt3f << camera_model->Project(Eigen::Vector3f(static_cast<float>(local_pt[0]),
                                                  static_cast<float>(local_pt[1]),
                                                  static_cast<float>(local_pt[2]))),
        static_cast<float>(local_pt[2]);
    Eigen::Vector3d pt3d = pt3f.cast<double>();
    if (IsPtInFrustum(pt3d, width, height)) {
      rc_x_similarity_params_2_.welsh_loss_scale_ = rc_x_similarity_params_2_welsh_loss_scale_;
      // compute similarity on x direction
      double x_similarity = ComputeRadarCameraXSimilarity(pt3d.x(), box2d_ct.x(), box2d_size.x(),
                                                          rc_x_similarity_params_2_);
      // compute similarity on y direction
      double y_similarity = ComputeRadarCameraXSimilarity(pt3d.y(), box2d_ct.y(), box2d_size.y(),
                                                          rc_x_similarity_params_2_);
      std::vector<double> multiple_similarities = {x_similarity, y_similarity};
      similarity = FuseMultipleProbabilities(multiple_similarities);
    }
  }
  return similarity;
}

double TrackObjectDistance::ComputePolygonIOU(const SensorObjectConstPtr& fused_object,
                                              const SensorObjectPtr& sensor_object) {
  const fusion::ObjectConstPtr& obj_fused = fused_object->GetBaseObject();
  fusion::ObjectConstPtr object = sensor_object->GetBaseObject();

  double fusion_timestamp = fused_object->GetTimestamp();
  double sensor_timestamp = sensor_object->GetTimestamp();
  double time_diff = sensor_timestamp - fusion_timestamp;

  double trans_x = obj_fused->velocity(0) * time_diff;
  double trans_y = obj_fused->velocity(1) * time_diff;
  IOU iou;
  double iou_area = iou.Compute(obj_fused->track_id, object->id, obj_fused->polygon_utm,
                                object->polygon_utm, trans_x, trans_y);
  return iou_area;
}

// @brief: compute polygon distance between fused object and sensor object
// @return 3d distance between fused object and sensor object
float TrackObjectDistance::ComputePolygonDistance3d(const SensorObjectConstPtr& fused_object,
                                                    const SensorObjectPtr& sensor_object,
                                                    const Eigen::Vector3d& ref_pos,
                                                    int range) {
  const fusion::ObjectConstPtr& obj_f = fused_object->GetBaseObject();
  Eigen::Vector3d fused_poly_center(0, 0, 0);
  if (!QueryPolygonDCenter(obj_f, ref_pos, range, &fused_poly_center)) {
    return (std::numeric_limits<float>::max());
  }
  const fusion::ObjectConstPtr obj_s = sensor_object->GetBaseObject();
  Eigen::Vector3d sensor_poly_center(0, 0, 0);
  if (!QueryPolygonDCenter(obj_s, ref_pos, range, &sensor_poly_center)) {
    return (std::numeric_limits<float>::max());
  }
  double fusion_timestamp = fused_object->GetTimestamp();
  double sensor_timestamp = sensor_object->GetTimestamp();
  double time_diff = sensor_timestamp - fusion_timestamp;
  fused_poly_center(0) += obj_f->velocity(0) * time_diff;
  fused_poly_center(1) += obj_f->velocity(1) * time_diff;
  float distance = ComputeEuclideanDistance(fused_poly_center, sensor_poly_center);
  return distance;
}

// @brief: compute euclidean distance of input pts
// @return eculidean distance of input pts
float TrackObjectDistance::ComputeEuclideanDistance(const Eigen::Vector3d& des,
                                                    const Eigen::Vector3d& src) {
  Eigen::Vector3d diff_pos = des - src;
  float distance =
      static_cast<float>(std::sqrt(diff_pos.head(2).cwiseProduct(diff_pos.head(2)).sum()));
  return distance;
}

// @brief: compute polygon center
// @return true if get center successfully, otherwise return false
bool TrackObjectDistance::ComputePolygonCenter(const fusion::PolygonDType& polygon,
                                               Eigen::Vector3d* center) {
  int size = static_cast<int>(polygon.size());
  if (size == 0) {
    return false;
  }
  *center = Eigen::Vector3d(0, 0, 0);
  for (int i = 0; i < size; ++i) {
    const auto& point = polygon.at(i);
    (*center)[0] += point.x;
    (*center)[1] += point.y;
  }
  (*center) /= size;
  return true;
}

float TrackObjectDistance::ComputeTypeProbality(const ObjectType& fused_type, const ObjectType& sensor_type) {
  float type_density = 0.0;
  if (fused_type == sensor_type)
    return 1.0;
    // 需要继续完善
    return type_density;
  }

// @brief: compute polygon center
// @return true if get center successfully, otherwise return false
bool TrackObjectDistance::ComputePolygonCenter(const fusion::PolygonDType& polygon,
                                               const Eigen::Vector3d& ref_pos,
                                               int range,
                                               Eigen::Vector3d* center) {
  fusion::PolygonDType polygon_part;
  std::map<double, int> distance2idx;
  for (size_t idx = 0; idx < polygon.size(); ++idx) {
    const auto& point = polygon.at(idx);
    double distance = sqrt(pow(point.x - ref_pos(0), 2) + pow(point.y - ref_pos(1), 2));
    distance2idx.insert(std::make_pair(distance, idx));
  }
  int size = static_cast<int>(distance2idx.size());
  int nu = std::max(range, size / range + 1);
  nu = std::min(nu, size);
  int count = 0;
  std::map<double, int>::iterator it = distance2idx.begin();
  for (; it != distance2idx.end(), count < nu; ++it, ++count) {
    polygon_part.push_back(polygon[it->second]);
  }
  bool state = ComputePolygonCenter(polygon_part, center);
  return state;
}

// Modify@jiangnan
float TrackObjectDistance::ComputerPointLinesegmentDistance(const fusion::PointD& PointA,
                                                            const fusion::PointD& PointB,
                                                            const fusion::PointD& PointO) {
  double mProduct =
      (PointB.x - PointA.x) * (PointO.x - PointA.x) + (PointB.y - PointA.y) * (PointO.y - PointA.y);
  if (mProduct <= 0)
    return sqrt((PointO.x - PointA.x) * (PointO.x - PointA.x) +
                (PointO.y - PointA.y) * (PointO.y - PointA.y));
  double nProduct =
      (PointB.x - PointA.x) * (PointB.x - PointA.x) + (PointB.y - PointA.y) * (PointB.y - PointA.y);
  if (mProduct >= nProduct)
    return sqrt((PointB.x - PointO.x) * (PointB.x - PointO.x) +
                (PointB.y - PointO.y) * (PointB.y - PointO.y));
  double r = mProduct / nProduct;
  double ox = PointA.x + (PointB.x - PointA.x) * r;
  double oy = PointA.y + (PointB.y - PointA.y) * r;
  return sqrt((PointO.x - ox) * (PointO.x - ox) + (PointO.y - oy) * (PointO.y - oy));
}

}  // namespace fusion
}  // namespace perception
