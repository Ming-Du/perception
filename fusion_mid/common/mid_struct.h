#include <opencv2/opencv.hpp>
#include "common/sensor_data/cloud_data.h"
#include "perception/base/sensor_manager/sensor_manager.h"
#include "common/proto/object.pb.h"
#include "perception/fusion_mid/common/vec2.h"

#ifndef STRUCTMID_H
#define STRUCTMID_H
namespace perception {
namespace mid_fusion {
enum camera_postion {
  FRONT_FOV_60 = 1,
  FRONT_FOV_30,
  FRONT_FOV_120,
};

struct CameraIDRect {
  cv::Rect camera_rect_;
  int id;
  CameraIDRect(int a, int b, int c, int d) {
    cv::Rect temprect(a, b, c, d);
    camera_rect_ = temprect;
    id = -1;
  }
};
struct FrustumLidarPoint {  // single lidar point in space
  unsigned int id;
  camera_postion cp;  // front FOV_30, FOV_60. FOV_120, left FOV_120. right
                      // FOV_120, rear FOV_120
  double point3D_x, point3D_y, point3D_z,
      point3D_r;  // x,y,z in [m], r is point reflectivity
  bool is_on_img = false;
  double point2D_x, point2D_y;  // point 2D on image
  int camera_object_id = -1;
  float frustum_angle = 0.0;
  int only_camera_count = 0;
};

struct Fusion_RawData {
  mid_fusion::CloudData current_cloud_data_;
  perception::RadarObjects current_radar_data_;
  perception::TrackedObjects current_lidarobjects_data_;
  void clearData() {
    current_cloud_data_.cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
    current_radar_data_.Clear();
    current_lidarobjects_data_.Clear();
  }
};

struct PreprocessFpntInput {
  std::vector<float> pcd_xyzi;
  std::vector<float> box_det2d;
  std::vector<uint32_t> box_ID;
  std::unordered_map<int, std::string> index_to_camtopic_map;
  int box_count;
  void ClearData() {
    pcd_xyzi.clear();
    box_det2d.clear();
    box_ID.clear();
    index_to_camtopic_map.clear();
    box_count = 0;
  }
  bool empty() {
    if (box_count == 0) {
      return true;
    }
    return false;
  }
};


struct BoundaryPoint{
	double x;
	double y;
	int type;  //type未定义
};
struct LaneMarkerData {
	std::vector<perception::mid_fusion::Point2d> points;
	float confidence;
	int type;
	int color;
	std::vector<BoundaryPoint> points_with_type;
};

struct LaneMarkerDataRecord {  // left、left2、left_right分别代表什么
  LaneMarkerData left;
  LaneMarkerData left_right;
  LaneMarkerData left2;
  LaneMarkerData right;
  LaneMarkerData right_left;
  LaneMarkerData right2;
  double time_stamp;
  void clear() {
    time_stamp = 0.0;
    left.points.clear();
    left_right.points.clear();
    left2.points.clear();
    right.points.clear();
    right_left.points.clear();
    right2.points.clear();
  }
  bool empty() {
    return left.points.empty() && left_right.points.empty() && left2.points.empty() &&
           right.points.empty() && right_left.points.empty() && right2.points.empty();
  }
};
typedef std::list<LaneMarkerDataRecord> LaneMarkersDataRecords;

}  // namespace mid_fusion
}  // namespace perception
#endif