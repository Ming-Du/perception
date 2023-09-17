#include "mg_roigrid.h"
#include <ros/ros.h>
#include <fstream>
#include <queue>
#include <stack>
#include <vector>
namespace perception {
namespace radar {

bool MgRoigrid::isInRoadMap(double local_x, double local_y, int& road_type) {
  road_type = tileMapFiltering(local_x, local_y);
  return (1 == road_type);
}

void MgRoigrid::setBinaryInRoad(cv::Mat& imgGray) {
  int road_value = 1;
  int value2bin[255] = {0};
  value2bin[road_value] = 1;
  int rows = imgGray.rows;
  int cols = imgGray.cols;
  for (int row = 0; row < rows; row++) {
    uchar* p2 = imgGray.ptr<uchar>(row);
    for (int col = 0; col < cols; col++) {
      uchar& pix2 = *p2++;
      pix2 = value2bin[pix2];
    }
  }
}

int MgRoigrid::tileMapFiltering(double distance_x, double distance_y) {
  const auto& current_lon = local_cur_ptr->longitude();
  const auto& current_lat = local_cur_ptr->latitude();
  double utm_x = local_cur_ptr->position().x();
  double utm_y = local_cur_ptr->position().y();
  double utm_z = local_cur_ptr->position().z();
  double heading = M_PI_2 - local_cur_ptr->yaw();
  // distance_x和distance_y是要过滤的位置
  std::string UTM_ZONE = std::to_string(local_cur_ptr->utm_zone());
  convertLocalToGlobal(utm_x, utm_y, utm_z, heading, distance_x, distance_y, UTM_ZONE);
  int64_t tile_id = getTileID(distance_x, distance_y, level);
  std::string png_path = MgRoigrid::grid_map_data_path + "/" + std::to_string(tile_id) + ".png";
  ROS_DEBUG_STREAM(png_path);
  if (find(invalid_paths.begin(), invalid_paths.end(), png_path) != invalid_paths.end()) {
    return 1;
  }
  if (find(curfrm_used_paths.begin(), curfrm_used_paths.end(), png_path) ==
      curfrm_used_paths.end()) {
    curfrm_used_paths.push_back(png_path);
    ROS_DEBUG_STREAM(HDEBUG_B << "UTM x,y:" << distance_x << ", " << distance_y);
  }
  if (tile_maps.find(tile_id) == tile_maps.end()) {
    // new id
    if (access(png_path.c_str(), F_OK) == 0) {
      cv::Mat img = cv::imread(png_path, CV_LOAD_IMAGE_GRAYSCALE);
      setBinaryInRoad(img);
      cv::Mat kernel;
      cv::Mat usedImg;
      int kernel_size = abs(roadmap_shrink_) * 2 + 1;
      kernel = cv::getStructuringElement(CV_SHAPE_ELLIPSE, cv::Size(kernel_size, kernel_size));
      if (roadmap_shrink_ > 0) {
        cv::erode(img, usedImg, kernel);
        tile_maps[tile_id] = usedImg;  // img;
      } else if (roadmap_shrink_ < 0) {
        cv::dilate(img, usedImg, kernel);
        tile_maps[tile_id] = usedImg;  // img;
      } else {
        // usedImg = img.clone();
        tile_maps[tile_id] = img;  // img;
      }
    } else {
      ROS_WARN_STREAM(png_path << " file not found!");
      invalid_paths.push_back(png_path);
      return 1;
    }
  }
  tile_ids_lostnum[tile_id] = 10;
  int ele = getRoadElement(distance_x, distance_y, level, tile_id);
  return ele;
}

void MgRoigrid::delOldMap() {
  std::vector<int64_t> todel;
  for (auto& [id, lostnum] : tile_ids_lostnum) {
    lostnum--;
    if (lostnum <= 0) {
      todel.push_back(id);
    }
  }
  for (auto ti : todel) {
    tile_maps.erase(ti);
    tile_ids_lostnum.erase(ti);
  }
  ROS_DEBUG_STREAM(HDEBUG_R << "tile_ids_lostnum.size:" << tile_ids_lostnum.size());
  ROS_DEBUG_STREAM(HDEBUG_R << "tile_maps.size:" << tile_maps.size());
}
void MgRoigrid::convertLocalToGlobal(double utm_x,
                                     double utm_y,
                                     double utm_z,
                                     double heading,
                                     double& distance_x,
                                     double& distance_y,
                                     std::string UTM_ZONE) {
  double dist = std::sqrt(distance_x * distance_x + distance_y * distance_y);
  double p_angle = heading - std::atan2(distance_y, distance_x);
  utm_x += dist * std::sin(p_angle);
  utm_y += dist * std::cos(p_angle);

  static bool is_init = false;
  if (!is_init) {
    std::string str =
        "+proj=utm +zone=" + UTM_ZONE + " +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs";
    utm_source_ = pj_init_plus(str.c_str());
    wgs84pj_target_ = pj_init_plus("+proj=latlong +ellps=WGS84");
    is_init = true;
  }
  pj_transform(utm_source_, wgs84pj_target_, 1, 1, &utm_x, &utm_y, &utm_z);
  utm_x *= RAD_TO_DEG_LOCAL;
  utm_y *= RAD_TO_DEG_LOCAL;

  distance_x = utm_x;
  distance_y = utm_y;
}
int64_t MgRoigrid::getTileID(const double& lon, const double& lat, const int64_t& level) {
  int64_t tile_id = 0;
  // toMortonCode
  int64_t bit = 1;
  int64_t mortonCode = 0;
  int64_t x = (int64_t)(lon * (UINT32_MAX_DEFINE / 360.0));
  int64_t y = (int64_t)(lat * (UINT32_MAX_DEFINE / 360.0));
  if (y < 0) {
    y += 0x7FFFFFFF;
  }
  y <<= 1;
  for (int i = 0; i < 32; i++) {
    // x-part
    mortonCode |= x & bit;
    x <<= 1;
    bit <<= 1;
    // y-part
    mortonCode |= y & bit;
    y <<= 1;
    bit <<= 1;
  }

  // get tile id
  tile_id = (((mortonCode >> (2 * (31 - level))) & 0xffffffff) + ((int64_t)1 << (16 + level)));
  return tile_id;
}
int MgRoigrid::getRoadElement(double& current_lon,
                              double& current_lat,
                              const int64_t& level,
                              const int64_t tile_id) {
  int64_t tile_idx = tile_id - ((int64_t)1 << (16 + level));
  int64_t mortonCode = ((int64_t)tile_idx) << (2 * (31 - level));

  int64_t coord_x = 0, coord_y = 0;

  // mortonCodeToCoord
  {
    int64_t bit = 1;

    for (int i = 0; i < 32; i++) {
      coord_x |= (mortonCode & bit);
      mortonCode >>= 1;
      coord_y |= (mortonCode & bit);
      bit <<= 1;
    }
  }

  // normalizeCoord
  {
    // if x > 180 degrees, then subtract 360 degrees
    if (coord_x > NDS_180_DEGREES) {
      coord_x -= NDS_360_DEGREES + 1;       // add 1 because 0 must be counted as well
    } else if (coord_x < -NDS_180_DEGREES)  // if x < 180 , x += 360
    {
      coord_x += NDS_360_DEGREES + 1;  // add 1 because 0 must be counted as well
    }

    // if y > 90 degrees, then subtract 180 degrees
    if (coord_y > NDS_90_DEGREES) {
      coord_y -= NDS_180_DEGREES + 1;      // add 1 because 0 must be counted as well
    } else if (coord_y < -NDS_90_DEGREES)  // if y < 90, y += 180
    {
      coord_y += NDS_180_DEGREES + 1;  // add 1 because 0 must be counted as well
    }
  }

  coord_x += 1;
  coord_y += 1;

  //左下经纬度，lat为纵轴，lon横轴
  double left_bottom_lon = (double)((coord_x * 360.0) / UINT32_MAX_DEFINE);
  double left_bottom_lat = (double)((coord_y * 360.0) / UINT32_MAX_DEFINE);
  //右上角经纬度
  int64_t c = (int64_t(1) << 31 - level) - 1;
  double right_top_lon = (double)(((coord_x + c) * 360.0) / UINT32_MAX_DEFINE);
  double right_top_lat = (double)(((coord_y + c) * 360.0) / UINT32_MAX_DEFINE);
  int pos_lon = (current_lon - left_bottom_lon) * 1250 / tile_length;
  int pos_lat = (current_lat - left_bottom_lat) * 1250 / tile_length;
  //元素在瓦片中的相对坐标
  // return (tile_maps[tile_id].at<int>(pos_lon, pos_lat));
  int ret_val = tile_maps[tile_id].at<unsigned char>(pos_lat, pos_lon);
  return ret_val;
}
}  // namespace radar
}  // namespace perception