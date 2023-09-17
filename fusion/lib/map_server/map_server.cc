#include "map_server.h"

#include <fstream>
#include <string>

#include "perception/lib/config_manager/config_manager.h"

namespace perception {
namespace fusion {
namespace map{

std::map<Point, V_Points> kVegationPolygons;
std::map<Point, V_Points> kBuildingPolygons;
V_V_Points KCurVegPolygons;
V_V_Points KCurBuildingPolygons;

static Json::Value LoadObstacle(std::string file_name) {
  Json::Value obstacle_value(Json::objectValue);
  std::ifstream is;
  std::string obstacle_file_url = file_name;
  is.open(obstacle_file_url, std::ifstream::binary);
  Json::Reader obstacle_reader;
  if (obstacle_reader.parse(is, obstacle_value)) {
  	is.close();
    ROS_INFO("[LoadObstacle] %s parse success.", file_name.c_str());
  	return obstacle_value;
  } else {
  	ROS_WARN("[LoadObstacle] %s parse failed !", file_name.c_str());
  	is.close();
  	return obstacle_value;
  }
}

static V_V_Points ReadPolygons(Json::Value& value) {
  V_V_Points polygons;
  V_Points polygon;
  try {
   for (unsigned int i = 0; i < value.size(); ++i) {
      if (value[i].isNull()) { continue; }
      for (unsigned int j = 0; j < value[i].size(); ++j) {
        polygon.emplace_back(value[i][j]["x"].asDouble(), value[i][j]["y"].asDouble(), .0);
      }
      polygons.push_back(polygon);
      polygon.clear();
    } 
    ROS_INFO("[ReadPolygons] read polygon size:%lu", polygons.size());
  } catch (std::exception e) {
    ROS_ERROR("[ReadPolygons] read polygon error [%s]", e.what());
  }
  
  return polygons;
}

Point computeCenter(const V_Points& polygon) {
  Point center(.0, .0, .0);
  int size = 0;
  for(const auto& p : polygon) {
    if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) {
      continue;
    }
    size++;
    center += p;
  }
  center /= size;
  return center;
}

static void LoadValue(std::string map_name) {
  kVegationPolygons.clear();
  kBuildingPolygons.clear();

  std::string map_file = lib::ConfigManager::Instance()->work_root() + "map/" + map_name;//这个地址改为从rosparam读取

  // load map.obstacle
  Json::Value value = LoadObstacle(map_file);//读取map.obstacle参数文件,保存在value中
  if (value.isNull()) { return; }
  // 1.读取植被
  auto&& polygons = ReadPolygons(value["vegatations"]["polygons"]);
  for (auto polygon : polygons) {
    if (polygon.size() < 3) {
      continue;
    }
    Point center = computeCenter(polygon);
    kVegationPolygons.insert(std::make_pair(center, polygon));//如果出现center重复怎么办？不鲁棒
  }
  // 2.读取建筑物(后续可以参数化列表读取想要读取的类型)
  auto&& building_polygons = ReadPolygons(value["building"]["polygons"]);
  for (auto polygon : building_polygons) {
    if (polygon.size() < 3) {
      continue;
    }
    Point center = computeCenter(polygon);
    kBuildingPolygons.insert(std::make_pair(center, polygon));
  }
}

static double CalDis(Point polygon_center, const localization::Localization& localization) {
  double dis = std::sqrt(std::pow(polygon_center.x - localization.position().x(), 2) + std::pow(polygon_center.y - localization.position().y(), 2));
  return dis;
}

void Init(std::string map_name) {
  LoadValue(map_name);
}

V_V_Points GetCurrentVegPolygons() {
  return KCurVegPolygons;
}

V_V_Points GetCurrentBuildingPolygons() {
  return KCurBuildingPolygons;
}

void UploadPolygons(const localization::Localization& localization) {
  // 加载自车20m以内的polygons
  KCurVegPolygons.clear();
  for (const auto& polygon : kVegationPolygons) {
    if (CalDis(polygon.first, localization) <= 30.0) {
      KCurVegPolygons.push_back(polygon.second);
    }
  }

  KCurBuildingPolygons.clear();
  for (const auto& polygon : kBuildingPolygons) {
    if (CalDis(polygon.first, localization) <= 30.0) {
      KCurBuildingPolygons.push_back(polygon.second);
    }
  }
}

void Destroy() {
  kVegationPolygons.clear();
  KCurVegPolygons.clear();
  kBuildingPolygons.clear();
  KCurBuildingPolygons.clear();

}

} // namespace map
} // namespace fusion
} // namespace perception
