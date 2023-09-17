#include "utils.h"

#include <unordered_map>

#include <Eigen/Core>

namespace perception {
namespace mid_fusion {

constexpr double PI = 3.141592653589793238462;
constexpr double kEpsilon = 1e-6;

void FindBboxPoint(const perception::Object& obj,
                   std::vector<geometry_msgs::Point>& box_points,
                   const bool is_box_corner) {
  if (is_box_corner) {
    Eigen::MatrixXf corner_points(4, 3);
    corner_points << -obj.size().x() / 2, -obj.size().y() / 2, 1, -obj.size().x() / 2,
        obj.size().y() / 2, 1, obj.size().x() / 2, obj.size().y() / 2, 1, obj.size().x() / 2,
        obj.size().y() / 2, 1;
    Eigen::Matrix3f rotation_and_translation;
    double yaw = obj.angle();
    if (yaw >= 2 * PI)
      yaw -= 2 * PI;
    if (yaw < 0)
      yaw += 2 * PI;
    float angle = yaw;  //变换角度,0-2pi

    rotation_and_translation << cos(angle), -sin(angle), obj.center().x(), sin(angle), cos(angle),
        obj.center().y(), 0, 0, 1;
    Eigen::MatrixXf pose_result(4, 3);
    pose_result.setIdentity();
    pose_result = (rotation_and_translation * corner_points.transpose()).transpose();

    geometry_msgs::Point lfd;  // lbd
    lfd.x = pose_result(0, 0);
    lfd.y = pose_result(0, 1);
    lfd.z = obj.center().z() - obj.size().z() / 2.;

    geometry_msgs::Point lbd;  // rbd
    lbd.x = pose_result(1, 0);
    lbd.y = pose_result(1, 1);
    lbd.z = obj.center().z() - obj.size().z() / 2.;

    geometry_msgs::Point rfd;  // lfd
    rfd.x = pose_result(2, 0);
    rfd.y = pose_result(2, 1);
    rfd.z = obj.center().z() - obj.size().z() / 2.;

    geometry_msgs::Point rbd;  // rfd
    rbd.x = pose_result(3, 0);
    rbd.y = pose_result(3, 1);
    rbd.z = obj.center().z() - obj.size().z() / 2.;

    geometry_msgs::Point lfu;  // lbu
    lfu.x = pose_result(0, 0);
    lfu.y = pose_result(0, 1);
    lfu.z = obj.center().z() + obj.size().z() / 2;

    geometry_msgs::Point lbu;  // rbu
    lbu.x = pose_result(1, 0);
    lbu.y = pose_result(1, 1);
    lbu.z = obj.center().z() + obj.size().z() / 2;

    geometry_msgs::Point rfu;  // lfu
    rfu.x = pose_result(2, 0);
    rfu.y = pose_result(2, 1);
    rfu.z = obj.center().z() + obj.size().z() / 2;

    geometry_msgs::Point rbu;  // rfu
    rbu.x = pose_result(3, 0);
    rbu.y = pose_result(3, 1);
    rbu.z = obj.center().z() + obj.size().z() / 2;

    box_points.push_back(lfd);
    box_points.push_back(lbd);
    box_points.push_back(rbd);
    box_points.push_back(rfd);
    box_points.push_back(lfu);
    box_points.push_back(lbu);
    box_points.push_back(rbu);
    box_points.push_back(rfu);
  } else {
    for (size_t i = 0; i < obj.contour_size(); i++) {
      geometry_msgs::Point temp_point;  // lbd
      temp_point.x = obj.contour(i).x();
      temp_point.y = obj.contour(i).y();
      temp_point.z = obj.contour(i).z();
      box_points.push_back(temp_point);
      geometry_msgs::Point temp_point2;  // lbd
      temp_point2.x = obj.contour(i).x();
      temp_point2.y = obj.contour(i).y();
      temp_point2.z = obj.contour(i).z() + obj.size().z();
      box_points.push_back(temp_point2);
    }
  }
}

/* @brief: 计算两个障碍物的最近距离
 * @param [in]:object1-障碍物1,object2-障碍物2
 * @param [out]:最近距离
 * @return:最近距离
 */
float ComputeNearDistance(std::vector<float>& corners1, perception::RadarObject* object2) {
  //先计算目标的四个角点
  // TODO(@liuxinyu): 毫米波点与激光的四个点分别计算，取最小值为最小距离
  int num1 = corners1.size() / 2;
  float x1, y1, x2, y2, dis;

  x2 = object2->obj().center().x();
  y2 = object2->obj().center().y();

  float dis_min = 40;
  for (int i = 0; i < num1; i++) {
    x1 = corners1[2 * i + 0];
    y1 = corners1[2 * i + 1];
    dis = std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    dis_min = std::min(dis_min, dis);
  }
  return dis_min;
}

/* @brief: 计算障碍物的四个角点
 * @param [in]:object-障碍物
 * @param [out]:四个角点Ｘ和Ｙ坐标值
 * @return:角点坐标向量
 */
std::vector<float> ExtractCorners(const perception::Object& object) {
  std::vector<float> corners;
  for (int i = 0; i < object.contour_size(); i++) {
    corners.push_back(object.contour(i).x());
    corners.push_back(object.contour(i).y());
  }
  return corners;
}

/* @brief: 设置目标关联距离门限
 * @param [in]: 目标距离
 * @param [out]: 关联距离门限
 * @return:关联距离门限
 */
float SetAssoDisThr(float range, double x, double y) {
  if (fabs(x) < 8 && fabs(y) > 1.5) {
    return 1.5;
  }
  float max_range = 60;
  float min_range = 1;
  float start_dis_near = 1;
  float thr = 0;
  if (range <= 15) {
    thr = start_dis_near;
  } else {
    thr = start_dis_near + 2 * (range - min_range) / (max_range - min_range);
  }
  return thr;
}

float ComputeIou2D(const perception::Object& object1,
                   const perception::Object& object2,
                   const perception::mid_common::DevRotateIouPtr& dev_rotate_iou_ptr) {
  float rbox1[5];
  float rbox2[5];
  rbox1[0] = object1.center().y();
  rbox1[1] = object1.center().x();
  rbox1[2] = object1.size().y();
  rbox1[3] = object1.size().x();
  rbox1[4] = 0.5 * M_PI - object1.angle();

  rbox2[0] = object2.center().y();
  rbox2[1] = object2.center().x();
  rbox2[2] = object2.size().y();
  rbox2[3] = object2.size().x();
  rbox2[4] = 0.5 * M_PI - object2.angle();

  float iou = dev_rotate_iou_ptr->devRotateIoUEval(rbox1, rbox2);
  //    ROS_INFO_STREAM("ComputeIou2D begin input::");
  //    ROS_INFO_STREAM("rbox1::" << rbox1[0] << "," << rbox1[1] << "," <<
  //    rbox1[2] << ","<<rbox1[3] <<", " << rbox1[4] );
  //    ROS_INFO_STREAM("rbox2::" << rbox2[0] << "," << rbox2[1] << "," <<
  //    rbox2[2] << ","<<rbox2[3] <<", " << rbox2[4]);
  //    ROS_INFO_STREAM("ComputeIou2D::" << iou);
  return iou;
}

bool IsPolygonboxIntersection(const perception::Object& object1,
                              const perception::Object& object2) {
  for (size_t n = 0; n < object2.contour_size(); n++) {
    auto input_x = object2.contour(n).x();
    auto input_y = object2.contour(n).y();
    int left_cross = 0, right_cross = 0;
    for (int i = 0; i < object1.contour_size(); ++i) {
      auto& p1 = object1.contour(i);                                 //当前节点
      auto& p2 = object1.contour((i + 1) % object1.contour_size());  //下一个节点
      if (p1.y() == p2.y() && std::fabs(input_y - p1.y()) < 0.0001 &&
          input_x < std::max(p1.x(), p2.x()) && input_x > std::min(p1.x(), p2.x()))
        return true;
      if (p1.y() == p2.y())  // p1p2 与 y平行，无交点
        continue;
      if (input_y <= std::min(p1.y(), p2.y()))  //线段在上方，无交点
        continue;
      if (input_y > std::max(p1.y(), p2.y()))
        continue;
      // 从P发射一条水平射线 求交点的 X 坐标 ------原理:
      // ((p2.y-p1.y)/(p2.x-p1.x))=((y-p1.y)/(x-p1.x))
      //直线k值相等 交点y=p.y
      double x = (input_y - p1.y()) * (p2.x() - p1.x()) / (p2.y() - p1.y()) + p1.x();

      if (std::fabs(x - input_x) < 0.005) {
        return true;
      } else if (x > input_x)
        right_cross++;
      else
        left_cross++;
    }
    // 两边都是单数
    if (right_cross % 2 == 1 && left_cross % 2 == 1) {
      return true;
    }
  }

  //遍历ai的每个顶点
  for (int i = 0; i < object1.contour_size(); ++i) {
    auto input_x = object1.contour(i).x();
    auto input_y = object1.contour(i).y();
    int left_cross = 0, right_cross = 0;
    for (size_t n = 0; n < object2.contour_size(); n++) {
      auto& p1 = object2.contour(n);                                 //当前节点
      auto& p2 = object2.contour((n + 1) % object2.contour_size());  //下一个节点

      if (p1.y() == p2.y() && std::fabs(input_y - p1.y()) < 0.0001 &&
          input_x < std::max(p1.x(), p2.x()) && input_x > std::min(p1.x(), p2.x()))
        return true;

      if (p1.y() == p2.y())  // p1p2 与 y平行，无交点
        continue;

      if (input_y <= std::min(p1.y(), p2.y()))  //线段在上方，无交点
        continue;

      if (input_y > std::max(p1.y(), p2.y()))
        continue;

      // 从P发射一条水平射线 求交点的 X 坐标 ------原理:
      // ((p2.y-p1.y)/(p2.x-p1.x))=((y-p1.y)/(x-p1.x))
      //直线k值相等 交点y=p.y
      double x = (input_y - p1.y()) * (p2.x() - p1.x()) / (p2.y() - p1.y()) + p1.x();

      if (std::fabs(x - input_x) < 0.005) {
        return true;
      } else if (x > input_x)
        right_cross++;
      else
        left_cross++;
    }
    // 两边都是单数
    if (right_cross % 2 == 1 && left_cross % 2 == 1) {
      return true;
    }
  }
  return false;
}

bool IsLineIntersection(const perception::Object& A, const perception::Object& B) {
  // A: CAM30 B: CAM60
  for (int i = 0; i < A.contour_size(); i++) {
    auto& a_p1 = A.contour(i);                           //当前节点
    auto& a_p2 = A.contour((i + 1) % A.contour_size());  //下一个节点
    ROS_DEBUG_STREAM("A: " << "id: " << A.id() << "a_p1(" << a_p1.x() << "," << a_p1.y() << ")" << " a_p2(" << a_p2.x() << "," << a_p2.y() << ")");
    for (int j = 0; j < B.contour_size(); j++) {
      auto& b_p1 = B.contour(j);                           //当前节点
      auto& b_p2 = B.contour((j + 1) % B.contour_size());  //下一个节点
      ROS_DEBUG_STREAM("B: " << "id: " << B.id() << "b_p1(" << b_p1.x() << "," << b_p1.y() << ")" << " b_p2(" << b_p2.x() << "," << b_p2.y() << ")");
      //快速排斥实验
      if ((a_p1.x() > a_p2.x() ? a_p1.x() : a_p2.x()) <
              (b_p1.x() < b_p2.x() ? b_p1.x() : b_p2.x()) ||
          (a_p1.y() > a_p2.y() ? a_p1.y() : a_p2.y()) <
              (b_p1.y() < b_p2.y() ? b_p1.y() : b_p2.y()) ||
          (b_p1.x() > b_p2.x() ? b_p1.x() : b_p2.x()) <
              (a_p1.x() < a_p2.x() ? a_p1.x() : a_p2.x()) ||
          (b_p1.y() > b_p2.y() ? b_p1.y() : b_p2.y()) <
              (a_p1.y() < a_p2.y() ? a_p1.y() : a_p2.y())) {
        continue;
      }
      //跨立实验
      if ((((a_p1.x() - b_p1.x()) * (b_p2.y() - b_p1.y()) -
            (a_p1.y() - b_p1.y()) * (b_p2.x() - b_p1.x())) *
           ((a_p2.x() - b_p1.x()) * (b_p2.y() - b_p1.y()) -
            (a_p2.y() - b_p1.y()) * (b_p2.x() - b_p1.x()))) > 0 ||
          (((b_p1.x() - a_p1.x()) * (a_p2.y() - a_p1.y()) -
            (b_p1.y() - a_p1.y()) * (a_p2.x() - a_p1.x())) *
           ((b_p2.x() - a_p1.x()) * (a_p2.y() - a_p1.y()) -
            (b_p2.y() - a_p1.y()) * (a_p2.x() - a_p1.x()))) > 0) {
        continue;
      }
      return true;
    }
  }
  return false;
}

  bool IsIntersection(const perception::Object& A, const perception::Object& B) {
    geos::geom::GeometryFactory::Ptr factory = geos::geom::GeometryFactory::create();
    std::vector<Coordinate> A_cas;
    for (int i = 0; i < A.contour_size(); i++) {
      ROS_DEBUG_STREAM(std::setprecision(8) << A.contour(i).x() << " " << A.contour(i).y());
      auto x = A.contour(i).x();
      auto y = A.contour(i).y();
      A_cas.push_back(Coordinate(x, y));
    }
    A_cas.push_back(Coordinate(A.contour(0).x(), A.contour(0).y()));
    std::unique_ptr<LinearRing> A_lr = factory->createLinearRing(std::move(A_cas));
    std::vector<Coordinate> B_cas;
    for (int i = 0; i < B.contour_size(); i++) {
      ROS_DEBUG_STREAM(std::setprecision(8) << B.contour(i).x() << " " << B.contour(i).y());
      auto x = B.contour(i).x();
      auto y = B.contour(i).y();
      B_cas.push_back(Coordinate(x, y));
    }
    B_cas.push_back(Coordinate(B.contour(0).x(), B.contour(0).y()));
    std::unique_ptr<LinearRing> B_lr = factory->createLinearRing(std::move(B_cas));

    std::unique_ptr<Polygon> A_poly = factory->createPolygon(std::move(A_lr));
    std::unique_ptr<Polygon> B_poly = factory->createPolygon(std::move(B_lr));
    std::unique_ptr<Geometry> inter = A_poly->intersection(B_poly.get());
    double min_area = std::min(A_poly->getArea(), B_poly->getArea());
    double area = inter->getArea() / min_area;
    if (area > 0.1) {
      return true;
    }
    return false;
  }

  int MarginProcess(const perception::VisualObjects& camera_2dboxs, std::vector<int>& margin_vec,
                    const int margin) {
    int box_size = 0;
    int width = camera_2dboxs.width();
    int height = camera_2dboxs.height();
    cv::Rect camera_rect(0, 0, width, height);
    int length_size = camera_2dboxs.objs_size();
    for (int i = 0; i < length_size; ++i) {
      cv::Rect box2d_rect(camera_2dboxs.objs(i).x() - margin, camera_2dboxs.objs(i).y() - margin,
                          camera_2dboxs.objs(i).width() + margin * 2,
                          camera_2dboxs.objs(i).height() + margin * 2);
      cv::Rect and_box = camera_rect | box2d_rect;
      if (camera_rect.area() == and_box.area()) {
        margin_vec[box_size] = i;
        box_size++;
      }
    }
    return box_size;
  }

  void FpnetobjectMatch(
      const ::google::protobuf::RepeatedPtrField<::perception::TrackedObject>& objs_input,
      ::google::protobuf::RepeatedPtrField<::perception::TrackedObject>* objs_output) {
    std::vector<perception::TrackedObject> vec_30s;
    std::vector<perception::TrackedObject> vec_60s;
    std::vector<perception::TrackedObject> vec_right120;
    for (int i = 0; i < objs_input.size(); ++i) {
      if (objs_input.Get(i).obj().sensor_name() == "/perception/camera/camera_obstacle_front30") {
        vec_30s.push_back(objs_input.Get(i));
      } else if (objs_input.Get(i).obj().sensor_name() ==
                 "/perception/camera/camera_obstacle_front60") {
        vec_60s.push_back(objs_input.Get(i));
      } else if (objs_input.Get(i).obj().sensor_name() ==
                 "/perception/camera/camera_obstacle_right120") {
        vec_right120.push_back(objs_input.Get(i));
      }
    }

    // is line intersection?
    std::unordered_map<int, int> index_30to60map;
    for (int j = 0; j < vec_60s.size(); j++) {
      for (int i = 0; i < vec_30s.size(); i++) {
        bool isintersection = IsLineIntersection(vec_30s[i].obj(), vec_60s[j].obj());
        if (isintersection == true) {
          index_30to60map[i] = j;
          break;
        }
      }
    }
    // reserve 60 detection
    for (int i = 0; i < vec_60s.size(); i++) {
      perception::TrackedObject* result_object = objs_output->Add();
      ROS_DEBUG_STREAM("c60: id: " << vec_60s[i].obj().id());
      result_object->CopyFrom(vec_60s[i]);
    }
    // reserve right 120 detection
    for (int i = 0; i < vec_right120.size(); i++) {
      perception::TrackedObject* result_object = objs_output->Add();
      result_object->CopyFrom(vec_right120[i]);
    }
    // reserve 30 detection
    for (int i = 0; i < vec_30s.size(); i++) {
      if (index_30to60map.count(i) == 0) {
        perception::TrackedObject* result_object = objs_output->Add();
        ROS_DEBUG_STREAM("c30: id: " << vec_30s[i].obj().id());
        result_object->CopyFrom(vec_30s[i]);
      }
    }
  }

  void ComputeInitialVelocity(const perception::TrackedObjects& last_frame_objects,
                              perception::TrackedObjects& current_frame_objects,
                              std::map<uint32_t, double> fpnet_pred_obj_count,
                              const localization::Localization& local_current) {
    const double time_diff = (current_frame_objects.header().stamp().sec() +
                              current_frame_objects.header().stamp().nsec() * 1e-9) -
                             (last_frame_objects.header().stamp().sec() +
                              last_frame_objects.header().stamp().nsec() * 1e-9);
    if (time_diff < kEpsilon) {
      ROS_WARN_STREAM("ComputeInitialVelocity: Frame time diff is " << time_diff);
      return;
    }
    double host_yaw_global = local_current.yaw();
    double cos_host = std::cos(host_yaw_global);
    double sin_host = std::sin(host_yaw_global);
    double host_velocity = std::sqrt(std::pow(local_current.longitudinal_v(), 2) +
                                     std::pow(local_current.lateral_v(), 2));
    for (int i = 0; i < current_frame_objects.objs_size(); i++) {
      perception::TrackedObject* current_object = current_frame_objects.mutable_objs(i);
      uint32_t current_object_id = current_object->obj().id();
      for (int j = 0; j < last_frame_objects.objs_size(); j++) {
        const perception::TrackedObject& last_object = last_frame_objects.objs(j);
        uint32_t last_object_id = last_object.obj().id();
        if (last_object_id != current_object_id ||
            current_object->obj().type() == perception::ObjectType::TYPE_TRIANGLEROADBLOCK) {
          continue;
        }
        // extra tracking time corresponding id
        double tracking_time = 1;
        std::map<uint32_t, double>::iterator iter;
        iter = fpnet_pred_obj_count.find(current_object_id);
        if (iter != fpnet_pred_obj_count.end()) {
          tracking_time = iter->second;
        }
        current_object->mutable_obj()->set_tracking_time(tracking_time);
        // set veh - rel vel
        double rel_vx, rel_vy, rel_vz;
        rel_vx = (current_object->obj().center().x() - last_object.obj().center().x()) / time_diff;
        rel_vy = (current_object->obj().center().y() - last_object.obj().center().y()) / time_diff;
        rel_vz = (current_object->obj().center().z() - last_object.obj().center().z()) / time_diff;
        double utm_vx = (rel_vx + host_velocity) * cos_host - rel_vy * sin_host;
        double utm_vy = (rel_vx + host_velocity) * sin_host + rel_vy * cos_host;
        double abs_v = std::sqrt(std::pow(utm_vx, 2) + std::pow(utm_vy, 2));
        double alpha = std::atan2(utm_vx,utm_vy);
        double cos_alpha = std::cos(alpha);
        double sin_alpha = std::sin(alpha);
        // Limiting Speed
        if (abs_v * 3.6 > kType2SpdThr.at(current_object->obj().type())) {
          double max_spd = kType2SpdThr.at(current_object->obj().type()) / 3.6;
          double refine_utm_vx = max_spd * sin_alpha;
          double refine_utm_vy = max_spd * cos_alpha;
          rel_vx = refine_utm_vx * cos_host + refine_utm_vy * sin_host - local_current.longitudinal_v();
          rel_vy = -refine_utm_vx * sin_host + refine_utm_vy * cos_host - local_current.lateral_v();
        }
        current_object->mutable_obj()->mutable_velocity()->set_x(rel_vx);
        current_object->mutable_obj()->mutable_velocity()->set_y(rel_vy);
        current_object->mutable_obj()->mutable_velocity()->set_z(rel_vz);
      }
    }
  }

  void ReasonablenessCheck(const perception::TrackedObjects& history_frame_objects,
                           perception::TrackedObjects& current_frame_objects,
                           const localization::Localization& local_current) {
    const double time_diff = (current_frame_objects.header().stamp().sec() +
                              current_frame_objects.header().stamp().nsec() * 1e-9) -
                             (history_frame_objects.header().stamp().sec() +
                              history_frame_objects.header().stamp().nsec() * 1e-9);
    if (time_diff < kEpsilon) {
      ROS_WARN_STREAM("ComputeInitialVelocity: Frame time diff is " << time_diff);
      return;
    }
    double host_yaw_global = local_current.yaw();
    double cos_host = std::cos(host_yaw_global);
    double sin_host = std::sin(host_yaw_global);
    double host_velocity = std::sqrt(std::pow(local_current.longitudinal_v(), 2) +
                                  std::pow(local_current.lateral_v(), 2));
    for (int i = 0; i < current_frame_objects.objs_size(); i++) {
      perception::TrackedObject* current_object = current_frame_objects.mutable_objs(i);
      if (current_object->obj().type() != perception::ObjectType::TYPE_BICYCLE ||
          current_object->obj().type() != perception::ObjectType::TYPE_PEDESTRIAN) {
        continue;
      }
      uint32_t current_object_id = current_object->obj().id();
      double latest_utm_vx = (current_object->obj().velocity().x() + host_velocity) * cos_host -
                             current_object->obj().velocity().y() * sin_host;
      double latest_utm_vy = (current_object->obj().velocity().x() + host_velocity) * sin_host +
                             current_object->obj().velocity().y() * cos_host;
      double latest_abs_v = std::sqrt(std::pow(latest_utm_vx, 2) + std::pow(latest_utm_vy, 2));
      if (latest_abs_v*3.6 < kType2SpdThr.at(current_object->obj().type())) {
        continue;
      }
      for (int j = 0; j < history_frame_objects.objs_size(); j++) {
        const perception::TrackedObject& last_object = history_frame_objects.objs(j);
        uint32_t last_object_id = last_object.obj().id();
        if (last_object_id == current_object_id) {
          double his_utm_vx = (last_object.obj().velocity().x() + host_velocity) * cos_host -
                                 last_object.obj().velocity().y() * sin_host;
          double his_utm_vy = (last_object.obj().velocity().x() + host_velocity) * sin_host +
                                 last_object.obj().velocity().y() * cos_host;
          // 默认匀速
          double refine_vx = last_object.obj().velocity().x();
          double refine_vy = last_object.obj().velocity().x();
          // double dev_vx_ego = fusion_object_ptr->velocity(0) * cos_host +
          //                     fusion_object_ptr->velocity(1) * sin_host -
          //                     localization.longitudinal_v();
          // double dev_vy_ego = -fusion_object_ptr->velocity(0) * sin_host +
          //                     fusion_object_ptr->velocity(1) * cos_host - localization.lateral_v();

          current_object->mutable_obj()->mutable_velocity()->set_x(refine_vx);
          current_object->mutable_obj()->mutable_velocity()->set_y(refine_vy);
        }
      }
    }
  }

  void ObjectMatch(const perception::TrackedObjects& objects_fpnet,
                   perception::TrackedObjects& objects_result,
                   std::unordered_map<int, std::vector<int>>& lidar_intersect_fpnet,
                   std::unordered_set<int>& fpnet_match_big) {
    float iou = 0.0;
    float iou_max = 0.0;
    int num = 0;
    int index = -1;
    bool isintersection = false;
    std::vector<int> match_id_vec;
#if 1
  for (int i = 0; i < objects_fpnet.objs_size(); i++) {
    match_id_vec.clear();
    for (int j = 0; j < objects_result.objs_size(); j++) {
      double x_diff =
          objects_fpnet.objs(i).obj().center().x() - objects_result.objs(j).obj().center().x();
      double y_diff =
          objects_fpnet.objs(i).obj().center().y() - objects_result.objs(j).obj().center().y();
      double center_distance = std::sqrt((std::pow(x_diff, 2)) + (std::pow(y_diff, 2)));
      if (center_distance > MATCH_LIMIT_THR) {
        continue;
      }
      isintersection =
          IsPolygonboxIntersection(objects_fpnet.objs(i).obj(), objects_result.objs(j).obj());
      if (isintersection) {
        if (objects_result.objs(j).obj().type() == perception::ObjectType::TYPE_CAR ||
            objects_result.objs(j).obj().type() == perception::ObjectType::TYPE_TRUCK ||
            objects_result.objs(j).obj().type() == perception::ObjectType::TYPE_BUS ||
            objects_result.objs(j).obj().size().y() > 1.0) {
          fpnet_match_big.emplace(objects_fpnet.objs(i).obj().id());
          break;
        }
        match_id_vec.push_back(j);  // 保存一对多
      }
    }
    if (match_id_vec.empty()) {
      match_id_vec.push_back(-1);
    }
    lidar_intersect_fpnet[i] = match_id_vec;
    
  }
#else
  perception::mid_common::DevRotateIouPtr dev_rotate_iou_ptr;
  dev_rotate_iou_ptr.reset(new perception::mid_common::DevRotateIou());
  for (int i = 0; i < objects_fpnet.objs_size(); i++) {
    index = -1;
    for (int j = 0; j < objects_result.objs_size(); j++) {
      iou = ComputeIou2D(objects_fpnet.objs(i).obj(), objects_result.objs(j).obj(),
                         dev_rotate_iou_ptr);
      if (iou_max < iou) {
        iou_max = iou;
        index = j;
      }
    }
    if (iou_max > 0 && index != -1) {
      lidar_intersect_fpnet[i] = index;
    } else {
      lidar_intersect_fpnet[i] = -1;
    }
  }
#endif
}

float IOU_cv(const cv::Rect& r1, const cv::Rect& r2) {
  cv::Rect and_box = r1 | r2;
  cv::Rect U = r1 & r2;
  return U.area() * 1.0 / and_box.area();
}

float EIOU_cv(const cv::Rect& r1, const cv::Rect& r2) {
  //计算eiou：考虑了重叠面积，中心点距离、长宽边长真实差，基于CIOU解决了纵横比的模糊定义
  cv::Rect and_box = r1 | r2;
  cv::Rect U = r1 & r2;
  double iou = U.area() * 1.0 / and_box.area();

  double width_1 = r1.br().x - r1.tl().x;
  double height_1 = r1.br().y - r1.tl().y;

  double width_2 = r2.br().x - r2.tl().x;
  double height_2 = r2.br().y - r2.tl().y;

  //考虑中心点
  //计算每个框的中心点
  double center_x1 = (r1.br().x + r1.tl().x) / 2;
  double center_y1 = (r1.br().y + r1.tl().y) / 2;
  double center_x2 = (r2.br().x + r2.tl().x) / 2;
  double center_y2 = (r2.br().y + r2.tl().y) / 2;
  //计算中心点距离的平方
  double p2 = std::pow((center_x2 - center_x1), 2) + std::pow((center_y2 - center_y1), 2);
  //计算对角线长度的平方
  double width_c = std::max(r1.br().x, r2.br().x) - std::min(r1.tl().x, r2.tl().x);
  double height_c = std::max(r1.br().y, r2.br().y) - std::min(r1.tl().y, r2.tl().y);
  double c2 = std::pow(width_c, 2) + std::pow(height_c, 2);
  return iou - p2 / c2;
  // //考虑横纵比
  // //找到最小外接框的边
  // double left = std::min(r1.tl().x,r2.tl().x);
  // double top = std::min(r1.tl().y,r2.tl().y);
  // double bottom = std::max(r1.br().y,r2.br().y);
  // double right = std::max(r1.br().x,r2.br().x);
  // double cwidth=std::pow((right-left),2);
  // double cheight=std::pow((bottom-top),2);
  // double pwidth=std::pow((width_2-width_1),2);
  // double pheight=std::pow((height_2-height_1),2);
  // return iou - p2 / c2 - pwidth/cwidth - pheight/cheight;

  // //calculate v
  // double arctan = std::atan(width_2 / height_2) -  std::atan(width_1 /
  // height_1); double v = (4.0 / std::pow(PI,2)) * (std::pow(arctan,2));
  // double alpha = v / (1 - iou + v);
  // //calculate ciou(iou - p2 / c2 - alpha * v)
  // return iou - p2 / c2 - alpha * v;
}

void TrackObjectInfoCopy(const perception::TrackedObjects& source_track_object,
                         perception::TrackedObjects& target_track_object) {
  target_track_object.mutable_header()->set_seq(source_track_object.header().seq());
  target_track_object.mutable_header()->mutable_stamp()->set_sec(
      source_track_object.header().stamp().sec());
  target_track_object.mutable_header()->mutable_stamp()->set_nsec(
      source_track_object.header().stamp().nsec());
  target_track_object.mutable_header()->set_frame_id(source_track_object.header().frame_id());
  target_track_object.mutable_header()->set_module_name(source_track_object.header().module_name());
  target_track_object.set_sensor_name(source_track_object.sensor_name());
}

bool IsBoundaryLane(perception::mid_fusion::LaneMarkerDataRecord& lanemark,
                    perception::mid_fusion::Point2d& candidate_pnt) {
  if (lanemark.empty()) {
    ROS_WARN_STREAM("IsBoundaryLane: no lane!!!!");
    return false;
  }
  LaneMarkerData left_boundary_lane;
  LaneMarkerData right_boundary_lane;
  if (!lanemark.left2.points.empty()) {
    left_boundary_lane = lanemark.left2;
  } else if (!lanemark.left_right.points.empty()) {
    left_boundary_lane = lanemark.left_right;
  } else if (!lanemark.left.points.empty()) {
    left_boundary_lane = lanemark.left;
  } else {
    ROS_WARN_STREAM("IsBoundaryLane: no left lane!");
  }
  if (!lanemark.right2.points.empty()) {
    right_boundary_lane = lanemark.right2;
  } else if (!lanemark.right_left.points.empty()) {
    right_boundary_lane=lanemark.right_left;
  } else if (!lanemark.right.points.empty()) {
    right_boundary_lane = lanemark.right;
  } else {
    ROS_WARN_STREAM("IsBoundaryLane: no right lane!");
  }
  // 计算最小距离
  double min_left_dis, min_right_dis;
  min_left_dis = GetDistance(left_boundary_lane.points, candidate_pnt);
  min_right_dis = GetDistance(right_boundary_lane.points, candidate_pnt);
  ROS_INFO_STREAM(H_DEBUG_R << "L " << min_left_dis << " - R " << min_right_dis);
  if (min_left_dis < 0.6 || min_right_dis < 0.6) {
    return true;
  } else {
    return false;
  }
}

double GetDistance(const std::vector<perception::mid_fusion::Point2d>& points,
                   const perception::mid_fusion::Point2d& candidate_pnt) {
  double min_dis = 4.0;
  for (int i = 0; i < points.size(); i++) {
    double dis = std::sqrt((points[i].x - candidate_pnt.x) * (points[i].x - candidate_pnt.x) +
                           (points[i].y - candidate_pnt.y) * (points[i].y - candidate_pnt.y));
    if (dis < min_dis) {
      min_dis = dis;
    }
  }
  return min_dis;
}

void UndisortBox(const perception::VisualObject& camera_object, Eigen::Matrix4d& intrinsics,
                 Eigen::VectorXd& distcoeff, perception::VisualObject& camera_object_undistort) {
  if (distcoeff.size() == 0) {
    ROS_WARN_STREAM("UndisortBox:: Distcoeff Param is NULL");
  }
  // std::cout << " = = = Not undisorted = = = \n"
  //           << camera_object.x() << " " << camera_object.y() << " " << camera_object.width() << " "
  //           << camera_object.height() << std::endl;
  // ru: right-up rd: right-down ld: left-down lu: left-up
  std::vector<cv::Point2f> src_ru, src_rd, src_ld, src_lu;
  src_ru.push_back(cv::Point2f(camera_object.x() + camera_object.width(), camera_object.y()));
  src_rd.push_back(cv::Point2f(camera_object.x() + camera_object.width(),
                               camera_object.y() + camera_object.height()));
  src_ld.push_back(cv::Point2f(camera_object.x(), camera_object.y() + camera_object.height()));
  src_lu.push_back(cv::Point2f(camera_object.x(), camera_object.y()));
  std::vector<cv::Point2f> ru, rd, ld, lu;
  DistortPoint(src_ru, intrinsics, distcoeff, ru);
  DistortPoint(src_rd, intrinsics, distcoeff, rd);
  DistortPoint(src_ld, intrinsics, distcoeff, ld);
  DistortPoint(src_lu, intrinsics, distcoeff, lu);
  double width = ru[0].x - lu[0].x;
  double height = rd[0].y - ru[0].y;
  // std::cout << " = = = undistorted = = = "
  //           << lu[0].x << " " << lu[0].y << " " << width << " " << height << std::endl;
  camera_object_undistort.set_x(lu[0].x);
  camera_object_undistort.set_y(lu[0].y);
  camera_object_undistort.set_width(width);
  camera_object_undistort.set_height(height);
}

void DistortPoint(std::vector<cv::Point2f>& src, Eigen::Matrix4d& intrinsics,
                  Eigen::VectorXd& distcoeff, std::vector<cv::Point2f>& dst) {
  double fx = intrinsics(0, 0);
  double fy = intrinsics(1, 1);
  double cx = intrinsics(0, 2);
  double cy = intrinsics(1, 2);
  double k1 = distcoeff(0);
  double k2 = distcoeff(1);
  double p1 = distcoeff(2);
  double p2 = distcoeff(3);
  cv::Mat cv_cam_matrix_ = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  cv::Mat cv_dist_params_ = (cv::Mat_<float>(4, 1) << k1, k2, p1, p2);
  cv::undistortPoints(src, dst, cv_cam_matrix_, cv_dist_params_, cv::noArray(), cv_cam_matrix_);
}

}  // namespace mid_fusion
}  // namespace perception
