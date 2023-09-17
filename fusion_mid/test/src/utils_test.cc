
/**
 * @file test_utils.cc
 * @brief unit test file for utils.cc
 * @author Jiahui Lu, Liwei Jiang
 * @date 2023/02/07
 */

// headers in gtest
#include "base/utils.h"
#include "geometry_msgs/Point.h"

#include <gtest/gtest.h>
#include <Eigen/Core>

#include <iostream>

namespace perception{
namespace mid_fusion{

TEST(mid_fusion_utils_test_cases, ComputeIou2DTest) {
  perception::Object *object1 = new perception::Object;
  perception::Object *object2 = new perception::Object;
  object1->mutable_center()->set_x(0.0);
  object1->mutable_center()->set_y(0.0);
  object1->mutable_size()->set_x(5.0);
  object1->mutable_size()->set_y(5.0);
  object1->set_angle(0.0);

  object2->mutable_center()->set_x(2.5);
  object2->mutable_center()->set_y(0.0);
  object2->mutable_size()->set_x(5.0);
  object2->mutable_size()->set_y(5.0);
  object2->set_angle(0.0);

  perception::mid_common::DevRotateIouPtr dev_rotate_iou_ptr;
  dev_rotate_iou_ptr.reset(new perception::mid_common::DevRotateIou());
  // Normal test
  EXPECT_NEAR(0.33333333333333, ComputeIou2D(*object1, *object2, dev_rotate_iou_ptr), 1e-6);
  // Total overlap
  object2->mutable_center()->set_x(0.0);
  EXPECT_NEAR(1.0, ComputeIou2D(*object1, *object2, dev_rotate_iou_ptr), 1e-6);
  // No overlap
  object2->mutable_center()->set_x(10.0);
  EXPECT_NEAR(0.0, ComputeIou2D(*object1, *object2, dev_rotate_iou_ptr), 1e-6);
}

TEST(mid_fusion_utils_test_cases, IsPolygonboxIntersectionTest) {
  perception::Object *object1 = new perception::Object;
  perception::Object *object2 = new perception::Object;
  std::vector<std::pair<double, double> > polygon1_points = {
      {2.5, 2.5}, {2.5, -2.5}, {-2.5, 2.5}, {-2.5, -2.5}};
  for (int i = 0; i < polygon1_points.size(); i++) {
    auto point = object1->add_polygon();
    point->set_x(polygon1_points[i].first);
    point->set_y(polygon1_points[i].second);
  }
  // Polygon intersect is true.
  std::vector<std::pair<double, double> > polygon2_points = {
      {2.5, 2.5}, {2.5, -2.5}, {-2.5, 2.5}, {-2.5, -2.5}};
  for (int i = 0; i < polygon2_points.size(); i++) {
    auto point = object2->add_polygon();
    point->set_x(polygon2_points[i].first);
    point->set_y(polygon2_points[i].second);
  }
  EXPECT_TRUE(IsPolygonboxIntersection(*object1, *object2));

  // Polygon intersect is false.
  object2->mutable_polygon()->Clear();
  polygon2_points = {{10.5, 10.5}, {12.5, 12.5}, {10.5, 12.5}, {12.5, 10.5}};
  for (int i = 0; i < polygon2_points.size(); i++) {
    auto point = object2->add_polygon();
    point->set_x(polygon2_points[i].first);
    point->set_y(polygon2_points[i].second);
  }
  EXPECT_FALSE(IsPolygonboxIntersection(*object1, *object2));
}

TEST(mid_fusion_utils_test_cases, GetObjectVelocityTest) {
  // Get last frame objects
  perception::TrackedObjects last_frame_objects;
  last_frame_objects.mutable_header()->mutable_stamp()->set_sec(10.0);
  last_frame_objects.mutable_header()->mutable_stamp()->set_nsec(123456789);
  TrackedObject* lidar_track_object1 = last_frame_objects.add_objs();
  perception::Object *object1 = lidar_track_object1->mutable_obj();
  object1->set_id(1);
  object1->mutable_center()->set_x(5.0);
  object1->mutable_center()->set_y(10.0);
  object1->mutable_center()->set_z(0.0);
  object1->set_type(perception::ObjectType::TYPE_PEDESTRIAN);

  TrackedObject* lidar_track_object2 = last_frame_objects.add_objs();
  perception::Object *object2 = lidar_track_object2->mutable_obj();
  object2->set_id(2);
  object2->mutable_center()->set_x(4.0);
  object2->mutable_center()->set_y(4.0);
  object2->mutable_center()->set_z(0.0);
  object2->set_type(perception::ObjectType::TYPE_TRIANGLEROADBLOCK);

  // Get current frame objects
  perception::TrackedObjects current_frame_objects;
  current_frame_objects.mutable_header()->mutable_stamp()->set_sec(11.0);
  current_frame_objects.mutable_header()->mutable_stamp()->set_nsec(123456789);
  TrackedObject* cur_lidar_track_object1 = current_frame_objects.add_objs();
  perception::Object *cur_object1 = cur_lidar_track_object1->mutable_obj();
  cur_object1->set_id(1);
  cur_object1->mutable_center()->set_x(6.0);
  cur_object1->mutable_center()->set_y(12.0);
  cur_object1->mutable_center()->set_z(-3.0);
  cur_object1->set_type(perception::ObjectType::TYPE_PEDESTRIAN);

  TrackedObject* cur_lidar_track_object2 = current_frame_objects.add_objs();
  perception::Object *cur_object2 = cur_lidar_track_object2->mutable_obj();
  cur_object2->set_id(2);
  cur_object2->mutable_center()->set_x(4.0);
  cur_object2->mutable_center()->set_y(4.0);
  cur_object2->mutable_center()->set_z(0.0);
  cur_object2->set_type(perception::ObjectType::TYPE_TRIANGLEROADBLOCK);

  GetObjectVelocity(last_frame_objects, current_frame_objects);

  perception::TrackedObject* current_object1 = current_frame_objects.mutable_objs(0);
  EXPECT_NEAR(current_object1->obj().velocity().x(), 1.0, 1e-6);
  EXPECT_NEAR(current_object1->obj().velocity().y(), 2.0, 1e-6);
  EXPECT_NEAR(current_object1->obj().velocity().z(), -3.0, 1e-6);

  perception::TrackedObject* current_object2 = current_frame_objects.mutable_objs(1);
  EXPECT_NEAR(current_object2->obj().velocity().x(), 0.0, 1e-6);
  EXPECT_NEAR(current_object2->obj().velocity().y(), 0.0, 1e-6);
}

TEST(mid_fusion_utils_test_cases, ComputeNearDistance) {
  // case 1
  // Lidar Obj corners
  std::vector<float> corners1;
  float x1{0.0F};
  float y1{0.0F};
  float x2{5.0F};
  float y2{0.0F};
  float x3{5.0F};
  float y3{3.0F};
  float x4{0.0F};
  float y4{3.0F};
  corners1.push_back(x1);
  corners1.push_back(y1);
  corners1.push_back(x2);
  corners1.push_back(y2);
  corners1.push_back(x3);
  corners1.push_back(y3);
  corners1.push_back(x4);
  corners1.push_back(y4);
  // Radar Obj
  perception::RadarObject object2;
  object2.mutable_obj()->mutable_center()->set_x(5.0F);
  object2.mutable_obj()->mutable_center()->set_y(4.0F);
  float min_dis{perception::mid_fusion::ComputeNearDistance(corners1, &object2)};
  EXPECT_FLOAT_EQ(1.0F, min_dis);

  // case 2
  object2.mutable_obj()->mutable_center()->set_x(4.0F);
  object2.mutable_obj()->mutable_center()->set_y(1.0F);
  min_dis = perception::mid_fusion::ComputeNearDistance(corners1, &object2);
  EXPECT_FLOAT_EQ(sqrt(2), min_dis);

  // case3
  object2.mutable_obj()->mutable_center()->set_x(0.0F);
  object2.mutable_obj()->mutable_center()->set_y(1.5F);
  min_dis = perception::mid_fusion::ComputeNearDistance(corners1, &object2);
  EXPECT_FLOAT_EQ(1.5F, min_dis);
}

TEST(mid_fusion_utils_test_cases, ExtractCorners) {
  perception::Object object;
  std::vector<float> corners_lidarobject;
  ::geometry::Point* contours0{object.add_contour()};
  ::geometry::Point* contours1{object.add_contour()};
  ::geometry::Point* contours2{object.add_contour()};
  ::geometry::Point* contours3{object.add_contour()};
  contours0->set_x(0.0F);
  contours0->set_y(0.0F);
  contours1->set_x(5.0F);
  contours1->set_y(0.0F);
  contours2->set_x(5.0F);
  contours2->set_y(3.0F);
  contours3->set_x(0.0F);
  contours3->set_y(3.0F);
  corners_lidarobject=perception::mid_fusion::ExtractCorners(object);
  EXPECT_FLOAT_EQ(3.0F, corners_lidarobject.at(5));
}

TEST(mid_fusion_utils_test_cases, FindBboxPoint) {
  // case1
  perception::Object object;
  std::vector<geometry_msgs::Point> box_points;
  bool is_box_corner{true};
  object.mutable_center()->set_x(-1.0F);
  object.mutable_center()->set_y(0.0F);
  object.mutable_center()->set_z(0.5F);
  ::geometry::Vector3* temp_size = object.mutable_size();
  temp_size->set_x(10.0F);
  temp_size->set_y(10.0F);
  temp_size->set_z(10.0F);
  object.set_angle(30.0F);
  ::geometry::Point* contours0{object.add_contour()};
  ::geometry::Point* contours1{object.add_contour()};
  ::geometry::Point* contours2{object.add_contour()};
  ::geometry::Point* contours3{object.add_contour()};
  contours0->set_x(0.0F);
  contours0->set_y(0.0F);
  contours1->set_x(5.0F);
  contours1->set_y(0.0F);
  contours2->set_x(5.0F);
  contours2->set_y(3.0F);
  contours3->set_x(0.0F);
  contours3->set_y(3.0F);
  perception::mid_fusion::FindBboxPoint(object, box_points, is_box_corner);
  Eigen::Vector3f check_point;
  Eigen::Vector3f check_point_after;
  check_point << 4.0F, 5.0F, 5.5F;
  Eigen::Matrix3f rotation;
  rotation << cos(M_PI / 6.0F), sin(M_PI / 6.0F), 0, -sin(M_PI / 6.0F), cos(M_PI / 6.0F), 0, 0, 0,
      1.0F;
  check_point_after = rotation * check_point;
  EXPECT_FLOAT_EQ(sqrt(3.0F) * 2.0F + 2.5F, check_point_after(0));

  // case2
  is_box_corner = false;
  box_points.clear();
  perception::mid_fusion::FindBboxPoint(object, box_points, is_box_corner);
  EXPECT_FLOAT_EQ(10.0F, box_points.at(1).z);
  EXPECT_FLOAT_EQ(5.0F, box_points.at(2).x);
}

TEST(mid_fusion_utils_test_cases, SetAssoDisThr) {
  // case1
  float range{61.0F};
  double x{53.0F};
  double y{6.0F};
  float ret{perception::mid_fusion::SetAssoDisThr(range, x, y)};
  EXPECT_FLOAT_EQ(60.F / 59.F * 2.F + 1.F, ret);

  // case2
  range = 61.F;
  x = 7.F;
  y = 1.8F;
  ret = perception::mid_fusion::SetAssoDisThr(range, x, y);
  EXPECT_FLOAT_EQ(1.5F, ret);

  // case3
  range = 10.F;
  x = 7.F;
  y = 1.F;
  ret = perception::mid_fusion::SetAssoDisThr(range, x, y);
  EXPECT_FLOAT_EQ(1.F, ret);
}



TEST(mid_fusion_utils_test_cases, IOUTest) {
  cv::Rect r1(1, 1, 1, 1);
  cv::Rect r2(1, 1, 1, 1);
  /* EXPECT_NEAR(val1, val2, abs_error); */
  // total overlap
  EXPECT_NEAR(1.0,IOU_cv(r1, r2),0.0);
  cv::Rect r3(1, 1, 1, 1);
  cv::Rect r4(3, 2, 1, 1);
  // no overlap
  EXPECT_NEAR(0.0,IOU_cv(r3, r4),0.0);
  // normal overlap 
  // note: cv::Rect - Discard data after the decimal point
  cv::Rect r5(1, 1, 2, 2);
  cv::Rect r6(2, 1, 2, 2);
  EXPECT_NEAR(0.33333333,IOU_cv(r5, r6),0.05);
}

TEST(mid_fusion_utils_test_cases, EIOUTest) {
  cv::Rect r1(1, 1, 1, 1);
  cv::Rect r2(1, 1, 1, 1);
  // total overlap
  // L_IOU - L_dis / L_asp
  EXPECT_NEAR(1.0,EIOU_cv(r1, r2),0.0);
  cv::Rect r3(2, 2, 2, 2);
  cv::Rect r4(6, 4, 2, 2);
  // no overlap
  EXPECT_NEAR(-0.3846,EIOU_cv(r3, r4),0.01);
  // normal overlap
  cv::Rect r5(1, 1, 2, 2);
  cv::Rect r6(2, 1, 2, 2);
  EXPECT_NEAR(0.2560,EIOU_cv(r5, r6),0.01);
}

TEST(mid_fusion_utils_test_cases, MarginProcessTest) {
  perception::VisualObjects camera_2d_box;
  camera_2d_box.set_width(1920);
  camera_2d_box.set_height(1080);
  VisualObject* visual_obj1 = camera_2d_box.add_objs();
  visual_obj1->set_x(15);
  visual_obj1->set_y(15);
  visual_obj1->set_width(2);
  visual_obj1->set_height(2);

  VisualObject* visual_obj2 = camera_2d_box.add_objs();
  visual_obj2->set_x(1000);
  visual_obj2->set_y(1070);
  visual_obj2->set_width(20);
  visual_obj2->set_height(10);

  VisualObject* visual_obj3 = camera_2d_box.add_objs();
  visual_obj3->set_x(1910);
  visual_obj3->set_y(500);
  visual_obj3->set_width(10);
  visual_obj3->set_height(20);

  std::vector<int> margin_vec(camera_2d_box.objs_size(), -1);
  int margin = 10;
  EXPECT_NEAR(1,MarginProcess(camera_2d_box, margin_vec, margin),0.01);
}

TEST(mid_fusion_utils_test_cases, FpnetobjectMatchTest) {
  perception::TrackedObjects lidar_objects_fpnet_result;
  perception::TrackedObjects lidar_objects_fpnet;
  perception::TrackedObject lidar_obj_temp;
  perception::Object* object1 = lidar_obj_temp.mutable_obj();
  perception::Object* object2 = lidar_obj_temp.mutable_obj();
  std::string camera_sensor_name30 = "/perception/camera/camera_obstacle_front30";
  std::string camera_sensor_name60 = "/perception/camera/camera_obstacle_front60";
  object1->set_sensor_name(camera_sensor_name30);
  object2->set_sensor_name(camera_sensor_name60);
    std::vector<std::pair<double, double> > polygon1_points = {
      {2.5, 2.5}, {2.5, -2.5}, {-2.5, 2.5}, {-2.5, -2.5}};
  for (int i = 0; i < polygon1_points.size(); i++) {
    auto point = object1->add_polygon();
    point->set_x(polygon1_points[i].first);
    point->set_y(polygon1_points[i].second);
  }
  // Polygon intersect is true.
  std::vector<std::pair<double, double> > polygon2_points = {
      {2.5, 2.5}, {2.5, -2.5}, {-2.5, 2.5}, {-2.5, -2.5}};
  for (int i = 0; i < polygon2_points.size(); i++) {
    auto point = object2->add_polygon();
    point->set_x(polygon2_points[i].first);
    point->set_y(polygon2_points[i].second);
  } 

  // EXPECT_NEAR(1,FpnetobjectMatch(lidar_objects_fpnet, lidar_objects_fpnet_result),0.01);

  // Polygon intersect is false.
  object2->mutable_polygon()->Clear();
  polygon2_points = {{10.5, 10.5}, {12.5, 12.5}, {10.5, 12.5}, {12.5, 10.5}};
  for (int i = 0; i < polygon2_points.size(); i++) {
    auto point = object2->add_polygon();
    point->set_x(polygon2_points[i].first);
    point->set_y(polygon2_points[i].second);
  }
  perception::TrackedObject* result_object = lidar_objects_fpnet.add_objs();
  result_object->CopyFrom(lidar_obj_temp);  
}

TEST(mid_fusion_utils_test_cases, TrackObjectInfoCopyTest) {
  perception::TrackedObjects source_tracks;
  source_tracks.mutable_header()->set_seq(10000);
  source_tracks.mutable_header()->mutable_stamp()->set_sec(10.0);
  source_tracks.mutable_header()->mutable_stamp()->set_nsec(123456789);
  source_tracks.mutable_header()->set_frame_id("base_link");
  source_tracks.mutable_header()->set_module_name("fusion");
  source_tracks.set_sensor_name("lidar");

  perception::TrackedObjects target_tracks;
  TrackObjectInfoCopy(source_tracks, target_tracks);
  EXPECT_EQ(target_tracks.header().seq(), 10000);
  EXPECT_NEAR(target_tracks.header().stamp().sec(), 10, 1e-6);
  EXPECT_NEAR(target_tracks.header().stamp().nsec(), 123456789, 1e-6);
  EXPECT_EQ(target_tracks.header().frame_id(), "base_link");
  EXPECT_EQ(target_tracks.header().module_name(), "fusion");
  EXPECT_EQ(target_tracks.sensor_name(), "lidar");
}

} // mid_fusion
} // perception
