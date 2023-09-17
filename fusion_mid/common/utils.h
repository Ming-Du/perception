#pragma once

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <geos/geom/Coordinate.h>
#include <opencv2/opencv.hpp>

#include "common/dev_rotate_iou.h"
#include "geometry_msgs/Point.h"
#include "common/proto/object.pb.h"
#include "common/proto/localization.pb.h"
#include "geos/geom/CoordinateArraySequence.h"
#include "geos/geom/Geometry.h"
#include "geos/geom/GeometryFactory.h"
#include "geos/io/WKTReader.h"
#include "define.h"
#include <ros/ros.h>

#include "perception/fusion_mid/common/vec2.h"
#include "perception/fusion_mid/common/mid_struct.h"

namespace perception {
namespace mid_fusion {
using geos::geom::Coordinate;
using geos::geom::CoordinateSequence;
using geos::geom::Geometry;
using geos::geom::GeometryFactory;
using namespace geos::geom;

const std::map<perception::ObjectType, double> kType2SpdThr = {
    {perception::ObjectType::TYPE_BICYCLE, BIC_SPEED_THRESHOLD},
    {perception::ObjectType::TYPE_PEDESTRIAN, PED_SPEED_THRESHOLD},
};

void FindBboxPoint(const perception::Object& obj,
                   std::vector<geometry_msgs::Point>& box_points,
                   const bool is_box_corner);

float ComputeNearDistance(std::vector<float>& corners1, perception::RadarObject* object2_ptr);

std::vector<float> ExtractCorners(const perception::Object& object);

float SetAssoDisThr(float range, double x, double y);

float ComputeIou2D(const perception::Object& object1,
                   const perception::Object& object2,
                   const perception::mid_common::DevRotateIouPtr& dev_rotate_iou_ptr);

bool IsPolygonboxIntersection(const perception::Object& object1, const perception::Object& object2);

void ComputeInitialVelocity(const perception::TrackedObjects& last_frame_objects,
                            perception::TrackedObjects& current_frame_objects,
                            std::map<uint32_t, double> fpnet_pred_obj_count,
                            const localization::Localization& local_current);

void ReasonablenessCheck(const perception::TrackedObjects& history_frame_objects,
                         perception::TrackedObjects& current_frame_objects,
                         const localization::Localization& local_current);

void ObjectMatch(const perception::TrackedObjects& objects_fpnet,
                 perception::TrackedObjects& objects_result,
                 std::unordered_map<int, std::vector<int>>& lidar_intersect_fpnet,
                 std::unordered_set<int>& fpnet_match_big);

bool IsIntersection(const perception::Object& A, const perception::Object& B);

bool IsLineIntersection(const perception::Object& A, const perception::Object& B);
/*
 *@brief: Delete the duplicate target of camera30 and camera60
 *@author:liuxinyu
 */
void FpnetobjectMatch(
    const ::google::protobuf::RepeatedPtrField<::perception::TrackedObject>& objs_input,
    ::google::protobuf::RepeatedPtrField<::perception::TrackedObject>* objs_output);

/*
 *@brief: Filter camera edge half-frame target
 *@author:liuxinyu
 */
int MarginProcess(const perception::VisualObjects& camera_2dboxs,
                  std::vector<int>& margin_vec,
                  const int margin);

float IOU_cv(const cv::Rect& r1, const cv::Rect& r2);

float EIOU_cv(const cv::Rect& r1, const cv::Rect& r2);

void TrackObjectInfoCopy(const perception::TrackedObjects& source_track_object,
                         perception::TrackedObjects& target_track_object);

bool IsBoundaryLane(perception::mid_fusion::LaneMarkerDataRecord& lanemark,
                    perception::mid_fusion::Point2d& candidate_pnt);
double GetDistance(const std::vector<perception::mid_fusion::Point2d>& points,
                   const perception::mid_fusion::Point2d& candidate_pnt);

void UndisortBox(const perception::VisualObject& camera_object, Eigen::Matrix4d& intrinsics,
                 Eigen::VectorXd& distcoeff, perception::VisualObject& camera_object_undisort);
void DistortPoint(std::vector<cv::Point2f>& src, Eigen::Matrix4d& intrinsics,
                  Eigen::VectorXd& distcoeff, std::vector<cv::Point2f>& dst);
}  // namespace mid_fusion
}  // namespace perception
