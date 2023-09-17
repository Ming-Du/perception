//
// Created by tiger on 2022/8/9.
//
#include <gtest/gtest.h>
#include <stdio.h>
#include <stdlib.h>
#include "Eigen/Core"
#include "base/sensor_object.h"
#include "lib/data_association/hm_data_association/track_object_distance.h"

class TrackObjectDistanceTest : public perception::fusion::TrackObjectDistance {
 public:
  TrackObjectDistanceTest() = default;
  float Test() {
    Eigen::Vector3d host_position(460406, 4.43619e+06, 17.8823);  // UMT coordinate
    Eigen::Vector3d position(460412, 4.43619e+06, 0);
    // Eigen::Vector3d velocity(3.65055, -12.1448, 0);
    Eigen::Vector3d center_ego(4.28858, 5.26582, 0.996138);
    Eigen::Vector3f size(4.55734, 2.04868, 1.79336);
    Eigen::Vector3d center(27.2834, 4.70416, 0.999437);
    Eigen::Vector3f velocity(1.87238, 0.62777, 0);
    // Eigen::Vector3f acceleration();
    obj1.id = 476;
    obj1.host_yaw = 5.43682;
    obj1.theta = -0.46669;
    obj1.theta_variance = -0.46669 + 0.0001;
    obj1.host_position = host_position;
    obj1.position = position;
    obj1.position_uncertainty = Eigen::Matrix3f::Identity() * 0.02;
    obj1.center_ego = center_ego;
    obj1.center = center;
    obj1.center_uncertainty = Eigen::Matrix3f::Identity() * 0.02;
    obj1.size = size;
    obj1.size_variance = Eigen::Vector3f::Identity() * 0.02;
    obj1.anchor_point = center;
    obj1.type = perception::fusion::ObjectType::UNKNOWN;
    obj1.type_probs.assign(static_cast<int>(perception::fusion::ObjectType::MAX_OBJECT_TYPE),
                           0.01f);
    obj1.sub_type = perception::fusion::ObjectSubType::UNKNOWN;
    obj1.sub_type_probs.assign(static_cast<int>(perception::fusion::ObjectSubType::MAX_OBJECT_TYPE),
                               0.01f);
    obj1.confidence = 0.8;
    obj1.track_id = 100;
    obj1.match_id_radar = 20;
    obj1.match_id_lidar = 20;
    obj1.match_id_camera = 20;
    obj1.match_id_falcon = 20;
    obj.match_id_obu = 77;
    obj1.velocity = velocity;
    obj1.velocity_uncertainty = Eigen::Matrix3f::Identity() * 0.02;
    obj1.velocity_converged = true;
    obj1.velocity_confidence = 0.1;
    // obj1.acceleration = ;
    // obj1.acceleration_uncertainty = ;
    // obj1.tracking_time = ;
    // obj1.latest_tracked_time = ;
    obj1.motion_state = perception::fusion::MotionState::UNKNOWN;
    perception::fusion::SensorFrameHeader frame_header;
    frame_header.timestamp = 1658305166.676520;
    std::shared_ptr<const perception::fusion::Object> object_ptr(
        new perception::fusion::Object(obj1));
    std::shared_ptr<const perception::fusion::SensorFrameHeader> frame_header_ptr(
        new perception::fusion::SensorFrameHeader(frame_header));

    perception::fusion::SensorObject fused_object(object_ptr, frame_header_ptr);
    perception::fusion::SensorObject sensor_object(object_ptr, frame_header_ptr);
    // perception::fusion::TrackObjectDistance track_object_distance;

    perception::fusion::SensorObjectConstPtr fused_object_ptr(
        new perception::fusion::SensorObject(fused_object));
    perception::fusion::SensorObjectPtr sensor_object_ptr(
        new perception::fusion::SensorObject(sensor_object));

    float val = ComputeLidarLidar(fused_object_ptr, sensor_object_ptr, *ref_point, range);
    return val;
  }

 private:
  perception::fusion::Object obj1;
  perception::fusion::Object obj2;
  Eigen::Vector3d* ref_point = nullptr;
  int range = 10;
};

TEST(Test1, case1) {
  TrackObjectDistanceTest track_object_distance;
  // track_object_distance.Test();
  EXPECT_EQ(track_object_distance.Test(), 0);
  // SensorObjectConstPtr fused_object;
  // SensorObjectPtr sensor_object;
  // Eigen::Vector3d ref_points;

  // perception::fusion::SensorObject fused_object;
  // perception::fusion::SensorObject sensor_object;
}
// TEST(Test1, case2) {}
// TEST(Test1, case3) {}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
