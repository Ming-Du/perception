/**
 * @file test_utils.cc
 * @brief unit test file for mid_fusion_ros.cc
 * @author Jiahui Lu
 * @date 2023/02/07
 */

// headers in gtest
#include "mid_fusion_ros.h"

#include <gtest/gtest.h>

class MidFusionROSTest : public perception::mid_fusion::MidFusionROS 
{
public: 
    MidFusionROSTest()=default;
    ~MidFusionROSTest()=default;
    inline bool IsFilterForRadarTest(const perception::TrackedObject* lidar_track){
        IsFilterForRadar(lidar_track);
    }

};

// TEST(mid_fusion_ros_test_cases,IsFilterForRadarTest)
// {
//     perception::TrackedObject lidar_track;
//     ::geometry::Point* contours0{lidar_track.mutable_obj()->add_contour()};
//     ::geometry::Point* contours1{lidar_track.mutable_obj()->add_contour()};
//     ::geometry::Point* contours2{lidar_track.mutable_obj()->add_contour()};
//     ::geometry::Point* contours3{lidar_track.mutable_obj()->add_contour()};
//     contours0->set_x(0.0F);
//     contours0->set_y(0.0F);
//     contours1->set_x(5.0F);
//     contours1->set_y(0.0F);
//     contours2->set_x(5.0F);
//     contours2->set_y(3.0F);
//     contours3->set_x(0.0F);
//     contours3->set_y(3.0F);
//     MidFusionROSTest test;
//     bool flag{test.IsFilterForRadarTest(&lidar_track)};
//     EXPECT_TRUE(flag);
// }

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}