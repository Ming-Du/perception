// /******************************************************************************
//  * Copyright 2018 The Apollo Authors. All Rights Reserved.
//  *
//  * Licensed under the Apache License, Version 2.0 (the License);
//  * you may not use this file except in compliance with the License.
//  * You may obtain a copy of the License at
//  *
//  * http://www.apache.org/licenses/LICENSE-2.0
//  *
//  * Unless required by applicable law or agreed to in writing, software
//  * distributed under the License is distributed on an AS IS BASIS,
//  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  * See the License for the specific language governing permissions and
//  * limitations under the License.
//  *****************************************************************************/
// #include "tracker/common/mid_fusiontrack.h"

// #include "gtest/gtest.h"

// #include "cyber/common/log.h"
// #include "tracker/filter/adaptive_kalman_filter.h"

// namespace perception {

// namespace mid_fusion {
// TEST(MidFusionTrackTest, mid_fusiontrack_test) {
//   int tracked_times_threshold = 1;
//   MidFusionTrack::SetTrackedTimesThreshold(tracked_times_threshold);
//   std::string chosen_filter = "AdaptiveKalmanFilter";
//   MidFusionTrack::SetChosenFilter(chosen_filter);
//   bool use_filter = false;
//   MidFusionTrack::SetUseFilter(use_filter);

//   mid_fusion::ObjectPtr object(new mid_fusion::Object);
//   object->track_id = 100;
//   object->center << 10.0, 20.0, 0.0;
//   object->velocity << 3.0, 4.0, 0.0;
//   double timestamp = 123456789.0;
//   MidFusionTrackPtr mid_fusiontrack(new MidFusionTrack(object, timestamp));
//   EXPECT_EQ(mid_fusiontrack->GetObsId(), 0);
//   EXPECT_FALSE(mid_fusiontrack->ConfirmTrack());
//   EXPECT_FALSE(mid_fusiontrack->IsDead());
//   mid_fusiontrack->SetDead();
//   EXPECT_TRUE(mid_fusiontrack->IsDead());

//   mid_fusion::ObjectPtr object2(new mid_fusion::Object);
//   object2->track_id = 100;
//   object2->center << 10.3, 20.4, 0.0;
//   object2->velocity << 3.0, 4.0, 0.0;
//   double timestamp2 = 123456789.1;
//   mid_fusiontrack->UpdataDetectedObs(object2, timestamp2);
//   EXPECT_TRUE(mid_fusiontrack->ConfirmTrack());

//   MidFusionTrackPtr mid_fusiontrack2(new MidFusionTrack(object, timestamp));
//   use_filter = true;
//   MidFusionTrack::SetUseFilter(use_filter);
//   mid_fusion::ObjectPtr object3(new mid_fusion::Object);
//   object3->track_id = 100;
//   object3->center << 10.3, 20.4, 0.0;
//   object3->velocity << 3.0, 4.0, 0.0;
//   double timestamp3 = 123456789.1;
//   mid_fusiontrack2->UpdataDetectedObs(object3, timestamp3);
//   EXPECT_LT(mid_fusiontrack->GetTrackingTime() - 0.1, 1e-5);
//   EXPECT_TRUE(mid_fusiontrack2->ConfirmTrack());

//   chosen_filter = "Default";
//   MidFusionTrack::SetChosenFilter(chosen_filter);
//   MidFusionTrackPtr mid_fusiontrack3(new MidFusionTrack(object, timestamp));
// }

// TEST(MidFusionTrackTest, mid_fusiontrack_function_test) {
//   mid_fusion::ObjectPtr object(new mid_fusion::Object);
//   object->track_id = 100;
//   object->center << 10.0, 20.0, 0.0;
//   object->velocity << 3.0, 4.0, 0.0;
//   double timestamp = 123456789.0;
//   MidFusionTrackPtr mid_fusiontrack(new MidFusionTrack(object, timestamp));
//   EXPECT_LT(std::fabs(mid_fusiontrack->GetTrackingTime() - 0.0), 1e-5);
//   EXPECT_LT(std::fabs(mid_fusiontrack->GetTimestamp() - timestamp), 1e-5);
//   mid_fusiontrack->SetObsNullptr();
//   EXPECT_EQ(mid_fusiontrack->GetDetectedObs(), nullptr);
//   EXPECT_EQ(mid_fusiontrack->GetObs(), nullptr);
// }

// }  // namespace mid_fusion
// }  // namespace perception
