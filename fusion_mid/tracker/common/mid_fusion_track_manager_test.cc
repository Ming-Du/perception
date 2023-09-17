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

// #include "tracker/common/mid_fusion_track_manager.h"

// // #include "cyber/common/log.h"
// #include "gtest/gtest.h"
// #include "tracker/filter/adaptive_kalman_filter.h"

// namespace perception {
// namespace mid_fusion {
// TEST(MidFusionTrackManagerTest, mid_fusion_track_manager_test) {
//   mid_fusion::ObjectPtr object(new mid_fusion::Object);
//   object->track_id = 100;
//   object->center << 10.0, 20.0, 0.0;
//   object->velocity << 3.0, 4.0, 0.0;
//   double timestamp = 123456789.0;
//   MidFusionTrackPtr mid_fusion_track(new MidFusionTrack(object, timestamp));
//   MidFusionTrackPtr mid_fusion_track2(new MidFusionTrack(object, timestamp));

//   MidFusionTrackManager* manager = new MidFusionTrackManager();
//   EXPECT_NE(manager, nullptr);
//   manager->AddTrack(mid_fusion_track);
//   auto& mid_fusion_tracks = manager->mutable_tracks();
//   EXPECT_EQ(mid_fusion_tracks.size(), 1);
//   manager->AddTrack(mid_fusion_track2);
//   manager->RemoveLostTracks();
//   EXPECT_EQ(manager->GetTracks().size(), 2);
//   mid_fusion_tracks[0]->SetDead();
//   manager->RemoveLostTracks();
//   EXPECT_EQ(manager->GetTracks().size(), 1);
//   manager->ClearTracks();
//   EXPECT_EQ(manager->GetTracks().size(), 0);
//   delete manager;
// }

// }  // namespace mid_fusion
// }  // namespace perception
