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

// #include "tracker/interface/base_matcher.h"

// #include "cyber/common/log.h"
// #include "gtest/gtest.h"
// #include "tracker/common/mid_fusion_track.h"
// #include "tracker/common/mid_fusion_track_manager.h"

// namespace perception {
// namespace mid_fusion {

// TEST(BaseMatcherTest, base_matcher_test) {
//   std::unique_ptr<BaseMatcher> matcher(new BaseMatcher());
//   EXPECT_TRUE(matcher->Init());
//   EXPECT_EQ(matcher->Name(), "BaseMatcher");
//   double match_distance = 2.5;
//   BaseMatcher::SetMaxMatchDistance(match_distance);
//   double distance = BaseMatcher::GetMaxMatchDistance();
//   EXPECT_LT(std::fabs(match_distance - distance), 1e-5);

//   std::vector<MidFusionTrackPtr> mid_fusion_tracks;
//   mid_fusion::Frame mid_fusion_frame;
//   std::vector<TrackObjectPair> assignments;
//   std::vector<size_t> unassigned_tracks;
//   std::vector<size_t> unassigned_objects;
//   matcher->IDMatch(mid_fusion_tracks, mid_fusion_frame, &assignments, &unassigned_tracks,
//                    &unassigned_objects);
//   EXPECT_EQ(assignments.size(), 0);
//   EXPECT_EQ(unassigned_tracks.size(), 0);
//   EXPECT_EQ(unassigned_objects.size(), 0);

//   mid_fusion::ObjectPtr object(new mid_fusion::Object);
//   object->track_id = 100;
//   double timestamp = 123456789.0;
//   MidFusionTrackPtr mid_fusion_track(new MidFusionTrack(object, timestamp));
//   mid_fusion::ObjectPtr object2(new mid_fusion::Object);
//   object2->track_id = 101;
//   MidFusionTrackPtr mid_fusion_track2(new MidFusionTrack(object2, timestamp));
//   mid_fusion_tracks.push_back(mid_fusion_track);
//   mid_fusion_tracks.push_back(mid_fusion_track2);
//   mid_fusion_frame.timestamp = 123456789.074;
//   mid_fusion_frame.objects.resize(2);
//   mid_fusion_frame.objects[0].reset(new mid_fusion::Object);
//   mid_fusion_frame.objects[0]->track_id = 100;
//   mid_fusion_frame.objects[1].reset(new mid_fusion::Object);
//   mid_fusion_frame.objects[1]->track_id = 200;
//   matcher->IDMatch(mid_fusion_tracks, mid_fusion_frame, &assignments, &unassigned_tracks,
//                    &unassigned_objects);
//   EXPECT_EQ(assignments.size(), 1);
//   EXPECT_EQ(unassigned_tracks.size(), 1);
//   EXPECT_EQ(unassigned_objects.size(), 1);

//   TrackObjectMatcherOptions options;
//   bool match_state =
//       matcher->Match(mid_fusion_tracks, mid_fusion_frame, options, &assignments,
//                      &unassigned_tracks, &unassigned_objects);
//   EXPECT_TRUE(match_state);
// }

// }  // namespace mid_fusion
// }  // namespace perception
