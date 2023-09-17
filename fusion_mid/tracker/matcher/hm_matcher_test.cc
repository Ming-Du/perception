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
// #include "tracker/matcher/hm_matcher.h"

// #include "gtest/gtest.h"

// #include "cyber/common/log.h"
// #include "common/perception_gflags.h"
// #include "tracker/common/mid_fusion_track.h"
// #include "tracker/common/mid_fusion_track_manager.h"

// namespace perception {
// namespace mid_fusion {

// TEST(HMMatcherTest, hm_matcher_init_test) {
//   BaseMatcher* matcher = new HMMatcher();
//   EXPECT_NE(matcher, nullptr);
//   FLAGS_work_root = "/apollo/testdata/mid_fusion/matcher";
//   EXPECT_TRUE(matcher->Init());
//   delete matcher;
// }

// TEST(HMMatcherTest, hm_matcher_name_test) {
//   BaseMatcher* matcher = new HMMatcher();
//   EXPECT_EQ(matcher->Name(), "HMMatcher");
//   delete matcher;
// }

// TEST(HMMatcherTest, hm_matcher_propterty_match_test) {
//   HMMatcher* matcher = new HMMatcher();
//   std::vector<MidFusionTrackPtr> mid_fusion_tracks;
//   mid_fusion::Frame mid_fusion_frame;
//   std::vector<TrackObjectPair> assignments;
//   std::vector<size_t> unassigned_tracks;
//   std::vector<size_t> unassigned_objects;
//   unassigned_tracks.push_back(0);
//   matcher->TrackObjectPropertyMatch(mid_fusion_tracks, mid_fusion_frame, &assignments,
//                                     &unassigned_tracks, &unassigned_objects);
//   EXPECT_EQ(unassigned_tracks.size(), 1);
//   EXPECT_EQ(unassigned_objects.size(), 0);
//   delete matcher;
// }

// TEST(HMMatcherTest, hm_matcher_test) {
//   BaseMatcher* matcher = new HMMatcher();
//   EXPECT_NE(matcher, nullptr);
//   FLAGS_work_root = "/apollo/testdata/mid_fusion/matcher";
//   EXPECT_TRUE(matcher->Init());
//   EXPECT_EQ(matcher->Name(), "HMMatcher");
//   double match_distance = 2.5;
//   BaseMatcher::SetMaxMatchDistance(match_distance);

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
//   object->center << 10.0, 20.0, 0.0;
//   object->velocity << 3.0, 4.0, 0.0;
//   double timestamp = 123456789.0;
//   MidFusionTrackPtr mid_fusion_track(new MidFusionTrack(object, timestamp));
//   mid_fusion::ObjectPtr object2(new mid_fusion::Object);
//   object2->track_id = 101;
//   object2->center << 50.0, 100.0, 0.0;
//   object2->velocity << 3.0, 4.0, 0.0;
//   MidFusionTrackPtr mid_fusion_track2(new MidFusionTrack(object2, timestamp));
//   mid_fusion::ObjectPtr object3(new mid_fusion::Object);
//   object3->track_id = 102;
//   object3->center << 20.0, 30.0, 0.0;
//   object3->velocity << 3.0, 4.0, 0.0;
//   MidFusionTrackPtr mid_fusion_track3(new MidFusionTrack(object3, timestamp));
//   mid_fusion_tracks.push_back(mid_fusion_track);
//   mid_fusion_tracks.push_back(mid_fusion_track2);
//   mid_fusion_tracks.push_back(mid_fusion_track3);
//   mid_fusion_frame.timestamp = 123456789.1;
//   mid_fusion_frame.objects.resize(3);
//   mid_fusion_frame.objects[0].reset(new mid_fusion::Object);
//   mid_fusion_frame.objects[0]->track_id = 100;
//   mid_fusion_frame.objects[0]->center << 12.0, 15.0, 0.0;
//   mid_fusion_frame.objects[0]->velocity << 3.0, 4.0, 0.0;
//   mid_fusion_frame.objects[1].reset(new mid_fusion::Object);
//   mid_fusion_frame.objects[1]->track_id = 200;
//   mid_fusion_frame.objects[1]->center << 50.3, 100.4, 0.0;
//   mid_fusion_frame.objects[1]->velocity << 3.0, 4.0, 0.0;
//   mid_fusion_frame.objects[2].reset(new mid_fusion::Object);
//   mid_fusion_frame.objects[2]->track_id = 102;
//   mid_fusion_frame.objects[2]->center << 20.3, 30.4, 0.0;
//   mid_fusion_frame.objects[2]->velocity << 3.0, 4.0, 0.0;

//   TrackObjectMatcherOptions options;
//   bool match_state =
//       matcher->Match(mid_fusion_tracks, mid_fusion_frame, options, &assignments,
//                      &unassigned_tracks, &unassigned_objects);
//   EXPECT_EQ(assignments.size(), 2);
//   EXPECT_EQ(unassigned_tracks.size(), 1);
//   EXPECT_EQ(unassigned_objects.size(), 1);
//   EXPECT_TRUE(match_state);
//   delete matcher;
// }

// }  // namespace mid_fusion
// }  // namespace perception
