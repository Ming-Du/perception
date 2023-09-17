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
// #include "tracker/mid_fusion_tracker/mid_fusion_tracker.h"

// #include "gtest/gtest.h"

// #include "cyber/common/log.h"
// #include "common/perception_gflags.h"

// namespace perception {
// namespace mid_fusion {

// TEST(MidFusionTrackerTest, mid_fusion_tracker_init_test) {
//   std::unique_ptr<BaseTracker> tracker(new MidFusionTracker());
//   FLAGS_work_root =
//       "/apollo/testdata/"
//       "mid_fusion/mid_fusion_tracker";
//   EXPECT_TRUE(tracker->Init());
//   EXPECT_EQ(tracker->Name(), "MidFusionTracker");
// }

// TEST(MidFusionTrackerTest, mid_fusion_tracker_track_test) {
//   std::unique_ptr<BaseTracker> tracker(new MidFusionTracker());
//   FLAGS_work_root = "./mid_fusion_test_data/mid_fusion_tracker";
//   tracker->Init();
//   mid_fusion::Frame mid_fusion_frame;
//   mid_fusion_frame.timestamp = 123456789.1;
//   mid_fusion_frame.objects.resize(2);
//   mid_fusion_frame.objects[0].reset(new mid_fusion::Object);
//   mid_fusion_frame.objects[0]->track_id = 100;
//   mid_fusion_frame.objects[0]->center << 12.0, 15.0, 0.0;
//   mid_fusion_frame.objects[0]->velocity << 3.0, 4.0, 0.0;
//   mid_fusion_frame.objects[1].reset(new mid_fusion::Object);
//   mid_fusion_frame.objects[1]->track_id = 200;
//   mid_fusion_frame.objects[1]->center << 50.3, 100.4, 0.0;
//   mid_fusion_frame.objects[1]->velocity << 3.0, 4.0, 0.0;
//   TrackerOptions options;
//   mid_fusion::FramePtr tracked_frame(new mid_fusion::Frame);
//   bool state = tracker->Track(mid_fusion_frame, options, tracked_frame);
//   EXPECT_TRUE(state);
// }

// TEST(MidFusionTrackerTest, mid_fusion_tracker_collect_test) {
//   mid_fusion::ObjectPtr object(new mid_fusion::Object);
//   object->track_id = 100;
//   object->center << 10.0, 20.0, 0.0;
//   object->velocity << 3.0, 4.0, 0.0;
//   double timestamp = 123456789.0;
//   MidFusionTrackPtr mid_fusion_track(new MidFusionTrack(object, timestamp));

//   std::unique_ptr<MidFusionTracker> tracker(new MidFusionTracker());
//   FLAGS_work_root = "./mid_fusion_test_data/mid_fusion_tracker";
//   tracker->Init();
//   tracker->track_manager_->ClearTracks();
//   tracker->track_manager_->AddTrack(mid_fusion_track);
//   mid_fusion::FramePtr tracked_frame(new mid_fusion::Frame);
//   tracker->CollectTrackedFrame(tracked_frame);
//   EXPECT_EQ(tracked_frame->objects.size(), 0);
//   MidFusionTrack::SetTrackedTimesThreshold(0);
//   tracker->CollectTrackedFrame(tracked_frame);
//   EXPECT_EQ(tracked_frame->objects.size(), 1);
// }

// TEST(MidFusionTrackerTest, mid_fusion_tracker_unassigned_test) {
//   mid_fusion::ObjectPtr object(new mid_fusion::Object);
//   object->track_id = 100;
//   object->center << 10.0, 20.0, 0.0;
//   object->velocity << 3.0, 4.0, 0.0;
//   double timestamp = 123456789.0;
//   MidFusionTrackPtr mid_fusion_track(new MidFusionTrack(object, timestamp));

//   std::vector<size_t> unassigned_tracks;
//   unassigned_tracks.push_back(0);

//   mid_fusion::Frame mid_fusion_frame;
//   mid_fusion_frame.timestamp =
//       timestamp + MidFusionTracker::s_tracking_time_win_ + 1e-5;

//   std::unique_ptr<MidFusionTracker> tracker(new MidFusionTracker());
//   FLAGS_work_root = "./mid_fusion_test_data/mid_fusion_tracker";
//   tracker->Init();
//   tracker->track_manager_->ClearTracks();
//   tracker->track_manager_->AddTrack(mid_fusion_track);
//   tracker->UpdateUnassignedTracks(mid_fusion_frame, unassigned_tracks);
//   tracker->track_manager_->RemoveLostTracks();
//   EXPECT_EQ(tracker->track_manager_->GetTracks().size(), 0);

//   tracker->track_manager_->ClearTracks();
//   MidFusionTrackPtr mid_fusion_track2(new MidFusionTrack(object, timestamp));
//   tracker->track_manager_->AddTrack(mid_fusion_track2);
//   mid_fusion_frame.timestamp =
//       timestamp + MidFusionTracker::s_tracking_time_win_ - 1e-5;
//   tracker->UpdateUnassignedTracks(mid_fusion_frame, unassigned_tracks);
//   tracker->track_manager_->RemoveLostTracks();
//   EXPECT_EQ(tracker->track_manager_->GetTracks().size(), 1);

//   tracker->track_manager_->ClearTracks();
//   MidFusionTrackPtr mid_fusion_track3(new MidFusionTrack(object, timestamp));
//   mid_fusion_track3->SetObsNullptr();
//   tracker->track_manager_->AddTrack(mid_fusion_track3);
//   mid_fusion_frame.timestamp =
//       timestamp + MidFusionTracker::s_tracking_time_win_ - 1e-5;
//   tracker->UpdateUnassignedTracks(mid_fusion_frame, unassigned_tracks);
//   tracker->track_manager_->RemoveLostTracks();
//   EXPECT_EQ(tracker->track_manager_->GetTracks().size(), 0);
// }

// }  // namespace mid_fusion
// }  // namespace perception
