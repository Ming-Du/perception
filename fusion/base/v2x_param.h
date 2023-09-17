#pragma once

namespace perception {
namespace fusion {
struct v2x_param {
  int v2x_process_mode = 1; // 1:Fusion 2:Blind 3:BeyondRange 4:PassThrough 5:Only V2i
  int pass_through_mode = 1; // 1-beyond visual range 2-blind area 
  float beyond_vis_range = 120;
  float radius_for_fusion_object = 5;
  float zombie_thr = 120000;
  bool enable_only_pub_zombie = false;

  void Reset(){
    v2x_process_mode = 1;
    pass_through_mode = 1;
    beyond_vis_range = 120;
    radius_for_fusion_object = 5;
    zombie_thr = 120000;
    enable_only_pub_zombie = false;
  }
};

}  // namespace fusion
}  // namespace perception