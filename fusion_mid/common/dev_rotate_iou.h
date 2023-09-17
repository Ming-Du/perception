#ifndef DEV_ROTATE_IOU_H
#define DEV_ROTATE_IOU_H

#include <boost/shared_ptr.hpp>
#include <cmath>
#include <iostream>

#include "common/include/log.h"

namespace perception {
namespace mid_common {
using namespace std;

class DevRotateIou {
 public:
  DevRotateIou();
  void demo();
  float devRotateIoUEval(float rbox1[5], float rbox2[5]);

 private:
  double inter(float rbbox1[5], float rbbox2[5]);
  void rbbox_to_corners(float rbbox[5], float* corners);
  int quadrilateral_intersection(float pts1[8], float pts2[8], float* int_pts);
  bool point_in_quadrilateral(float pt_x, float pt_y, float corners[8]);
  bool line_segment_intersection(float pts1[8], float pts2[8], int i, int j, float* temp_pts);
  void sort_vertex_in_convex_polygon(float* int_pts, int num_of_inter);
  float area(float int_pts[16], int num_of_inter);
  float trangle_area(float a[2], float b[2], float c[2]);
};

typedef boost::shared_ptr<DevRotateIou> DevRotateIouPtr;

}  // namespace mid_common
}  // namespace perception

#endif  // DEV_ROTATE_IOU_H
