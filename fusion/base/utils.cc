
#include"utils.h"

namespace perception {
namespace fusion {

int pointInRegion(double x, double y, 
perception::fusion::PointCloud<perception::fusion::PointD> ptlist) {
  int num = ptlist.size();
  int nCross = 0;
  double x1, y1, x2, y2;
  for (int index = 0; index < num; index++) {
    perception::fusion::PointD p1, p2;
    p1 = ptlist[index];
    p2 = ptlist[(index + 1) % num];
    x1 = p1.x;
    x2 = p2.x;
    y1 = p1.y;
    y2 = p2.y;
    if (y1 == y2) {
      continue;
    }
    double temp = y1 < y2 ? y1 : y2;
    if (y < temp) {
      continue;
    }
    temp = y1 > y2 ? y1 : y2;
    if (y >= temp) {
      continue;
    }
    double val = (double)(y - y1) * (double)(x2 - x1) / (double)(y2 - y1) + x1;
    if (val > x) {
      nCross++;
    }
  }
  return (nCross) % 2 == 1 ? 1 : 0;
}


}
}