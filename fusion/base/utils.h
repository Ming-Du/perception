
#include"point_cloud.h"

namespace perception {
namespace fusion {
/*
* @func:Determine whether a point is inside an outline
* @param1:point.x
* @param2:point.y
* @param3:points of outline
* @return:if the point in outline return 1,else return 0
*/
int pointInRegion(double x, double y, perception::fusion::PointCloud<perception::fusion::PointD> ptlist);

}
}