#include "dev_rotate_iou.h"

namespace perception {
namespace mid_common {

DevRotateIou::DevRotateIou() {}
void DevRotateIou::demo() {
  float rbox1[5];
  float rbox2[5];

  rbox1[0] = 0.0;
  rbox1[1] = 10.0;
  rbox1[2] = 2.0;
  rbox1[3] = 4.0;
  rbox1[4] = 0.5 * M_PI - 0.5 * M_PI;

  rbox2[0] = 1.0;
  rbox2[1] = 9.0;
  rbox2[2] = 2.0;
  rbox2[3] = 4.0;
  rbox2[4] = 0.5 * M_PI - 0.5 * M_PI;
  float value = devRotateIoUEval(rbox1, rbox2);
  return;
}

float DevRotateIou::devRotateIoUEval(float rbox1[5], float rbox2[5]) {
  if (fabs(rbox1[0]) > 20 || fabs(rbox2[0]) > 20 || fabs(rbox1[1]) > 70 || fabs(rbox2[1]) > 70 ||
      rbox2[2] < 0.01 || rbox2[3] < 0.01 || rbox1[2] < 0.01 || rbox1[3] < 0.01) {
    return 0;
  }
  double area1 = rbox1[2] * rbox1[3];
  double area2 = rbox2[2] * rbox2[3];
  double area_inter = inter(rbox1, rbox2);
  double area_diff = (area1 + area2 - area_inter);
  //  if(area_diff<0.5*(area1+area2)){return 1;}
  float iou = area_inter / area_diff;
  return iou;
}
double DevRotateIou::inter(float rbbox1[5], float rbbox2[5]) {
  float corners1[8];
  float corners2[8];
  float intersection_corners[16];
  rbbox_to_corners(rbbox1, &corners1[0]);
  rbbox_to_corners(rbbox2, &corners2[0]);

  float dxy;
  int cnt = 0;
  for (int i = 0; i < 8; i++) {
    dxy = fabs(corners1[i] - corners2[i]);
    if (dxy <= 0.1) {
      cnt = cnt + 1;
    }
  }
  if (8 == cnt) {
    return rbbox1[2] * rbbox1[3];
  }

  int num_intersection = quadrilateral_intersection(corners1, corners2, &intersection_corners[0]);
  sort_vertex_in_convex_polygon(&intersection_corners[0], num_intersection);

  double area_value = area(intersection_corners, num_intersection);
  /*
  std::cout << "corners1::" << corners1[0] << "," << corners1[1] << "   " <<
  corners1[2] << "," << corners1[3] << "   "; std::cout << "corners1::" <<
  corners1[4] << "," << corners1[5] << "   " << corners1[6] << "," <<
  corners1[7] << std::endl;

  std::cout << "corners2::" << corners2[0] << "," << corners2[1] << "   " <<
  corners2[2] << "," << corners2[3] << "   "; std::cout << "corners2::" <<
  corners2[4] << "," << corners2[5] << "   " << corners2[6] << "," <<
  corners2[7] << std::endl; std::cout << "num_intersection::" <<
  num_intersection << std::endl;

  std::cout << "intersection_corners::"  << std::endl;
  std::cout << intersection_corners[0] << "," << intersection_corners[1] << "
  "; std::cout << intersection_corners[2] << "," << intersection_corners[3] <<
  "    "; std::cout << intersection_corners[4] << "," <<
  intersection_corners[5] << "    ";

  std::cout << intersection_corners[6] << "," << intersection_corners[7] << "
  "; std::cout << intersection_corners[8] << "," << intersection_corners[9] <<
  "    "; std::cout << intersection_corners[10] << "," <<
  intersection_corners[11] << "    "; std::cout << intersection_corners[12] <<
  "," << intersection_corners[13] << "    "; std::cout <<
  intersection_corners[14] << "," << intersection_corners[15] << std::endl;
  */

  return area_value;
}

void DevRotateIou::rbbox_to_corners(float rbbox[5], float* corners) {
  float angle = rbbox[4];
  float a_cos = cos(angle);
  float a_sin = sin(angle);
  float center_x = rbbox[0];
  float center_y = rbbox[1];
  float x_d = rbbox[2];
  float y_d = rbbox[3];
  float corners_x[4];
  float corners_y[4];
  corners_x[0] = -x_d / 2;
  corners_x[1] = -x_d / 2;
  corners_x[2] = x_d / 2;
  corners_x[3] = x_d / 2;
  corners_y[0] = -y_d / 2;
  corners_y[1] = y_d / 2;
  corners_y[2] = y_d / 2;
  corners_y[3] = -y_d / 2;
  for (int i = 0; i < 4; i++) {
    *(corners + 2 * i) = a_cos * corners_x[i] + a_sin * corners_y[i] + center_x;
    *(corners + 2 * i + 1) = -a_sin * corners_x[i] + a_cos * corners_y[i] + center_y;
  }
}

int DevRotateIou::quadrilateral_intersection(float pts1[8], float pts2[8], float* int_pts) {
  //  LOG_Info()<<"pts1="<<"("<<pts1[0]<<","<<pts1[1]<<")"<<", ("<<pts1[2]<<","<<pts1[3]<<")" \
  //               <<", ("<<pts1[4]<<","<<pts1[5]<<")"<<", ("<<pts1[6]<<","<<pts1[7]<<")";
  // LOG_Info()<<"pts2="<<"("<<pts2[0]<<","<<pts2[1]<<")"<<", ("<<pts2[2]<<","<<pts2[3]<<")" \
  //         <<", ("<<pts2[4]<<","<<pts2[5]<<")"<<", ("<<pts2[6]<<","<<pts2[7]<<")";

  int num_of_inter = 0;

  float temp_pts[2];
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      bool has_pts = line_segment_intersection(pts1, pts2, i, j, &temp_pts[0]);
      if (has_pts) {
        *(int_pts + num_of_inter * 2) = temp_pts[0];
        *(int_pts + num_of_inter * 2 + 1) = temp_pts[1];
        num_of_inter += 1;
        // LOG_Info() << "inter_point3::"<<
        // temp_pts[0]<<","<<temp_pts[1];
      }
    }
  }
  if (num_of_inter >= 8) {
    return num_of_inter;
  }
  float dx, dy;
  bool flag = false;
  for (int i = 0; i < 4; i++) {
    if (point_in_quadrilateral(pts1[2 * i], pts1[2 * i + 1], pts2)) {
      flag = false;  //判断点是否重复
      for (int k = 0; k < num_of_inter; k++) {
        dx = fabs(*(int_pts + k * 2) - pts1[2 * i]);
        dy = fabs(*(int_pts + k * 2 + 1) - pts1[2 * i + 1]);
        if ((dx <= 0.1) && (dy <= 0.1)) {
          flag = true;  //点重复
          break;
        }
      }
      if (false == flag) {
        *(int_pts + num_of_inter * 2) = pts1[2 * i];
        *(int_pts + num_of_inter * 2 + 1) = pts1[2 * i + 1];
        num_of_inter += 1;
        //  LOG_Info() << "inter_point1::"<<
        //  pts1[2*i]<<","<<pts1[2*i+1];
      }
    }
    if (num_of_inter >= 8) {
      break;
    }
    if (point_in_quadrilateral(pts2[2 * i], pts2[2 * i + 1], pts1)) {
      flag = false;  //判断点是否重复
      for (int k = 0; k < num_of_inter; k++) {
        dx = fabs(*(int_pts + k * 2) - pts2[2 * i]);
        dy = fabs(*(int_pts + k * 2 + 1) - pts2[2 * i + 1]);
        if ((dx <= 0.1) && (dy <= 0.1)) {
          flag = true;  //点重复
          break;
        }
      }
      if (false == flag) {
        *(int_pts + num_of_inter * 2) = pts2[2 * i];
        *(int_pts + num_of_inter * 2 + 1) = pts2[2 * i + 1];
        num_of_inter += 1;
        // LOG_Info() << "inter_point2::"<< pts2[2*i]<<","<<pts2[2*i+1];
      }
    }
    if (num_of_inter >= 8) {
      break;
    }
  }
  // LOG_Info()<<"All points num="<<num_of_inter;
  return num_of_inter;
}

bool DevRotateIou::point_in_quadrilateral(float pt_x, float pt_y, float corners[8]) {
  float ab0 = corners[2] - corners[0];
  float ab1 = corners[3] - corners[1];

  float ad0 = corners[6] - corners[0];
  float ad1 = corners[7] - corners[1];

  float ap0 = pt_x - corners[0];
  float ap1 = pt_y - corners[1];

  float abab = ab0 * ab0 + ab1 * ab1;
  float abap = ab0 * ap0 + ab1 * ap1;
  float adad = ad0 * ad0 + ad1 * ad1;
  float adap = ad0 * ap0 + ad1 * ap1;

  double eps = -1e-6;
  bool result = (abab - abap >= eps) && (abap >= eps) && (adad - adap >= eps) && (adap >= eps);
  return result;
}

bool DevRotateIou::line_segment_intersection(float pts1[8], float pts2[8], int i, int j,
                                             float* temp_pts) {
  /*float A[2],B[2],C[2],D[2];
  A[0] = pts1[2 * i];
  A[1] = pts1[2 * i + 1];

  B[0] = pts1[2 * ((i + 1) % 4)];
  B[1] = pts1[2 * ((i + 1) % 4) + 1];

  C[0] = pts2[2 * j];
  C[1] = pts2[2 * j + 1];

  D[0] = pts2[2 * ((j + 1) % 4)];
  D[1] = pts2[2 * ((j + 1) % 4) + 1];

  float BA0 = B[0] - A[0];
  float BA1 = B[1] - A[1];
  float DA0 = D[0] - A[0];
  float CA0 = C[0] - A[0];
  float DA1 = D[1] - A[1];
  float CA1 = C[1] - A[1];
  bool acd = DA1 * CA0 > CA1 * DA0;
  bool bcd = (D[1] - B[1]) * (C[0] - B[0]) > (C[1] - B[1]) * (D[0] - B[0]);
  if (acd != bcd)
  {
      bool abc = CA1 * BA0 > BA1 * CA0;
      bool abd = DA1 * BA0 > BA1 * DA0;
      if (abc != abd)
      {
          float DC0 = D[0] - C[0];
          float DC1 = D[1] - C[1];
          float ABBA = A[0] * B[1] - B[0] * A[1];
          float CDDC = C[0] * D[1] - D[0] * C[1];
          float DH = BA1 * DC0 - BA0 * DC1;
          if(fabs(DH)<=0.0000000001){return false;}//出现平行向量认为无交点
          float Dx = ABBA * DC0 - BA0 * CDDC;
          float Dy = ABBA * DC1 - BA1 * CDDC;
          *(temp_pts+0) = Dx / DH;
          *(temp_pts+1) = Dy / DH;
          return true;
      }
  }
  return false;*/

  float a[2], b[2], c[2], d[2];
  a[0] = pts1[2 * i];
  a[1] = pts1[2 * i + 1];

  b[0] = pts1[2 * ((i + 1) % 4)];
  b[1] = pts1[2 * ((i + 1) % 4) + 1];

  c[0] = pts2[2 * j];
  c[1] = pts2[2 * j + 1];

  d[0] = pts2[2 * ((j + 1) % 4)];
  d[1] = pts2[2 * ((j + 1) % 4) + 1];

  double area_abc, area_abd, area_cda, area_cdb, t, dx, dy;
  area_abc = trangle_area(a, b, c);
  area_abd = trangle_area(a, b, d);

  if (area_abc * area_abd >= 0) return false;

  area_cda = trangle_area(c, d, a);
  area_cdb = area_cda + area_abc - area_abd;

  if (area_cda * area_cdb >= 0) return false;
  t = area_cda / (area_abd - area_abc);

  dx = t * (b[0] - a[0]);
  dy = t * (b[1] - a[1]);

  temp_pts[0] = a[0] + dx;
  temp_pts[1] = a[1] + dy;

  return true;
}

void DevRotateIou::sort_vertex_in_convex_polygon(float* int_pts, int num_of_inter) {
  if (num_of_inter > 8) {
    num_of_inter = 8;
  }
  if (num_of_inter > 1) {
    // 先计算所有点的中心点
    float center[2];
    center[0] = 0.0;
    center[1] = 0.0;
    for (int i = 0; i < num_of_inter; i++) {
      center[0] += *(int_pts + 2 * i);
      center[1] += *(int_pts + 2 * i + 1);
    }
    center[0] /= num_of_inter;
    center[1] /= num_of_inter;
    // 所有点离中心点的坐标的归一化
    double v[2];
    float vs[16];
    for (int i = 0; i < num_of_inter; i++) {
      v[0] = *(int_pts + 2 * i) - center[0];
      v[1] = *(int_pts + 2 * i + 1) - center[1];
      double d = sqrt(v[0] * v[0] + v[1] * v[1]);
      if (d < 0.00000001) {
        v[0] = 0;
        v[1] = 0;
      } else {
        v[0] = v[0] / d;
        v[1] = v[1] / d;
      }
      if (v[1] < 0) v[0] = -2 - v[0];
      vs[i] = v[0];
    }
    // 排序
    int j = 0;
    float temp = 0;
    for (int i = 1; i < num_of_inter; i++) {
      if (vs[i - 1] > vs[i]) {
        temp = vs[i];
        float tx = *(int_pts + 2 * i);
        float ty = *(int_pts + 2 * i + 1);
        j = i;
        while ((j > 0) && (vs[j - 1] > temp)) {
          vs[j] = vs[j - 1];
          *(int_pts + j * 2) = *(int_pts + j * 2 - 2);
          *(int_pts + j * 2 + 1) = *(int_pts + j * 2 - 1);
          j -= 1;
        }
        vs[j] = temp;
        *(int_pts + 2 * j) = tx;
        *(int_pts + 2 * j + 1) = ty;
      }
    }
  }
}
float DevRotateIou::area(float int_pts[16], int num_of_inter) {
  float area_val = 0.0;
  if (num_of_inter > 8) {
    num_of_inter = 8;
  }
  for (int i = 0; i < (num_of_inter - 2); i++) {
    float temp1[2];
    temp1[0] = int_pts[0];  // int_pts[:2]
    temp1[1] = int_pts[1];
    float temp2[2];
    temp2[0] = int_pts[2 * i + 2];  // int_pts[2 * i + 2:2 * i + 4]
    temp2[1] = int_pts[2 * i + 3];
    float temp3[2];
    temp3[0] = int_pts[2 * i + 4];  // int_pts[2 * i + 4:2 * i + 6]
    temp3[1] = int_pts[2 * i + 5];
    area_val += abs(trangle_area(temp1, temp2, temp3));
  }
  return area_val;
}
float DevRotateIou::trangle_area(float a[2], float b[2], float c[2]) {
  float value = ((a[0] - c[0]) * (b[1] - c[1]) - (a[1] - c[1]) * (b[0] - c[0])) / 2.0;
  return value;
}

}  // namespace mid_common
}  // namespace perception
