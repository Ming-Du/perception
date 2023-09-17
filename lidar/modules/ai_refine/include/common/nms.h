/******************************************************************************
 * Copyright 2021 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
// modify from https://github.com/open-mmlab/OpenPCDet/tree/master/pcdet/ops/iou3d_nms/src/iou3d_cpu.cpp
/*
3D Rotated IoU Calculation (CPU)
Written by Shaoshuai Shi
All Rights Reserved 2020.
*/

#pragma once


#include <algorithm>
#include <cmath>
#include <stdio.h>
#include <vector>
namespace robosense {

struct Bndbox {
    float x;
    float y;
    float z;
    float w;
    float l;
    float h;
    float rt;
    int id;
    float score;
    Bndbox(){};
    Bndbox(float x_, float y_, float z_, float w_, float l_, float h_, float rt_, int id_, float score_)
        : x(x_), y(y_), z(z_), w(w_), l(l_), h(h_), rt(rt_), id(id_), score(score_) {}
};

struct Point2D {
    float x, y;
    Point2D() {}
    Point2D(double _x, double _y) { x = _x, y = _y; }

    void set(float _x, float _y) {
        x = _x;
        y = _y;
    }

    Point2D operator+(const Point2D &b) const { return Point2D(x + b.x, y + b.y); }

    Point2D operator-(const Point2D &b) const { return Point2D(x - b.x, y - b.y); }
};

const float EPS = 1e-8;

inline float cross(const Point2D &a, const Point2D &b) { return a.x * b.y - a.y * b.x; }

inline float cross(const Point2D &p1, const Point2D &p2, const Point2D &p0) {
    return (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y);
}

inline float min(float a, float b) { return a > b ? b : a; }

inline float max(float a, float b) { return a > b ? a : b; }

inline int check_rect_cross(const Point2D &p1, const Point2D &p2, const Point2D &q1, const Point2D &q2) {
    int ret = min(p1.x, p2.x) <= max(q1.x, q2.x) && min(q1.x, q2.x) <= max(p1.x, p2.x) &&
              min(p1.y, p2.y) <= max(q1.y, q2.y) && min(q1.y, q2.y) <= max(p1.y, p2.y);
    return ret;
}

inline int check_in_box2d(const Bndbox &box, const Point2D &p) {
    const float MARGIN = 1e-2;

    float center_x = box.x, center_y = box.y;
    float angle_cos = cos(-box.rt), angle_sin = sin(-box.rt); // rotate the point in the opposite direction of box
    float rot_x = (p.x - center_x) * angle_cos + (p.y - center_y) * (-angle_sin);
    float rot_y = (p.x - center_x) * angle_sin + (p.y - center_y) * angle_cos;

    return (fabs(rot_x) < box.w / 2 + MARGIN && fabs(rot_y) < box.l / 2 + MARGIN);
}

inline int intersection(const Point2D &p1, const Point2D &p0, const Point2D &q1, const Point2D &q0, Point2D &ans) {
    // fast exclusion
    if (check_rect_cross(p0, p1, q0, q1) == 0)
        return 0;

    // check cross standing
    float s1 = cross(q0, p1, p0);
    float s2 = cross(p1, q1, p0);
    float s3 = cross(p0, q1, q0);
    float s4 = cross(q1, p1, q0);

    if (!(s1 * s2 > 0 && s3 * s4 > 0))
        return 0;

    // calculate intersection of two lines
    float s5 = cross(q1, p1, p0);
    if (fabs(s5 - s1) > EPS) {
        ans.x = (s5 * q0.x - s1 * q1.x) / (s5 - s1);
        ans.y = (s5 * q0.y - s1 * q1.y) / (s5 - s1);

    } else {
        float a0 = p0.y - p1.y, b0 = p1.x - p0.x, c0 = p0.x * p1.y - p1.x * p0.y;
        float a1 = q0.y - q1.y, b1 = q1.x - q0.x, c1 = q0.x * q1.y - q1.x * q0.y;
        float D = a0 * b1 - a1 * b0;

        ans.x = (b0 * c1 - b1 * c0) / D;
        ans.y = (a1 * c0 - a0 * c1) / D;
    }

    return 1;
}

inline void rotate_around_center(const Point2D &center, const float angle_cos, const float angle_sin, Point2D &p) {
    float new_x = (p.x - center.x) * angle_cos + (p.y - center.y) * (-angle_sin) + center.x;
    float new_y = (p.x - center.x) * angle_sin + (p.y - center.y) * angle_cos + center.y;
    p.set(new_x, new_y);
}

inline int point_cmp(const Point2D &a, const Point2D &b, const Point2D &center) {
    return atan2(a.y - center.y, a.x - center.x) > atan2(b.y - center.y, b.x - center.x);
}

inline float box_overlap(const Bndbox &box_a, const Bndbox &box_b) {
    float a_angle = box_a.rt, b_angle = box_b.rt;
    float a_dx_half = box_a.w / 2, b_dx_half = box_b.w / 2, a_dy_half = box_a.l / 2, b_dy_half = box_b.l / 2;
    float a_x1 = box_a.x - a_dx_half, a_y1 = box_a.y - a_dy_half;
    float a_x2 = box_a.x + a_dx_half, a_y2 = box_a.y + a_dy_half;
    float b_x1 = box_b.x - b_dx_half, b_y1 = box_b.y - b_dy_half;
    float b_x2 = box_b.x + b_dx_half, b_y2 = box_b.y + b_dy_half;

    Point2D center_a(box_a.x, box_a.y);
    Point2D center_b(box_b.x, box_b.y);

    Point2D box_a_corners[5];
    box_a_corners[0].set(a_x1, a_y1);
    box_a_corners[1].set(a_x2, a_y1);
    box_a_corners[2].set(a_x2, a_y2);
    box_a_corners[3].set(a_x1, a_y2);

    Point2D box_b_corners[5];
    box_b_corners[0].set(b_x1, b_y1);
    box_b_corners[1].set(b_x2, b_y1);
    box_b_corners[2].set(b_x2, b_y2);
    box_b_corners[3].set(b_x1, b_y2);

    // get oriented corners
    float a_angle_cos = cos(a_angle), a_angle_sin = sin(a_angle);
    float b_angle_cos = cos(b_angle), b_angle_sin = sin(b_angle);

    for (int k = 0; k < 4; k++) {
        rotate_around_center(center_a, a_angle_cos, a_angle_sin, box_a_corners[k]);
        rotate_around_center(center_b, b_angle_cos, b_angle_sin, box_b_corners[k]);
    }

    box_a_corners[4] = box_a_corners[0];
    box_b_corners[4] = box_b_corners[0];

    // get intersection of lines
    Point2D cross_points[16];
    Point2D poly_center;
    int cnt = 0, flag = 0;

    poly_center.set(0, 0);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            flag = intersection(box_a_corners[i + 1], box_a_corners[i], box_b_corners[j + 1], box_b_corners[j],
                                cross_points[cnt]);
            if (flag) {
                poly_center = poly_center + cross_points[cnt];
                cnt++;
            }
        }
    }

    // check corners
    for (int k = 0; k < 4; k++) {
        if (check_in_box2d(box_a, box_b_corners[k])) {
            poly_center = poly_center + box_b_corners[k];
            cross_points[cnt] = box_b_corners[k];
            cnt++;
        }
        if (check_in_box2d(box_b, box_a_corners[k])) {
            poly_center = poly_center + box_a_corners[k];
            cross_points[cnt] = box_a_corners[k];
            cnt++;
        }
    }

    poly_center.x /= cnt;
    poly_center.y /= cnt;

    // sort the points of polygon
    Point2D temp;
    for (int j = 0; j < cnt - 1; j++) {
        for (int i = 0; i < cnt - j - 1; i++) {
            if (point_cmp(cross_points[i], cross_points[i + 1], poly_center)) {
                temp = cross_points[i];
                cross_points[i] = cross_points[i + 1];
                cross_points[i + 1] = temp;
            }
        }
    }

    // get the overlap areas
    float area = 0;
    for (int k = 0; k < cnt - 1; k++) {
        area += cross(cross_points[k] - cross_points[0], cross_points[k + 1] - cross_points[0]);
    }

    return fabs(area) / 2.0;
}

inline float iou_bev(const Bndbox &box_a, const Bndbox &box_b) {
    float sa = box_a.w * box_a.l;
    float sb = box_b.w * box_b.l;
    float s_overlap = box_overlap(box_a, box_b);
    return s_overlap / fmaxf(sa + sb - s_overlap, EPS);
}

inline std::vector<float> iou_ior_bev(const Bndbox &box_a, const Bndbox &box_b) {
    std::vector<float> res(3, 0);
    float sa = box_a.w * box_a.l;
    float sb = box_b.w * box_b.l;
    float s_overlap = box_overlap(box_a, box_b);
    float iou = s_overlap / fmaxf(sa + sb - s_overlap, EPS);
    float ior1 = s_overlap / sa;
    float ior2 = s_overlap / sb;
    res[0] = iou; res[1] = ior1; res[2] = ior2;
    return res;
}

inline std::vector<Bndbox> nms_cpu(std::vector<Bndbox> &bndboxes, const float &nms_thresh, const float &ior_thresh) {
    std::sort(bndboxes.begin(), bndboxes.end(),
              [](Bndbox boxes1, Bndbox boxes2) { return boxes1.score > boxes2.score; });
    std::vector<int> suppressed(bndboxes.size(), 0);
    std::vector<Bndbox> nms_pred;
    for (size_t i = 0; i < bndboxes.size(); i++) {
        if (suppressed[i] == 1) {
            continue;
        }
        nms_pred.emplace_back(bndboxes[i]);
        for (size_t j = i + 1; j < bndboxes.size(); j++) {
            if (suppressed[j] == 1) {
                continue;
            }
            std::vector<float> iouiors = iou_ior_bev(bndboxes[i], bndboxes[j]);            
            if (iouiors[0] >= nms_thresh) {
                suppressed[j] = 1;
            }
            else if (iouiors[1] >= ior_thresh || iouiors[2] >= ior_thresh) {
                suppressed[j] = 1;
            }
        }
    }
    return nms_pred;
}

}  // namespace robosense
// #endif
