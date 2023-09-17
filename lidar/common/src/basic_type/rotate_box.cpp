/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
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
#include "basic_type/rotate_box.h"

namespace robosense{

double RotateBoxWrapper::calcuIntersectionArea(const RotateBox &box1, const RotateBox &box2) {
    std::vector<Eigen::Vector2d> inter;
    int res = rotatedRectangleIntersection(box1, box2, inter);
    if (inter.empty() || res == 0)
        return 0.0f;
    if (res == 2)
        return std::min(box1.area(), box2.area());
    return contourArea(inter);
}

int RotateBoxWrapper::rotatedRectangleIntersection(const RotateBox &box1, const RotateBox &box2,
                                                   std::vector<Eigen::Vector2d> &intersection_region) {
    double area1 = box1.size.x() * box1.size.y();
    double area2 = box2.size.x() * box2.size.y();
    // L2 metric
    const double samePointEps = std::max(1e-16d, 1e-6d * std::max(area1, area2));

    std::vector<Eigen::Vector2d> vec1(4), vec2(4);
    std::vector<Eigen::Vector2d> pts1(4), pts2(4);

    std::vector<Eigen::Vector2d> intersection;
    intersection.reserve(24);

    pts1 = getPoints(box1);
    pts2 = getPoints(box2);

    int ret = 2;

    // Specical case of rect1 == rect2
    {
        bool same = true;

        for (int i = 0; i < 4; i++) {
            if (fabs(pts1[i](0) - pts2[i](0)) > samePointEps || (fabs(pts1[i](1) - pts2[i](1)) > samePointEps)) {
                same = false;
                break;
            }
        }

        if (same) {
            intersection.resize(4);

            for (int i = 0; i < 4; i++) {
                intersection[i] = pts1[i];
            }

            intersection_region = intersection;

            return 2;
        }
    }

    // Line vector
    // A line from p1 to p2 is: p1 + (p2-p1)*t, t=[0,1]
    for (int i = 0; i < 4; i++) {
        vec1[i](0) = pts1[(i + 1) % 4](0) - pts1[i](0);
        vec1[i](1) = pts1[(i + 1) % 4](1) - pts1[i](1);

        vec2[i](0) = pts2[(i + 1) % 4](0) - pts2[i](0);
        vec2[i](1) = pts2[(i + 1) % 4](1) - pts2[i](1);
    }

    // Line test - test all line combos for intersection
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            // Solve for 2x2 Ax=b
            double x21 = pts2[j](0) - pts1[i](0);
            double y21 = pts2[j](1) - pts1[i](1);

            double vx1 = vec1[i](0);
            double vy1 = vec1[i](1);

            double vx2 = vec2[j](0);
            double vy2 = vec2[j](1);

            double det = vx2 * vy1 - vx1 * vy2;

            double t1 = (vx2 * y21 - vy2 * x21) / det;
            double t2 = (vx1 * y21 - vy1 * x21) / det;

            // This takes care of parallel lines
            if (std::isinf(t1) || std::isinf(t2) || std::isnan(t1) || std::isnan(t2)) {
                continue;
            }

            if (t1 >= 0.0f && t1 <= 1.0f && t2 >= 0.0f && t2 <= 1.0f) {
                double xi = pts1[i](0) + vec1[i](0) * t1;
                double yi = pts1[i](1) + vec1[i](1) * t1;

                intersection.emplace_back(Eigen::Vector2d(xi, yi));
            }
        }
    }

    if (!intersection.empty()) {
        ret = 1;
    }

    // Check for vertices from rect1 inside recct2
    for (int i = 0; i < 4; i++) {
        // We do a sign test to see which side the point lies.
        // If the point all lie on the same sign for all 4 sides of the rect,
        // then there's an intersection
        int posSign = 0;
        int negSign = 0;

        double x = pts1[i](0);
        double y = pts1[i](1);

        for (int j = 0; j < 4; j++) {
            // line equation: Ax + By + C = 0
            // see which side of the line this point is at
            double A = -vec2[j](1);
            double B = vec2[j](0);
            double C = -(A * pts2[j](0) + B * pts2[j](1));

            double s = A * x + B * y + C;

            if (s >= 0) {
                posSign++;
            } else {
                negSign++;
            }
        }

        if (posSign == 4 || negSign == 4) {
            intersection.push_back(pts1[i]);
        }
    }

    // Reverse the check - check for vertices from rect2 inside recct1
    for (int i = 0; i < 4; i++) {
        // We do a sign test to see which side the point lies.
        // If the point all lie on the same sign for all 4 sides of the rect,
        // then there's an intersection
        int posSign = 0;
        int negSign = 0;

        double x = pts2[i](0);
        double y = pts2[i](1);

        for (int j = 0; j < 4; j++) {
            // line equation: Ax + By + C = 0
            // see which side of the line this point is at
            double A = -vec1[j](1);
            double B = vec1[j](0);
            double C = -(A * pts1[j](0) + B * pts1[j](1));

            double s = A * x + B * y + C;

            if (s >= 0) {
                posSign++;
            } else {
                negSign++;
            }
        }

        if (posSign == 4 || negSign == 4) {
            intersection.push_back(pts2[i]);
        }
    }

    int N = (int) intersection.size();
    if (N == 0) {
        return 0;
    }

    // Get rid of duplicated points
    int Nstride = N;
    std::shared_ptr<double> distPt;
    distPt.reset(new double[N * N]);
    std::shared_ptr<int> ptDistRemap;
    ptDistRemap.reset(new int[N]);
    //            cv::AutoBuffer<double, 100> distPt(N * N);
    //            cv::AutoBuffer<int> ptDistRemap(N);
    for (int i = 0; i < N; ++i) {
        const Eigen::Vector2d pt0 = intersection[i];
        ptDistRemap.get()[i] = i;
        for (int j = i + 1; j < N;) {
            const Eigen::Vector2d pt1 = intersection[j];
            //                    double d2 = normL2Sqr<double>(pt1 - pt0);
            Eigen::Vector2d tmp_pt = pt1 - pt0;
            double d2 = tmp_pt(0) * tmp_pt(0) + tmp_pt(1) * tmp_pt(1);
            if (d2 <= samePointEps) {
                if (j < N - 1)
                    intersection[j] = intersection[N - 1];
                N--;
                continue;
            }
            distPt.get()[i * Nstride + j] = d2;
            ++j;
        }
    }
    while (N > 8) {// we still have duplicate points after samePointEps threshold (eliminate closest points)
        int minI = 0;
        int minJ = 1;
        double minD = distPt.get()[1];
        for (int i = 0; i < N - 1; ++i) {
            double *pDist = distPt.get() + Nstride * ptDistRemap.get()[i];
            for (int j = i + 1; j < N; ++j) {
                double d = pDist[ptDistRemap.get()[j]];
                if (d < minD) {
                    minD = d;
                    minI = i;
                    minJ = j;
                }
            }
        }
        Eigen::Vector2d tmp_pt = intersection[minI] - intersection[minJ];
        assert(
        fabs((tmp_pt(0) * tmp_pt(0) + tmp_pt(1) * tmp_pt(1)) - minD) < 1e-6); // ptDistRemap is not corrupted
        // drop minJ point
        if (minJ < N - 1) {
            intersection[minJ] = intersection[N - 1];
            ptDistRemap.get()[minJ] = ptDistRemap.get()[N - 1];
        }
        N--;
    }

    // order points
    for (int i = 0; i < N - 1; ++i) {
        Eigen::Vector2d diffI = intersection[i + 1] - intersection[i];
        for (int j = i + 2; j < N; ++j) {
            Eigen::Vector2d diffJ = intersection[j] - intersection[i];
            if ((diffI(0) * diffJ(1) - diffI(1) * diffJ(0)) < 0) {
                std::swap(intersection[i + 1], intersection[j]);
                diffI = diffJ;
            }
        }
    }

    intersection.resize(N);
    intersection_region = intersection;

    return ret;
}

std::vector<Eigen::Vector2d> RotateBoxWrapper::getPoints(const RotateBox &box) {
    double angle = box.angle * 180. / M_PI;
    double angle_ = angle * 3.1415926535897932384626433832795 / 180.;
    double b = (double) cos(angle_) * 0.5f;
    double a = (double) sin(angle_) * 0.5f;

    std::vector<Eigen::Vector2d> pts(4);

    pts[0].x() = box.center.x() - a * box.size.y() - b * box.size.x();
    pts[0].y() = box.center.y() + b * box.size.y() - a * box.size.x();
    pts[1].x() = box.center.x() + a * box.size.y() - b * box.size.x();
    pts[1].y() = box.center.y() - b * box.size.y() - a * box.size.x();
    pts[2].x() = 2 * box.center.x() - pts[0].x();
    pts[2].y() = 2 * box.center.y() - pts[0].y();
    pts[3].x() = 2 * box.center.x() - pts[1].x();
    pts[3].y() = 2 * box.center.y() - pts[1].y();
    return pts;
}

double RotateBoxWrapper::contourArea(std::vector<Eigen::Vector2d> contour) {
    int npoints = static_cast<int>(contour.size());

    if (npoints == 0)
        return 0.;

    double a00 = 0;
    Eigen::Vector2d prev = contour[npoints - 1];

    for (int i = 0; i < npoints; i++) {
        Eigen::Vector2d p = contour[i];
        a00 += prev(0) * p(1) - prev(1) * p(0);
        prev = p;
    }

    a00 *= 0.5;
    a00 = fabs(a00);

    return a00;
}

}
