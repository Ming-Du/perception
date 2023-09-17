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
#ifndef RS_COMMON_BASIC_TYPE_ROTATE_BOX_H_
#define RS_COMMON_BASIC_TYPE_ROTATE_BOX_H_

#include "rs_util.h"
#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <cmath>
#include <vector>

namespace robosense{

struct RotateBox {
    using Ptr = std::shared_ptr<RotateBox>;

    RotateBox(){
        heading = Eigen::Vector3d(0.,0.,0.);
    }

    RotateBox(const Eigen::Vector3d &center, const Eigen::Vector3d &size, const Eigen::Vector3d &dir){
        this->center = center;
        this->size = size;
        this->heading = dir;
        this->heading.normalize();
        this->angle = std::atan2(this->heading.y(),this->heading.x());
    }

    RotateBox(const Eigen::Vector3d &center, const Eigen::Vector3d &size, const double &ang){
        this->center = center;
        this->size = size;
        this->angle = normalizeAngle(ang);
        this->heading[0] = std::cos(this->angle);
        this->heading[1] = std::sin(this->angle);
        this->heading[2] = 0;
    }

    double area() const{
        return this->size.x() * this->size.y();
    }

    double volume() const{
        return this->size.x() * this->size.y() * this->size.z();
    }

    // get 8 corners from a box
    void corners(std::vector<Eigen::Vector3d> &corners) const{
        corners.resize(8, Eigen::Vector3d(0,0,0));

        auto dir = this->heading;
        dir.z() = 0.f;
        dir.normalize();
        Eigen::Vector3d ortho_dir = Eigen::Vector3d(-dir.y(), dir.x(), 0);
        Eigen::Vector3d z_dir = Eigen::Vector3d(0, 0, 1);

        const auto& center = this->center;
        const auto& size = this->size;

        corners[0] = center + dir * size.x() * 0.5f + ortho_dir * size.y() * 0.5f - z_dir * size.z() * 0.5f;
        corners[1] = center - dir * size.x() * 0.5f + ortho_dir * size.y() * 0.5f - z_dir * size.z() * 0.5f;
        corners[2] = center - dir * size.x() * 0.5f - ortho_dir * size.y() * 0.5f - z_dir * size.z() * 0.5f;
        corners[3] = center + dir * size.x() * 0.5f - ortho_dir * size.y() * 0.5f - z_dir * size.z() * 0.5f;

        corners[4] = center + dir * size.x() * 0.5f + ortho_dir * size.y() * 0.5f + z_dir * size.z() * 0.5f;
        corners[5] = center - dir * size.x() * 0.5f + ortho_dir * size.y() * 0.5f + z_dir * size.z() * 0.5f;
        corners[6] = center - dir * size.x() * 0.5f - ortho_dir * size.y() * 0.5f + z_dir * size.z() * 0.5f;
        corners[7] = center + dir * size.x() * 0.5f - ortho_dir * size.y() * 0.5f + z_dir * size.z() * 0.5f;
    }

    // get the nearest corner from a box
    Eigen::Vector3d anchor() const{
        std::vector<Eigen::Vector3d> corners;
        this->corners(corners);

        double min_dist = 1e9;
        int idx = -1;
        for (int i = 0; i < 4; ++i) {
            double dist = corners[i].norm();
            if (dist < min_dist) {
                min_dist = dist;
                idx = i;
            }
        }

        return corners[idx];
    }

    double normalizeAngle(const double &a) {
        double b = a;
        while (b < -M_PI) {
            b += 2 * M_PI;
        }
        while (b > M_PI) {
            b -= 2 * M_PI;
        }
        return b;
    }

//    bool isInside(const Eigen::Vector3d point) const{
//        std::vector<Eigen::Vector3d> corners;
//        this->corners(corners);
//
//        double vertx[4], verty[4];
//        for (int k = 0; k < 4; ++k) {
//            vertx[k] = corners[k].x();
//            verty[k] = corners[k].y();
//        }
//
//        if (inPolygon(4, vertx, verty, point.x(), point.y()) && point.z() > corners[0].z() &&
//            point.z() < corners[6].z()) {
//            return true;
//        }
//
//        return false;
//    }

    Eigen::Vector3d size,center,heading;
    double angle = 0.f;
};

class RotateBoxWrapper {
public:
    using Ptr = std::shared_ptr<RotateBoxWrapper>;

    double calcuIntersectionArea(const RotateBox &box1, const RotateBox &box2);

private:
    int rotatedRectangleIntersection(const RotateBox &box1, const RotateBox &box2,
                                     std::vector<Eigen::Vector2d> &intersection_region);

    std::vector<Eigen::Vector2d> getPoints(const RotateBox &box);

    double contourArea(std::vector<Eigen::Vector2d> contour);
};

}  // namespace robosense

#endif  // RS_COMMON_BASIC_TYPE_ROTATE_BOX_H_
