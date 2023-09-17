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
#include "basic_type/object.h"
namespace robosense {
void Object::transform(const Eigen::Matrix4d &trans_mat) {
    { // polygon
        auto &tmp = this->polygons;
        for (auto &tt : tmp) {
            Eigen::Vector4d tt_e;
            tt_e << tt.x(), tt.y(), tt.z(), 1;
            tt_e = trans_mat * tt_e;
            tt.x() = tt_e.x();
            tt.y() = tt_e.y();
            tt.z() = tt_e.z();
        }
    }
    { // corner_point
        auto &tmp = this->corner_point;
        for (int i = 0; i < tmp.rows(); i++) {
            auto &tt_x = tmp(i, 0);
            auto &tt_y = tmp(i, 1);
            Eigen::Vector4d tt_e;
            tt_e << tt_x, tt_y, 1, 1;
            tt_e = trans_mat * tt_e;
            tt_x = tt_e.x();
            tt_y = tt_e.y();
        }
    }
    { // center
        Eigen::Vector4d tt_e;
        auto &tmp = this->center;
        tt_e << tmp.x(), tmp.y(), tmp.z(), 1;
        tt_e = trans_mat * tt_e;
        tmp.x() = tt_e.x();
        tmp.y() = tt_e.y();
        tmp.z() = tt_e.z();
    }
    { // direction
        Eigen::Vector4d tt_e;
        auto &tmp = this->direction;
        tt_e << tmp.x(), tmp.y(), tmp.z(), 0;
        tt_e = trans_mat * tt_e;
        tmp.x() = tt_e.x();
        tmp.y() = tt_e.y();
        tmp.z() = tt_e.z();

        // heading
        this->heading = std::atan2(this->direction(1), this->direction(0));
    }
    { // velocity
        Eigen::Vector4d tt_e;
        auto &tmp = this->velocity;
        tt_e << tmp.x(), tmp.y(), tmp.z(), 0;
        tt_e = trans_mat * tt_e;
        tmp.x() = tt_e.x();
        tmp.y() = tt_e.y();
        tmp.z() = tt_e.z();
    }
}

} // namespace robosense
