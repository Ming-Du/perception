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

#ifndef RS_SDK_SIMPLE_2D_GRID_H
#define RS_SDK_SIMPLE_2D_GRID_H

#include "rs_define.h"
#include "msg/lidar_frame_msg.h"

#include <eigen3/Eigen/Dense>
namespace robosense {


struct Simple2DGridOptions {
    Eigen::Vector2d leaf = Eigen::Vector2d(0.2, 0.2);
};

//===================================
// 用于快速栅格滤波,区别于pcl的地方在于
// 1. 只做2D
// 2. 不取中心,只需要得到栅格中一个点即可
// 因此,只针对特定需求,按需使用
//===================================

class Simple2DGrid {
public:
    using Ptr = std::shared_ptr<Simple2DGrid>;

    explicit Simple2DGrid(const Simple2DGridOptions& options = Simple2DGridOptions());

    void filter(const PointCloud::Ptr& cloud_ptr, std::vector<int>& voxel_indice);
    void filter(const PointCloud::Ptr& cloud_ptr,
                const std::vector<int>& pc_indices, std::vector<int>& voxel_indice,bool is_debugobj);

    void setOptions(const Simple2DGridOptions& options);

private:
    std::string name() {
        return "Simple2DGrid";
    }
    Simple2DGridOptions options_;
    Eigen::Vector2d leaf_inverse_;
    Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;
};

}
#endif //RS_SDK_SIMPLE_2D_GRID_H
