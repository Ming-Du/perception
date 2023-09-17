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

#include "build_object/simple_2d_grid.h"
#include <pcl/common/common.h>
#include <numeric>
namespace robosense {


Simple2DGrid::Simple2DGrid(const Simple2DGridOptions& options){
    setOptions(options);
}

void Simple2DGrid::setOptions(const Simple2DGridOptions& options){
    options_ = options;
    if (options_.leaf.x() < 0 || options_.leaf.y() < 0){

    }
    leaf_inverse_.x() = 1. / options_.leaf.x();
    leaf_inverse_.y() = 1. / options_.leaf.y();
}

void Simple2DGrid::filter(const PointCloud::Ptr& cloud_ptr,std::vector<int>& voxel_indice){
    std::vector<int> pc_indices;
    std::iota(pc_indices.begin(),pc_indices.end(),0);
    filter(cloud_ptr,pc_indices,voxel_indice,0);
}

void Simple2DGrid::filter(const PointCloud::Ptr& cloud_ptr,const std::vector<int>& pc_indices,std::vector<int>& voxel_indice,bool is_debugobj){
    if (cloud_ptr == nullptr){

    }
    Eigen::Vector4f min_p, max_p;
    getMinMax3D(*(cloud_ptr),pc_indices,min_p,max_p);
    // Check that the leaf size is not too small, given the size of the data
    int64_t dx = static_cast<int64_t>((max_p[0] - min_p[0]) * leaf_inverse_.x())+1;
    int64_t dy = static_cast<int64_t>((max_p[1] - min_p[1]) * leaf_inverse_.y())+1;

    if ((dx*dy) > static_cast<int64_t>(std::numeric_limits<int32_t>::max())) {

        voxel_indice = pc_indices;
        return;
    }
    // Compute the minimum and maximum bounding box values
    min_b_[0] = static_cast<int> (floor (min_p[0] * leaf_inverse_.x()));
    max_b_[0] = static_cast<int> (floor (max_p[0] * leaf_inverse_.x()));
    min_b_[1] = static_cast<int> (floor (min_p[1] * leaf_inverse_.y()));
    max_b_[1] = static_cast<int> (floor (max_p[1] * leaf_inverse_.y()));

    // Compute the number of divisions needed along all axis
    div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones ();
    div_b_[3] = 0;

    // Set up the division multiplier
    divb_mul_ = Eigen::Vector4i (1, div_b_[0], 0, 0);
    std::vector<bool> flag(div_b_[0] * div_b_[1],false);
    voxel_indice.clear();
    voxel_indice.reserve(pc_indices.size());
    for (auto it = pc_indices.begin (); it != pc_indices.end (); ++it) {
        // Check if the point is invalid
        if (!std::isfinite (cloud_ptr->points[*it].x) ||
            !std::isfinite (cloud_ptr->points[*it].y) ||
            !std::isfinite (cloud_ptr->points[*it].z)){
            continue;
        }

        int ijk0 = static_cast<int> (floor (cloud_ptr->points[*it].x * leaf_inverse_.x()) - static_cast<double> (min_b_[0]));
        int ijk1 = static_cast<int> (floor (cloud_ptr->points[*it].y * leaf_inverse_.y()) - static_cast<double> (min_b_[1]));

        // Compute the centroid leaf index
        int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1];
        if (!flag[idx]){
            flag[idx] = true;
            voxel_indice.emplace_back(*it);
        }
    }
    voxel_indice.resize(voxel_indice.size());
}

}
