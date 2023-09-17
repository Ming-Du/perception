#ifndef BIRD_BIEW_SPACE_H
#define BIRD_BIEW_SPACE_H

#include <vector>
#include "basic_type/range.h"
#include <eigen3/Eigen/Dense>

namespace robosense
{
    struct BirdViewSpace
    {
        int rows, cols, grid_size;
        //栅格大小,反向计算更快
        double inverse_grid_size;
        Range3D range = Range3D(-100., 100., -100., 100., -1., 2.);
        std::vector<int> bev2pt_num_;
        std::vector<int> bev2_max_height_pt_idx_;
        std::vector<int> bev2_min_height_pt_idx_;
        std::vector<double> bev2_height_diff_;
        std::vector<Eigen::Vector2d> local_pos;
    };
}
#endif //BIRD_BIEW_SPACE_H
