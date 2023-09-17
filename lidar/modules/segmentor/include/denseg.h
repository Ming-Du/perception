#pragma once

#include "disjoint_set.h"
#include <memory>

namespace robosense {

struct Edge {
    int a;
    int b;
    double w;
};

class DenseSeg {
    public:
    // struct Edge {
    //     int a;
    //     int b;
    //     double w;
    // };
    
    void init(int xgrid_thre, int ygrid_thre, double init_thre, double unit_size, int cols, int rows);
    int run(const GridMapPtr &grid_map_ptr);
    void segmentgraph(int num_vertices);
    void extract_result(const GridMapPtr &grid_map_ptr, std::vector<int>& id_map);
    bool isboundary(int& row, int& col);

    private:
    static const size_t kMaxThresholdsNum = 50000;
    static const size_t kMaxVerticesNum = 10000;
    std::vector<std::vector<double>> edge_weights_;
    std::vector<double> thresholds_table_;
    std::vector<double> thresholds_;
    std::vector<Edge> edges_;
    int xgrid_thre_;
    int ygrid_thre_;
    int rows_;
    int cols_;
    double unit_size_;
    double init_thre_;
    Universe universe_;
};

}