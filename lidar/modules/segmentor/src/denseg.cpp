#include "denseg.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <math.h>
#include <omp.h>

namespace robosense {

bool operator<(const Edge &a, const Edge &b) { return a.w < b.w; }

bool DenseSeg::isboundary(int& row, int& col) {
    if((col < cols_ - ygrid_thre_) && (row < rows_ - xgrid_thre_)) {
        return false;
    }
    else {
        return true;
    }
}

void DenseSeg::init(int xgrid_thre, int ygrid_thre, double init_thre, double unit_size, int cols, int rows) {
    double dist, dx, dy;
    std::vector<double> tmp(ygrid_thre + 1);

    xgrid_thre_ = xgrid_thre;
    ygrid_thre_ = ygrid_thre;
    unit_size_ = unit_size;
    init_thre_ = init_thre;
    rows_ = rows;
    cols_ = cols;
    edges_.reserve(rows * cols * xgrid_thre_);
    thresholds_.reserve(kMaxVerticesNum);
    thresholds_table_.resize(kMaxThresholdsNum);
    for (auto x = 0; x <= xgrid_thre_; x++) {
        tmp.clear();
        for (auto y = 0; y <= ygrid_thre_; y++) {
            dx = x * unit_size_;
            dy = y * unit_size_;
            dist = sqrt(dx * dx + dy * dy);
            tmp.emplace_back(dist);
        }
        edge_weights_.emplace_back(tmp);
    }
    for (size_t i = 1; i < kMaxThresholdsNum; i++) {
        thresholds_table_[i] = init_thre_ / pow(i, 0.55);
    }
}

int DenseSeg::run(const GridMapPtr &grid_map_ptr) {
    edges_.clear();
    int x, y, vertices_id;
    int cur_x, cur_y, cur_idx;
    std::vector<int> id_map;
    CellInfoPtr cell_info_ptr;

    id_map.resize(rows_*cols_, -1);
    vertices_id = 0;
    for (auto &cell_id : grid_map_ptr->bin_cell_id_vec) {
        cell_info_ptr = grid_map_ptr->getCellInfo(cell_id);
        x = cell_info_ptr->cell_idx_;
        y = cell_info_ptr->cell_idy_;
        if (isboundary(x, y)) {
            continue;
        }
        id_map[cell_id] = vertices_id;
        vertices_id++;
    }

    for (auto &cell_id : grid_map_ptr->bin_cell_id_vec) {
        if(id_map[cell_id] == -1) {
            continue;
        }
        cell_info_ptr = grid_map_ptr->getCellInfo(cell_id);
        x = cell_info_ptr->cell_idx_;
        y = cell_info_ptr->cell_idy_;
        for (auto cur_x = x; cur_x <= x + xgrid_thre_; ++cur_x) {
            for (auto cur_y = y; cur_y <= y + ygrid_thre_; ++cur_y) {
                cur_idx = cur_x * cols_ + cur_y;
                if(id_map[cur_idx] == -1) {
                    continue;
                }
                if (cur_idx <= cell_id) {
                    continue;
                }
                Edge edge;
                edge.a = id_map[cell_id];
                edge.b = id_map[cur_idx];
                edge.w = edge_weights_[cur_x - x][cur_y - y];
                edges_.emplace_back(edge);
            }
        }
    }

    segmentgraph(vertices_id);
    extract_result(grid_map_ptr, id_map);

    return universe_.GetSetsNum();
}

void DenseSeg::segmentgraph(int num_vertices) {
    std::sort(edges_.begin(), edges_.end());
    universe_.Reset(num_vertices);
    thresholds_.assign(num_vertices, init_thre_);

    for (auto &edge : edges_) {
        int a = universe_.Find(edge.a);
        int b = universe_.Find(edge.b);
        if (a == b) {
            continue;
        }
        // TODO ?
        //  if (edge.w > thrs) {
        //      edge.uodate_weight
        //  }
        if (edge.w <= thresholds_[a] && edge.w <= thresholds_[b]) {
            universe_.Join(a, b);
            a = universe_.Find(a);
            unsigned int size_a = static_cast<unsigned int>(universe_.GetSize(a));
            thresholds_[a] = edge.w + thresholds_table_[size_a];
        }
    }
}

void DenseSeg::extract_result(const GridMapPtr &grid_map_ptr, std::vector<int>& id_map) {
    CellInfoPtr cell_info_ptr;
    
    for (auto &cell_id : grid_map_ptr->bin_cell_id_vec) {
        if(id_map[cell_id] == -1) {
            continue;
        }
        cell_info_ptr = grid_map_ptr->getCellInfo(cell_id);
        cell_info_ptr->obj_label_id_ = universe_.Find(id_map[cell_id]);
    }
}

}