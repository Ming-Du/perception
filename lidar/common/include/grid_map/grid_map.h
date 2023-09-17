#pragma once 

#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <unordered_set>

#include "basic_type/object_type.h"
#include "basic_type/range.h"
#include "config/rs_yamlReader.h"

namespace robosense {

typedef struct CellInfo {
    CellInfo() {
        cell_idx_ = -1;
        cell_idy_ = -1;
        cell_id_ = -1;
        local_x_ = 0;
        local_y_ = 0;

        noise_cell_idx_ = -1;
        noise_cell_idy_ = -1;
        noise_cell_id_ = -1;

        init();
    };

    inline void init() {
        cell_type_ = -1;

        global_x_ = 0;
        global_y_ = 0;

        lowest_indices_ = -1;
        non_ground_lowest_indices_ = -1;
        highest_indices_ = -1;
        cart_lowest_value_ = 100;
        cart_non_ground_lowest_value_ = 100;
        cart_highest_value_ = -100;
        cart_ground_height_ = 0;
        cart_inner_cell_height_diff_ = 0;
        cart_adjacent_cell_lowest_diff_ = 0;
        cart_valid_flag_ = false;
        cart_ground_flag_ = false;
        cart_roi_flag_ = false;
        cart_bin_flag_ = false;
        obj_label_id_ = -1;

        noise_type_ = NoiseType::IGNORE;

        points_indices_.clear();
        points_indices_.reserve(5000);

        ground_indices_.clear();
        ground_indices_.reserve(5000);

        non_ground_indices_.clear();
        non_ground_indices_.reserve(5000);                
    }

    // base info
    int cell_idx_; // row
    int cell_idy_; // col
    int cell_id_;  // cell index
    int cell_type_;

    int noise_cell_idx_;
    int noise_cell_idy_;
    int noise_cell_id_;

    double local_x_;
    double local_y_;
    double global_x_;
    double global_y_;

    int lowest_indices_;
    int non_ground_lowest_indices_;
    int highest_indices_;
    float cart_lowest_value_;
    float cart_non_ground_lowest_value_;
    float cart_highest_value_;
    float cart_ground_height_;
    float cart_adjacent_cell_lowest_diff_;
    float cart_inner_cell_height_diff_;

    bool cart_valid_flag_;

    bool cart_roi_flag_;
    bool cart_bin_flag_;
    bool cart_ground_flag_;
    std::vector<int> points_indices_;
    std::vector<int> ground_indices_;
    std::vector<int> non_ground_indices_;

    // 0:ground 1:road 2:flowerbeds 3:fence 10:roadside 11:flowerbedsside 12:fenceside
    RoadType road_type;

    NoiseType noise_type_;

    int obj_label_id_;

} CellInfo;

typedef std::shared_ptr<CellInfo> CellInfoPtr;
typedef std::shared_ptr<const CellInfo> CellInfoConstPtr;

typedef std::vector<CellInfoPtr> CellMap;
typedef std::shared_ptr<CellMap> CellMapPtr;

class GridMap {
public:
    using Ptr = std::shared_ptr<GridMap>;

    GridMap();

    inline GridMap &operator=(const GridMap &other) {
        if (this != &other) {
            grid_params_ = other.grid_params_;
            grid_rows = other.grid_rows;
            grid_cols = other.grid_cols;
            cell_size = other.cell_size;
            cart_grid_count = other.cart_grid_count;

            pt_cell_id_vec.clear();
            pt_cell_id_vec.reserve(cart_grid_count);
            roi_cell_id_vec.clear();
            roi_cell_id_vec.reserve(cart_grid_count);
            bin_cell_id_vec.clear();
            bin_cell_id_vec.reserve(cart_grid_count);

            cell_map_ptr_->clear();
            cell_map_ptr_->resize(cart_grid_count);
            int thread_pool_count_ = 5;
            std::vector<std::thread> threads(thread_pool_count_);
            for (int i = 0; i < thread_pool_count_; ++i) {
                int steps = other.cell_map_ptr_->size() / thread_pool_count_;
                auto start_pos = other.cell_map_ptr_->begin() + i * steps;
                auto end_pos = (i == thread_pool_count_ - 1) ? other.cell_map_ptr_->end() : other.cell_map_ptr_->begin() + (i + 1) * steps;
                threads[i] = std::thread([this, start_pos, end_pos]() {
                    for (auto it = start_pos; it != end_pos; it++){
                        if(*it == nullptr) continue;
                        CellInfoPtr cell_info_ptr = std::make_shared<CellInfo>(**it);
                        this->cell_map_ptr_->at(cell_info_ptr->cell_id_) = std::move(cell_info_ptr);
                    }
                });
            }
            for (auto &t : threads) {
                t.join();
            }
        }
        return (*this);
    }

    void init(const Rs_YAMLReader &configParse);
    void initParams();

    int calCellId(const double &pt_x, const double &pt_y, bool update_cell_info = false);
    bool calPxPyFromCellId(int cell_id, double &pt_center_x, double &pt_center_y);

    size_t getCellCount();
    CellMapPtr getCellMapPtr();
    CellInfoPtr getCellInfo(const int &index, bool is_check = true);

    GridMapParam getGridMapParam();

    int grid_rows;
    int grid_cols;
    int cart_grid_count;
    double cell_size;

    std::vector<bool> valid_cell_id_vec;
    
    std::vector<int> pt_cell_id_vec;
    std::vector<int> roi_cell_id_vec;
    std::vector<int> bin_cell_id_vec;

private:
    void setParams();
    bool isValid(const int &index);
    void createCellMap(const int &cell_idx);

    bool isInDenoiseRange(const double px, const double py);

private:
    std::mutex mutex_;
    CellMapPtr cell_map_ptr_;

    GridMapParam grid_params_;
    DenoiseParam denoise_params_;

};

typedef std::shared_ptr<GridMap> GridMapPtr;
typedef std::shared_ptr<const GridMap> GridMapConstPtr;

} // namespace robosense
