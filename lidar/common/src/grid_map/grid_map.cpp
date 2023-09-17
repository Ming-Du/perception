#include "grid_map/grid_map.h"

namespace robosense {

GridMap::GridMap()
    : grid_rows(0)
    , grid_cols(0)
    , cart_grid_count(0)
{
    cell_map_ptr_ = std::make_shared<CellMap>();
}

void GridMap::init(const Rs_YAMLReader &configParse) {
    grid_params_ = configParse.getValue<GridMapParam>("roi_filter");
    denoise_params_ = configParse.getValue<DenoiseParam>("denoise");

    setParams();
    initParams();

    cell_size = grid_params_.unit_size_;
    cell_map_ptr_->resize(cart_grid_count);

    for (size_t id = 0; id < cart_grid_count; id++) {
        createCellMap(id);
    }
}

void GridMap::setParams() {
    std::lock_guard<std::mutex> lock(mutex_);
    grid_rows = static_cast<int>((grid_params_.range_.xmax - grid_params_.range_.xmin) / grid_params_.unit_size_) + 1;
    grid_cols = static_cast<int>((grid_params_.range_.ymax - grid_params_.range_.ymin) / grid_params_.unit_size_) + 1;
    cart_grid_count = grid_cols * grid_rows;
}

void GridMap::initParams(){
    std::lock_guard<std::mutex> lock(mutex_);
    valid_cell_id_vec.clear();
    valid_cell_id_vec.assign(cart_grid_count, false);

    pt_cell_id_vec.clear();
    pt_cell_id_vec.reserve(cart_grid_count);
    roi_cell_id_vec.clear();
    roi_cell_id_vec.reserve(cart_grid_count);
    bin_cell_id_vec.clear();
    bin_cell_id_vec.reserve(cart_grid_count);
}

void GridMap::createCellMap(const int &cell_idx) {
    std::lock_guard<std::mutex> lock(mutex_);
    int x_id = cell_idx / grid_cols;
    int y_id = cell_idx % grid_cols;
    double local_x = (x_id + 0.5) * grid_params_.unit_size_ + grid_params_.range_.xmin;
    double local_y = (y_id + 0.5) * grid_params_.unit_size_ + grid_params_.range_.ymin;

    if (cell_map_ptr_->at(cell_idx) == nullptr)
        cell_map_ptr_->at(cell_idx) = std::make_shared<CellInfo>();

    cell_map_ptr_->at(cell_idx)->cell_idx_ = x_id;
    cell_map_ptr_->at(cell_idx)->cell_idy_ = y_id;
    cell_map_ptr_->at(cell_idx)->cell_id_ = cell_idx;
    cell_map_ptr_->at(cell_idx)->local_x_ = local_x;
    cell_map_ptr_->at(cell_idx)->local_y_ = local_y;

    if(isInDenoiseRange(local_x, local_y)){
        int noise_x_id = static_cast<int>((local_x - denoise_params_.range.xmin) / denoise_params_.unit_size);
        int noise_y_id = static_cast<int>((local_y - denoise_params_.range.ymin) / denoise_params_.unit_size);
        int noise_cell_id = noise_x_id * denoise_params_.ysize + noise_y_id;
        cell_map_ptr_->at(cell_idx)->noise_cell_idx_ = noise_x_id;
        cell_map_ptr_->at(cell_idx)->noise_cell_idy_ = noise_y_id;
        cell_map_ptr_->at(cell_idx)->noise_cell_id_ = noise_cell_id;
    }
}

int GridMap::calCellId(const double &pt_x, const double &pt_y, bool update_cell_info) {
    std::lock_guard<std::mutex> lock(mutex_);
    int x_id = static_cast<int>((pt_x - grid_params_.range_.xmin) / grid_params_.unit_size_);
    int y_id = static_cast<int>((pt_y - grid_params_.range_.ymin) / grid_params_.unit_size_);
    int cell_idx = x_id * grid_cols + y_id;
    if (!isValid(cell_idx)) {
        if (!update_cell_info)
            return -1;

        CellInfoPtr cell_info_ptr = std::make_shared<CellInfo>();
        cell_info_ptr->cell_idx_ = x_id;
        cell_info_ptr->cell_idy_ = y_id;
        cell_info_ptr->cell_id_ = cell_idx;
        cell_info_ptr->local_x_ = pt_x;
        cell_info_ptr->local_y_ = pt_y;

        cell_map_ptr_->at(cell_idx) = std::move(cell_info_ptr);
    }
    return std::move(cell_idx);
}

bool GridMap::calPxPyFromCellId(int cell_id, double &pt_center_x, double &pt_center_y) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!isValid(cell_id)) {
        pt_center_x = 0;
        pt_center_y = 0;
        return false;
    }
    int x_id = static_cast<int>(cell_id / grid_cols);
    int y_id = cell_id - x_id * grid_cols;
    pt_center_x = (x_id + 0.5) * grid_params_.unit_size_ + grid_params_.range_.xmin;
    pt_center_y = (y_id + 0.5) * grid_params_.unit_size_ + grid_params_.range_.ymin;
    return true;
}

bool GridMap::isValid(const int &index) {
    if (index < 0) 
        return false;
    else if (index > cart_grid_count - 1)
        return false;  
    else if (cell_map_ptr_->size() < 1)
        return false;
    else if(cell_map_ptr_->at(index) == nullptr)
        return false;
    else
        return true;
}

size_t GridMap::getCellCount() {
    std::lock_guard<std::mutex> lock(mutex_);
    return cell_map_ptr_->size();
}

CellInfoPtr GridMap::getCellInfo(const int &index, bool is_check) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_check && !isValid(index))
        return nullptr;
    return cell_map_ptr_->at(index);
}

CellMapPtr GridMap::getCellMapPtr() {
    std::lock_guard<std::mutex> lock(mutex_);
    return cell_map_ptr_;
}

GridMapParam GridMap::getGridMapParam(){
    std::lock_guard<std::mutex> lock(mutex_);
    return grid_params_;
}

bool GridMap::isInDenoiseRange(const double px, const double py) {
    if (px <= denoise_params_.range.xmin || px >= denoise_params_.range.xmax 
        || py <= denoise_params_.range.ymin || py >= denoise_params_.range.ymax) {
        return false;
    } else {
        return true;
    }
}

} // namespace robosense