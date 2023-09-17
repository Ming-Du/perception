#include "bev_builder.h"

namespace robosense {

BevBuilder::BevBuilder()
{
    grid_map_ptr_ = std::make_shared<GridMap>();
}

BevBuilder::~BevBuilder() {

}

void BevBuilder::init(const Rs_YAMLReader &configParse){
    grid_map_ptr_->init(configParse);
}

void BevBuilder::perception(const LidarFrameMsg::Ptr &msg_ptr) {
    const auto &cloud_ptr = msg_ptr->scan_ptr;
    const auto &valid_indices = msg_ptr->valid_indices;
    
    TRY_CATCH
    {
        grid_map_ptr_->initParams();
        CellMapPtr cell_map_ptr = grid_map_ptr_->getCellMapPtr();
        for (auto &vi : valid_indices) {
            PointT &pt = cloud_ptr->points.at(vi);
            int cell_id = grid_map_ptr_->calCellId(pt.x, pt.y);
            if (cell_id < 0) continue;

            CellInfoPtr cell_info_ptr = cell_map_ptr->at(cell_id); 
            if(cell_info_ptr == nullptr) continue;

            if(!grid_map_ptr_->valid_cell_id_vec.at(cell_id)){
                grid_map_ptr_->valid_cell_id_vec.at(cell_id) = true;
                grid_map_ptr_->pt_cell_id_vec.emplace_back(cell_id);
                
                cell_info_ptr->init();
                cell_info_ptr->cart_valid_flag_ = true;
            }
            cell_info_ptr->points_indices_.emplace_back(vi);
        }
        msg_ptr->grid_map_ptr = grid_map_ptr_;

    }
    END_TRY_CATCH
}

} // namespace robosense