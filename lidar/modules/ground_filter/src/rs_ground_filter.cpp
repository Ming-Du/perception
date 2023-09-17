#include "ros/ros.h"
#include "rs_ground_filter.h"
#include "common/include/rs_util.h"
#include "common/include/tic_toc.h"
#include "std_msgs/String.h"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <chrono>
#include <omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

namespace robosense {

RsGroundFilter::RsGroundFilter()
{

}

void RsGroundFilter::init(const Rs_YAMLReader &configParse) {
    params = configParse.getValue<GroundFilterParam>("ground_filter");
}

double RsGroundFilter::calcAngleErr2D(const double &a, const double &b) {
    double e = normalizeAngle(a) + normalizeAngle(b);
    return normalizeAngle(e);
}

void RsGroundFilter::perception(const LidarFrameMsg::Ptr &msg_ptr) {
    if (!params.enable) {
        return;
    }
    const auto &cloud_ptr = msg_ptr->scan_ptr;
    if (cloud_ptr->empty()) {
        return;
    }

    msg_ptr->transAxis(AxisType::VEHICLE_AXIS);

    linefit(msg_ptr);
}


void  RsGroundFilter::GroundMatch(std::vector<GroundCoeff> semantic_map, const GroundCoeff  leishen_semantic_map, std::vector<GroundCoeff>  &matched_semantic_map){
    double theta_vec[semantic_map.size()];
    for(int i = 0; i < semantic_map.size(); i++ ){
        // 10-20m match
        //vector_angle
        Eigen::Vector3f normal_a = semantic_map[i].ground_normal;
        Eigen::Vector4f coe_a = semantic_map[i].ground_coe;
        Eigen::Vector3f normal_b;
        Eigen::Vector4f coe_b;
        if(i ==0 ){
            normal_b = leishen_semantic_map.ground_normal;
            coe_b = leishen_semantic_map.ground_coe;
        }
        else{
            normal_b = semantic_map[i-1].ground_normal;
            coe_b = semantic_map[i-1].ground_coe;
        }
        float c_theta = normal_a.dot(normal_b) / ( normal_a.norm() *  normal_b.norm());
        float theta = acos(c_theta); // angle in radians
        if( theta > RS_M_PI / 2){
            theta = RS_M_PI - theta;
        }
        if(c_theta >= 1){
            theta = 0;
        } //advoid theta = nan , c_theta !> 1
        theta_vec[i] = theta;
        //plane_angle_diff   20-60.m
        if(i > 0){
            theta = theta -  theta_vec[0];
        }
        //rotate_plane
        float sin_theta = sin(theta);
        float cos_theta = cos(theta);
        float a_new = coe_a[0] * cos_theta * cos_theta + coe_a[1] * sin_theta * sin_theta - coe_a[2] * sin(2 * theta) + coe_a[1] * sin_theta * sin_theta + coe_a[0] * cos_theta * cos_theta + coe_a[2] * sin(2 * theta);
        float b_new = coe_a[0] * sin_theta * sin_theta + coe_a[1] * cos_theta * cos_theta + coe_a[2] * sin(2 * theta) + coe_a[1] * cos_theta * cos_theta + coe_a[0] * sin_theta * sin_theta - coe_a[2] * sin(2 * theta);
        float c_new = 2 * coe_a[0] * coe_a[1] * cos_theta * sin_theta + coe_a[2] * (cos_theta * cos_theta - sin_theta * sin_theta) + 2 * coe_a[1] * coe_a[2] * cos_theta * sin_theta;
        float d_new = coe_a[3];       
        GroundCoeff tmp_info;
        tmp_info.ground_coe << a_new, b_new, c_new, d_new;
        matched_semantic_map.push_back(tmp_info);
        
    }

}



void RsGroundFilter::RegionGroundFitting(const std::vector<std::vector<pcl::PointXYZ>> &ring_point, std::vector<GroundCoeff> &semantic_map) 
{
    if(ring_point[0].size() < 10){
        GroundCoeff tmp_info;
        tmp_info.start_ring = -1; //no fitting result
        semantic_map.push_back(tmp_info);
        return;
    }
    std::vector<int> empty_ring;    
    for(int i = 0;  i < ring_point.size() ; i ++){
        auto pointclound = ring_point[i];
        Eigen::MatrixX3f eigen_ground(pointclound.size(), 3);
        int j=0;
        for (auto &p: pointclound) {
            eigen_ground.row(j++) << p.x, p.y, p.z;
        }
        Eigen::MatrixX3f centered = eigen_ground.rowwise() - eigen_ground.colwise().mean();
        Eigen::MatrixX3f cov = (centered.adjoint() * centered) / double(eigen_ground.rows() - 1);
        
        Eigen::VectorXf pc_mean_;
        pc_mean_.resize(3);
        pc_mean_ << eigen_ground.colwise().mean()(0), eigen_ground.colwise().mean()(1), eigen_ground.colwise().mean()(2);

        Eigen::JacobiSVD<Eigen::MatrixX3f> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
        Eigen::VectorXf singular_values_ = svd.singularValues();
        
        // use the least singular vector as normal
        Eigen::VectorXf normal_ = (svd.matrixU().col(2));
        if (normal_(2) < 0) { for(int i=0; i<3; i++) normal_(i) *= -1; }
        // mean ground value
        Eigen::Vector3f seeds_mean = pc_mean_.head<3>();

        //float a_ = svd.matrixV()(0,2);
        //float b_ = svd.matrixV()(1,2);
        //float c_ = svd.matrixV()(2,2);
        // according to normal.T*[x,y,z] = -d
        float d_ = -(normal_.transpose() * seeds_mean)(0, 0);
        GroundCoeff tmp_info;
        tmp_info.start_ring = 10 + 10*i;
        tmp_info.end_ring = 20 + 10*i; 
        if(pointclound.size() < 10){ //no fitting result
            tmp_info.start_ring = -1;
            empty_ring.push_back(i);
        }
        tmp_info.ground_coe << normal_(0), normal_(1), normal_(2), d_;
        tmp_info.ground_normal << normal_(0), normal_(1), normal_(2);
        semantic_map.push_back(tmp_info);
    }

    for(auto index : empty_ring){
        if( index > 0 &&semantic_map[index-1].start_ring != -1){ // use front fround
            semantic_map[index] = semantic_map[index-1];
            continue;
        }
        if(index < (ring_point.size() -1 ) && semantic_map[index+1].start_ring != -1)
            semantic_map[index] = semantic_map[index+1];
        
    }
}


void RsGroundFilter::linefit(const LidarFrameMsg::Ptr &msg_ptr) {
    const auto &cloud_ptr = msg_ptr->scan_ptr;
    const auto &roifiltered_indices = msg_ptr->roifiltered_indices;
    auto &grid_map_ptr = msg_ptr->grid_map_ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_fitting_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    ground_fitting_cloud->reserve(roifiltered_indices.size());
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_fitting_close(new pcl::PointCloud<pcl::PointXYZ>());
    ground_fitting_close->reserve(roifiltered_indices.size());
    ring_point.clear();
    ring_point.resize(params.cell_distance_ranges.size());
    //std::vector<double> ring_point_sum(params.cell_distance_ranges.size(),0.0);
    for (auto &cell_id : grid_map_ptr->roi_cell_id_vec) {
        calAroundGridsHeightDiff(grid_map_ptr, cell_id);

        double tHDiff = 0, tInnerDiff = 0;
        CellInfoPtr cell_info_ptr = msg_ptr->grid_map_ptr->getCellInfo(cell_id);
        double distance = sqrtf(pow(cell_info_ptr->local_x_, 2) + pow(cell_info_ptr->local_y_, 2));
        int ring_index;
        for (int i = params.cell_distance_ranges.size() - 1; i >= 0; i--) {
            if (distance > params.cell_distance_ranges.at(i) ) {
                tHDiff = params.adjacent_cell_diff_ranges.at(i);
                tInnerDiff = params.inner_cell_diff_ranges.at(i);
                ring_index = i;
                break;
            }
        }
        if (cell_info_ptr->cart_adjacent_cell_lowest_diff_ < tHDiff 
            && cell_info_ptr->cart_inner_cell_height_diff_ < tInnerDiff) {
            pcl::PointXYZ w;
            w.x = cell_info_ptr->local_x_;
            w.y = cell_info_ptr->local_y_;
            w.z = cell_info_ptr->cart_lowest_value_;
            //ring_point_sum[ring_index] += w.z;
            //if( ring_point[ring_index].size() > 0 && (w.z > ring_point_sum[ring_index] / ring_point[ring_index].size())){
            //    continue;
            //}
            ring_point[ring_index].push_back(w);
            if(msg_ptr->is_semantic_map && cell_info_ptr->local_x_ < 20 && cell_info_ptr->local_x_ > -5){
                ground_fitting_close->push_back(w);
            }
            ground_fitting_cloud->push_back(w);
            msg_ptr->pointcloud_map.emplace_back(w.x, w.y,w.z);
            cell_info_ptr->cart_ground_flag_ = true;
            cell_info_ptr->cart_ground_height_ = cell_info_ptr->cart_lowest_value_;
        }
    }

    // ground fitting
    Eigen::VectorXf coefficient = Eigen::Vector3f::Zero();
    Eigen::VectorXf coefficient_falcon = Eigen::Vector3f::Zero();
    bool enable_ground_fitting = (ground_fitting_cloud->size() > 3) ? true : false;
    if (enable_ground_fitting && msg_ptr->hackarea_status != HackAreaType::ENABLE_UPHILL && !msg_ptr->is_pointcloud_map) {
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(ground_fitting_cloud));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);
        ransac.setDistanceThreshold(params.setDistanceThreshold);
        ransac.computeModel();
        ransac.getModelCoefficients(coefficient);
        Eigen::VectorXf coefficient_close = Eigen::Vector3f::Zero();
        if(ground_fitting_close->size() > 3 && msg_ptr->is_semantic_map){
            pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr close_ground(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(ground_fitting_close));
            pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_close(close_ground);
            ransac_close.setDistanceThreshold(params.setDistanceThreshold);
            ransac_close.computeModel();
            ransac_close.getModelCoefficients(coefficient_close);
            msg_ptr->leishen_semantic_map.ground_coe = coefficient_close;
            msg_ptr->leishen_semantic_map.ground_normal << coefficient_close(0), coefficient_close(1) , coefficient_close(2);
        }
        else{
            msg_ptr->is_semantic_map = false;
        }
    }
    if(msg_ptr->is_pointcloud_map){
        RegionGroundFitting(ring_point, msg_ptr->semantic_map);
        return;
    }
    
    if(enable_ground_fitting && msg_ptr->is_semantic_map && msg_ptr->hackarea_status != HackAreaType::ENABLE_UPHILL){
        if(msg_ptr->semantic_map.size() < params.cell_distance_ranges.size() || msg_ptr->semantic_map[0].start_ring == -1){
            msg_ptr->is_semantic_map = false;
        }
        else
            GroundMatch(msg_ptr->semantic_map, msg_ptr->leishen_semantic_map, msg_ptr->matched_semantic_map);
    }

    TRY_CATCH
    {
        for (auto &cell_id : grid_map_ptr->roi_cell_id_vec) {
            double cell_zmin = 0, cell_zmax = 0, ground_height = 0;
            CellInfoPtr cell_info_ptr = msg_ptr->grid_map_ptr->getCellInfo(cell_id);
            double distance = sqrtf(pow(cell_info_ptr->local_x_, 2) + pow(cell_info_ptr->local_y_, 2));
            for (int i = params.cell_distance_ranges.size() - 1; i >= 0; i--) {
                if (distance > params.cell_distance_ranges.at(i)) {
                    cell_zmin = params.cell_zmin_ranges.at(i);
                    cell_zmax = params.cell_zmax_ranges.at(i);
                    ground_height = params.ground_height_ranges.at(i);
                    break;
                }
            }
            if (enable_ground_fitting && coefficient(2) != 0 && msg_ptr->hackarea_status != HackAreaType::ENABLE_UPHILL) {
                double tmp_ground_h = -(coefficient(3) + cell_info_ptr->local_x_ * coefficient(0) + cell_info_ptr->local_y_ * coefficient(1)) / coefficient(2);
                if(cell_info_ptr->local_x_ > params.use_falcon_distance && msg_ptr->is_semantic_map){
                    int index = static_cast<int> (distance / 10.0);
                    coefficient_falcon = msg_ptr->matched_semantic_map[index].ground_coe;
                    tmp_ground_h = -(coefficient_falcon(3) + cell_info_ptr->local_x_ * coefficient_falcon(0) + cell_info_ptr->local_y_ * coefficient_falcon(1)) / coefficient_falcon(2);
                }
                if (cell_info_ptr->cart_ground_flag_  && (abs(tmp_ground_h - cell_info_ptr->cart_lowest_value_) < ground_height)) {
                    msg_ptr->pointcloud_map.emplace_back(cell_info_ptr->local_x_, cell_info_ptr->local_y_,tmp_ground_h);
                    cell_info_ptr->ground_indices_ = cell_info_ptr->points_indices_;
                } else {
                    msg_ptr->pointcloud_map.emplace_back(cell_info_ptr->local_x_, cell_info_ptr->local_y_,tmp_ground_h);
                    for (auto &indices : cell_info_ptr->points_indices_) {
                        
                        const auto &pt = cloud_ptr->points.at(indices);
                        if (((pt.z - tmp_ground_h) > cell_zmin) && ((pt.z - tmp_ground_h) < cell_zmax)) {
                            cell_info_ptr->non_ground_indices_.emplace_back(indices);
                            if (pt.z - cell_info_ptr->cart_non_ground_lowest_value_ < 0) {
                                cell_info_ptr->non_ground_lowest_indices_ = indices;
                                cell_info_ptr->cart_non_ground_lowest_value_ = pt.z;
                            }

                            if (!cell_info_ptr->cart_bin_flag_)
                                cell_info_ptr->cart_bin_flag_ = true;
                        }
                    }
                    cell_info_ptr->cart_ground_flag_ = false;
                    cell_info_ptr->cart_ground_height_ = tmp_ground_h;
                }
            } else {
                if (!cell_info_ptr->cart_ground_flag_){
                    for (auto &indices : cell_info_ptr->points_indices_) {
                        const auto &pt = cloud_ptr->points.at(indices);                  
                        if ((pt.z > cell_zmin) && (pt.z < cell_zmax)) {
                            cell_info_ptr->non_ground_indices_.emplace_back(indices);
                            if (pt.z - cell_info_ptr->cart_non_ground_lowest_value_ < 0) {
                                cell_info_ptr->non_ground_lowest_indices_ = indices;
                                cell_info_ptr->cart_non_ground_lowest_value_ = pt.z;
                            }

                            if (!cell_info_ptr->cart_bin_flag_)
                                cell_info_ptr->cart_bin_flag_ = true;
                        }
                    }
                }
            }

            if (cell_info_ptr->cart_bin_flag_) {
                msg_ptr->grid_map_ptr->bin_cell_id_vec.emplace_back(cell_id);
                msg_ptr->binmap.emplace_back(cell_info_ptr->local_x_, cell_info_ptr->local_y_);

                // set road type
                if (msg_ptr->is_roifilter_processed == true) {
                    int cell_type_ = cell_info_ptr->cell_type_;
                    if (cell_type_ == 0 || cell_type_ == 2 || cell_type_ == 3) {
                        continue;
                    }
                    switch (cell_type_) {
                    case 1:
                        cell_info_ptr->road_type = RoadType::ROAD;
                        break;
                    case 10:
                        cell_info_ptr->road_type = RoadType::ROADSIDE;
                        break;
                    case 11:
                        cell_info_ptr->road_type = RoadType::FLOWERBEDSSIDE;
                        break;
                    case 12:
                        cell_info_ptr->road_type = RoadType::FENCESIDE;
                        break;
                    }
                } else {
                    cell_info_ptr->road_type = RoadType::ROAD;
                }
            } else {
                msg_ptr->grid_map_ptr->valid_cell_id_vec.at(cell_id) = false;
            }
        }
    }
    END_TRY_CATCH
}

void RsGroundFilter::calAroundGridsHeightDiff(const GridMapPtr &grid_map_ptr, int grid_idx) {
    double adjacent_height_diff = DBL_MIN;
    auto cur_cell_info_ptr = grid_map_ptr->getCellInfo(grid_idx);
    int offset[8][2] = {{1, 1}, {1, 0}, {1, -1}, {0, 1}, {0, -1}, {-1, 1}, {-1, 0}, {-1, -1}};
    TRY_CATCH
    {
        for (auto num : offset) {
            if (grid_idx % grid_map_ptr->grid_cols == 0 && num[1] < 0) continue;
            if (grid_idx % grid_map_ptr->grid_cols == grid_map_ptr->grid_rows && num[1] > 0) continue;
            if (grid_idx / grid_map_ptr->grid_cols == 0 && num[0] < 0) continue;
            if (grid_idx / grid_map_ptr->grid_cols == grid_map_ptr->grid_cols && num[0] > 0) continue;
            int around_grid = grid_idx + num[1] + num[0] * grid_map_ptr->grid_cols;
            if(!grid_map_ptr->valid_cell_id_vec.at(around_grid)) continue;
            auto around_cell_info_ptr = grid_map_ptr->getCellInfo(around_grid);
            if (around_cell_info_ptr != nullptr && around_cell_info_ptr->cart_roi_flag_) {
                double adjacent_height_diff_temp = abs(cur_cell_info_ptr->cart_lowest_value_ - around_cell_info_ptr->cart_lowest_value_);
                if (adjacent_height_diff_temp > adjacent_height_diff)
                    adjacent_height_diff = adjacent_height_diff_temp;
            }
        }
        cur_cell_info_ptr->cart_adjacent_cell_lowest_diff_ = adjacent_height_diff;
        cur_cell_info_ptr->cart_inner_cell_height_diff_ = abs(cur_cell_info_ptr->cart_lowest_value_ - cur_cell_info_ptr->cart_highest_value_);
    }
    END_TRY_CATCH
}

} // namespace robosense
