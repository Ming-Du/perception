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
#include <ros/ros.h>

#include "common/include/build_object/object_build.h"
#include "common/include/rs_util.h"
#include "common/include/tic_toc.h"
#include "rs_segmentor.h"

#include <omp.h>
#include <queue>
#include <stack>
#include <string>
#include <unordered_map>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#define OPENCV_TRAITS_ENABLE_DEPRECATED

using namespace cv;

namespace robosense {

void RsSegmentor::init(const Rs_YAMLReader &configParse) {
    params = configParse.getValue<SegmentorParam>("segmentor");
}

void GetMinMaxGrid(const LidarFrameMsg::Ptr &msg_ptr, const std::vector<int> &cell_indices,
                 std::pair<int, int> &min_cell, std::pair<int, int> &max_cell) {
    CellInfoPtr cell_info_ptr;
    min_cell.first = min_cell.second = INT_MAX;
    max_cell.first = max_cell.second = INT_MIN;
    for (auto idx : cell_indices) {
        cell_info_ptr = msg_ptr->grid_map_ptr->getCellInfo(idx);
        min_cell.first = std::min(min_cell.first, cell_info_ptr->cell_idx_);
        max_cell.first = std::max(max_cell.first, cell_info_ptr->cell_idx_);
        min_cell.second = std::min(min_cell.second, cell_info_ptr->cell_idy_);
        max_cell.second = std::max(max_cell.second, cell_info_ptr->cell_idy_);
    }
}

int RsSegmentor::split_large_obj(const LidarFrameMsg::Ptr &msg_ptr, std::vector<Object::Ptr> &obj_vec, int start_id) {
    int split_thre, label_ori;
    std::vector<int> erase_object;
    CellInfoPtr cell_info_ptr;
    std::vector<Object::Ptr> tmp_obj_vec = obj_vec;
    label_ori = start_id;
    for (size_t i = 0; i < tmp_obj_vec.size(); ++i) {
        std::pair<int, int> min_cell;
        std::pair<int, int> max_cell;
        auto &obj = tmp_obj_vec.at(i);
        int seg_size = static_cast<int>(obj->cell_indices.size());
        if (seg_size < 5) {
            continue;
        }
        if (obj->status != RoadType::ROADSIDE) {
            split_thre = params.split_thre_road;
        } else {
            split_thre = params.split_thre_roadside;
        }
        GetMinMaxGrid(msg_ptr, obj->cell_indices, min_cell, max_cell);
        if (max_cell.second - min_cell.second > split_thre || max_cell.first - min_cell.first > split_thre) {
            std::vector<Object::Ptr> new_objects;
            std::vector<int> label_map;
            std::vector<int> newobj_id;
            int rows = (max_cell.first - min_cell.first) / split_thre + 1;
            int cols = (max_cell.second - min_cell.second) / split_thre + 1;
            erase_object.emplace_back(i);
            new_objects.resize(rows * cols);
            label_map.resize(rows * cols);
            for (auto cell_id : obj->cell_indices) {
                cell_info_ptr = msg_ptr->grid_map_ptr->getCellInfo(cell_id);
                int x = (cell_info_ptr->cell_idx_ - min_cell.first) / split_thre;
                int y = (cell_info_ptr->cell_idy_ - min_cell.second) / split_thre;
                int index = x * cols + y;
                if (new_objects.at(index) == nullptr) {
                    new_objects.at(index).reset(new Object);
                    new_objects.at(index)->cloud_indices.reserve(obj->cloud_indices.size());
                    new_objects.at(index)->cell_indices.reserve(obj->cell_indices.size());
                    new_objects.at(index)->status = obj->status;
                    label_map.at(index) = label_ori;
                    newobj_id.emplace_back(index);
                    label_ori++;
                }
                new_objects.at(index)->cell_indices.emplace_back(cell_id);
                new_objects.at(index)->cloud_indices.insert(new_objects.at(index)->cloud_indices.end(),
                                                       cell_info_ptr->non_ground_indices_.begin(), 
                                                       cell_info_ptr->non_ground_indices_.end());
                cell_info_ptr->obj_label_id_ = label_map.at(index);
            }
            for (auto &id : newobj_id) {
                obj_vec.emplace_back(new_objects.at(id));
            }
        }
    }
    if (erase_object.size() != 0) {
        reverse(erase_object.begin(), erase_object.end());
        for (const auto &i : erase_object) {
            obj_vec.erase(obj_vec.begin() + i);
        }
    }

    return label_ori;
}

int RsSegmentor::split_floating_obj(const LidarFrameMsg::Ptr &msg_ptr, std::vector<Object::Ptr> &obj_vec, int start_id) {
    int split_thre, label_ori;
    std::vector<int> erase_object;
    CellInfoPtr cell_info_ptr;
    auto obj_num = obj_vec.size();
    label_ori = start_id;
    for (size_t i = 0; i < obj_num; ++i) {
        std::map<double, int, std::greater<double>> cell_map;
        std::vector<int> floating_cell;
        std::vector<int> unfloating_cell;
        auto &obj = obj_vec.at(i);
        if (obj->status != RoadType::ROADSIDE || obj->cell_indices.size() < 8
            || msg_ptr->scan_ptr->points[obj->cloud_indices[0]].y > 0) {
            continue;
        }
        for (auto idx : obj->cell_indices) {
            cell_info_ptr = msg_ptr->grid_map_ptr->getCellInfo(idx);
            cell_map[msg_ptr->scan_ptr->points[cell_info_ptr->points_indices_[0]].y] = idx;
        }
        for (std::map<double,int>::iterator iter1 = cell_map.begin(); iter1 != cell_map.end(); ++iter1) {
            cell_info_ptr = msg_ptr->grid_map_ptr->getCellInfo(iter1->second);
            if (msg_ptr->scan_ptr->points[cell_info_ptr->non_ground_lowest_indices_].z > params.enable_split_floating_) {
                floating_cell.emplace_back(iter1->second);
            }
            else {
                for (std::map<double,int>::iterator iter2 = iter1; iter2 != cell_map.end(); ++iter2) {
                    unfloating_cell.emplace_back(iter2->second);
                }
                break;
            }
        }
        if (floating_cell.size() < 2) {
            continue;
        }
        Object::Ptr floating_obj, unfloating_obj;
        floating_obj.reset(new Object);
        floating_obj->cloud_indices.reserve(obj->cloud_indices.size());
        floating_obj->cell_indices.reserve(floating_cell.size());
        floating_obj->status = obj->status;
        floating_obj->filter_type = FilterObjType::SUSTECTED_FLOATING;
        for (auto &idx : floating_cell) {
            cell_info_ptr = msg_ptr->grid_map_ptr->getCellInfo(idx);
            floating_obj->cell_indices.emplace_back(idx);
            floating_obj->cloud_indices.insert(floating_obj->cloud_indices.end(),
                                               cell_info_ptr->non_ground_indices_.begin(), 
                                               cell_info_ptr->non_ground_indices_.end());
            cell_info_ptr->obj_label_id_ = label_ori;
        }
        label_ori++;
        unfloating_obj.reset(new Object);
        unfloating_obj->cloud_indices.reserve(obj->cloud_indices.size());
        unfloating_obj->cell_indices.reserve(unfloating_cell.size());
        unfloating_obj->status = obj->status;
        for (auto &idx : unfloating_cell) {
            cell_info_ptr = msg_ptr->grid_map_ptr->getCellInfo(idx);
            unfloating_obj->cell_indices.emplace_back(idx);
            unfloating_obj->cloud_indices.insert(unfloating_obj->cloud_indices.end(),
                                               cell_info_ptr->non_ground_indices_.begin(), 
                                               cell_info_ptr->non_ground_indices_.end());
            cell_info_ptr->obj_label_id_ = label_ori;
        }
        label_ori++;
        erase_object.emplace_back(i);
        obj_vec.emplace_back(floating_obj);
        obj_vec.emplace_back(unfloating_obj);
    }
    if (erase_object.size() != 0) {
        reverse(erase_object.begin(), erase_object.end());
        for (const auto &i : erase_object) {
            obj_vec.erase(obj_vec.begin() + i);
        }
    }

    return label_ori;
}

void RsSegmentor::perception(const LidarFrameMsg::Ptr &msg_ptr, const localization::Localization &local_current) {
    if (!params.enable) {
        return;
    }
    const auto &cloud_ptr = msg_ptr->scan_ptr;
    auto &obj_vec = msg_ptr->objects_rule;
    auto &grid_map_ptr = msg_ptr->grid_map_ptr;
    int tot_obj_num = 0;

    msg_ptr->transAxis(AxisType::VEHICLE_AXIS);

    if(init_denseg) {
        denseseg_.init(params.xgrid_thre_, params.ygrid_thre_, params.init_thre_, 
                msg_ptr->grid_map_ptr->cell_size, msg_ptr->grid_map_ptr->grid_cols, msg_ptr->grid_map_ptr->grid_rows);
        init_denseg = false;
    }

    tot_obj_num = denseseg_.run(grid_map_ptr);

    std::vector<Object::Ptr> tmp_obj_vec(tot_obj_num);
    for (auto &obj : tmp_obj_vec) {
        obj.reset(new Object(grid_map_ptr->bin_cell_id_vec.size()));
    }

    // Insert grid and point into obj_vec
    std::unordered_map<int, std::vector<RoadType>> label_count;
    std::map<int, int> id_map;
    int tmp_id = 0;
    for (auto &cell_id : grid_map_ptr->bin_cell_id_vec) {
        // Get object info besed grid map of label number
        CellInfoPtr cell_info_ptr = msg_ptr->grid_map_ptr->getCellInfo(cell_id); 
        if(cell_info_ptr->obj_label_id_ == -1) {
            continue;
        }
        int label_id = cell_info_ptr->obj_label_id_;
        if (id_map.find(label_id)==id_map.end()) {
            id_map[label_id] = tmp_id;
            tmp_id++;
        }
        tmp_obj_vec.at(id_map[label_id])->cell_indices.emplace_back(cell_id);
        tmp_obj_vec.at(id_map[label_id])->cloud_indices.insert(tmp_obj_vec.at(id_map[label_id])->cloud_indices.end(),
                                                       cell_info_ptr->non_ground_indices_.begin(), 
                                                       cell_info_ptr->non_ground_indices_.end());
        if (cell_info_ptr->road_type > RoadType::ROAD)
            tmp_obj_vec.at(id_map[label_id])->status = RoadType::ROADSIDE;
    }
    
    if (id_map.size() > 0) {
        auto tmp = id_map.rbegin();
        int start_id = tmp->first + 1;
        // split large obj
        if (params.enable_split_) {
            start_id = split_large_obj(msg_ptr, tmp_obj_vec, start_id);
        }
        // split floating obj
        if (params.enable_split_floating_) {
            start_id = split_floating_obj(msg_ptr, tmp_obj_vec, start_id);
        }
    }

    RsObjectBuilderInitOptions build_option;
    RsObjectBuilder builder(build_option);
    obj_vec.reserve(tmp_obj_vec.size());
    for (int i = 0; i < tmp_obj_vec.size(); ++i) {
        auto &obj = tmp_obj_vec.at(i);
        int seg_size = static_cast<int>(obj->cloud_indices.size());
        if (seg_size < params.seg_min_pts) {
            continue;
        }
        bool is_debugobj = false;
        if (builder.buildObject(cloud_ptr, obj, is_debugobj)) {
            obj_vec.emplace_back(obj);
        }
    }
}

} // namespace robosense
