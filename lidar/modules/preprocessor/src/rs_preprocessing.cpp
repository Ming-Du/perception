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
#include "rs_preprocessing.h"

namespace robosense {

void RsPreprocessing::init(const Rs_YAMLReader &configParse){
    params = configParse.getValue<PreprocessingParam>("preprocessing");
}

void RsPreprocessing::perception(const LidarFrameMsg::Ptr &msg_ptr) {
    const auto &scan_ptr = msg_ptr->scan_ptr;
    if (scan_ptr->empty()) {
        return;
    }

    msg_ptr->transAxis(AxisType::VEHICLE_AXIS);

    Filter(msg_ptr);
}

void RsPreprocessing::Filter(const LidarFrameMsg::Ptr &msg_ptr) {
    const auto &scan_ptr = msg_ptr->scan_ptr;
    auto &valid_indices = msg_ptr->valid_indices;
    auto& noise_indices = msg_ptr->noise_indices;
    valid_indices.reserve(scan_ptr->size());
    noise_indices.reserve(scan_ptr->size());
    for (size_t i = 0; i < scan_ptr->points.size(); ++i) {
        const auto &pt = scan_ptr->points[i];
        if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
            continue;
        }
        if (params.enable_vehicle_filter) {
            if (params.vehicle_filter.inRange(pt.x, pt.y, pt.z)) {
                continue;
            }
        }
        if (params.enable_range_filter) {
            if (params.range_filter.inRange(pt.x, pt.y, pt.z)) {
                if (msg_ptr->is_rain_mode) {
                    float it_threshold = -1;
                    float a = params.noise_point_yaml.polyfit_a; //-0.2;//y = a*x^2+c
                    float c = params.noise_point_yaml.polyfit_c; //45;// (0，+-15)
                    float front = 5;
                    float back = -1;
                    float dist = pt.x;
                    if (dist > front && dist < front + 15) {
                        it_threshold = a * (dist - front) * (dist - front) + c;
                    } else if (dist < back && dist > -15 + back) {
                        it_threshold = a * (dist - back) * (dist - back) + c;
                    } else if (dist >= back && dist <= front) {
                        it_threshold = c;
                    }
                    if (pt.intensity < it_threshold) {
                        noise_indices.emplace_back(i);
                    } else {
                        valid_indices.emplace_back(i);
                    }
                } else {
                    valid_indices.emplace_back(i);
                }
            }
        }
    }
    if (msg_ptr->is_rain_mode) {
        noisePoint(msg_ptr);
    }
}
void RsPreprocessing::noisePoint(const LidarFrameMsg::Ptr &msg_ptr) {
    const auto &scan_ptr = msg_ptr->scan_ptr;
    auto &valid_indices = msg_ptr->valid_indices;

    auto &noise_indices = msg_ptr->noise_indices;
    //H:add noise_point module
    float grid_size = params.noise_point_yaml.grid_size; //0.2;
    float xmin = -16;
    float xmax = 19;
    float ymin = -5;
    float ymax = 5;
    float zmin = -1;
    float zmax = 3;
    int cols = (xmax - xmin) / grid_size;
    int rows = (ymax - ymin) / grid_size;
    std::unordered_map<int, std::vector<int>> bev_valid_map_; // x -16,19,y -5,5
    std::vector<int> bev_noisenum_vec_(cols * rows, 0);       // x -16,19,y -5,5
    std::vector<bool> bev_todel_;
    bev_valid_map_.clear();
    bev_todel_.clear();
    bev_valid_map_.reserve(rows * cols);
    bev_valid_map_.max_load_factor(0.25);
    for (size_t i = 0; i < valid_indices.size(); ++i) {
        const auto &pt_idx = valid_indices[i];
        const auto &pt = scan_ptr->points[pt_idx];
        if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
            continue;
        }
        if (pt.x < xmin || pt.x >= xmax ||
            pt.y < ymin || pt.y >= ymax ||
            pt.z < zmin || pt.z >= zmax) {
            continue;
        }
        int row = static_cast<int>((pt.y - ymin) / grid_size);
        int col = static_cast<int>((pt.x - xmin) / grid_size);
        int bev_idx = row * cols + col;
        if (bev_idx >= cols * rows) {
            continue;
        }
        if (bev_valid_map_.find(bev_idx) == bev_valid_map_.end()) {
            bev_valid_map_.emplace(bev_idx, std::vector<int>());
            bev_valid_map_[bev_idx].reserve(4000);
        }
        bev_valid_map_[bev_idx].emplace_back(i);
    }
    for (size_t i = 0; i < noise_indices.size(); ++i) {
        const auto &pt_idx = noise_indices[i];
        const auto &pt = scan_ptr->points[pt_idx];
        if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
            continue;
        }
        if (pt.x < xmin || pt.x >= xmax ||
            pt.y < ymin || pt.y >= ymax ||
            pt.z < zmin || pt.z >= zmax) {
            continue;
        }
        int row = static_cast<int>((pt.y - ymin) / grid_size);
        int col = static_cast<int>((pt.x - xmin) / grid_size);
        int bev_idx = row * cols + col;
        if (bev_idx >= cols * rows) {
            continue;
        }
        bev_noisenum_vec_[bev_idx]++;
    }
    // to find 10% z top
    float todel_ingrid_percept = params.noise_point_yaml.todel_ingrid_percept;
    float noise_ingrid_percent = params.noise_point_yaml.noise_ingrid_percent;

    std::vector<int> todel_idx;
    // bool debug_runned=false;
    for (auto bi : bev_valid_map_) {
        int grid_idx = bi.first;
        int valid_num = bi.second.size();
        int noise_num = bev_noisenum_vec_[grid_idx];
        if (noise_num > (valid_num + noise_num) * noise_ingrid_percent) { // 1/3
            auto &grid_validIdx = bi.second;
            // if(debug_runned==false){std::cout<<H_DEBUG_R<<"grid_idx:"<<grid_idx<<std::endl;}
            int num_points = floor(grid_validIdx.size() * todel_ingrid_percept);
            // if(debug_runned==false){std::cout<<"num_points:"<<num_points<<std::endl;}
            std::vector<float> grid_point_z;
            for (int i = 0; i < grid_validIdx.size(); i++) {
                int idx = valid_indices[grid_validIdx[i]];
                float z = scan_ptr->points[idx].z;
                grid_point_z.emplace_back(z);
            }
            std::map<int, int> orig_index; // <z,validIdx>
            for (int i = 0; i < grid_point_z.size(); i++) {
                orig_index[grid_point_z[i]] = grid_validIdx[i];
                // if(debug_runned==false){std::cout<<"orig_index  grid_point_z[i]:"<<grid_point_z[i]<<", grid_validIdx["<<i<<"]:"<<grid_validIdx[i]<<std::endl;}
            }
            std::sort(grid_point_z.begin(), grid_point_z.end(), std::greater<int>());
            // for (int i = 0; i < num_points; i++) {
            //     todel_idx.emplace_back(orig_index[grid_point_z[i]]);
            //     if(debug_runned==false){std::cout<<"grid_point_z["<<i<<"]:"<<grid_point_z[i]<<", orig_index["<<grid_point_z[i]<<"]:"<<orig_index[grid_point_z[i]]<<std::endl;}
            // }
            for (int i = 0; i < grid_validIdx.size(); i++) {
                int idx = valid_indices[grid_validIdx[i]];
                float z = scan_ptr->points[idx].z;
                if (z > grid_point_z[num_points - 1]) {
                    todel_idx.emplace_back(grid_validIdx[i]);
                }
            }
            //print grid z
            int idx = valid_indices[grid_validIdx[0]];
            auto &pt = scan_ptr->points[idx];
        }
    }
    // todel
    std::sort(todel_idx.begin(), todel_idx.end(), std::greater<int>());
    for (int i : todel_idx) {
        valid_indices[i] = valid_indices.back();
        valid_indices.pop_back();
    }
}

} // namespace robosense
