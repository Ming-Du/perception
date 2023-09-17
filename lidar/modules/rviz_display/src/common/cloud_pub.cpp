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
#include "common/cloud_pub.h"
#define random(x) (rand() % x)
namespace robosense {

void CloudPub::pubCloud(const LidarFrameMsg::Ptr &msg_ptr, const DisplayModule &display_module) {
    const auto &scan_ptr = msg_ptr->scan_ptr;

    // origin
    if(display_module == DisplayModule::PREPROCESS){
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        const auto &valid_indice = msg_ptr->valid_indices;
        cloud_ptr->points.reserve(valid_indice.size());
        for (size_t i = 0; i < valid_indice.size(); ++i) {
            cloud_ptr->points.emplace_back(scan_ptr->points[valid_indice[i]]);
        }
        cloud_ptr->header.frame_id = options_.frame_id;
        pub_origin.publish(cloud_ptr);
    }
    if (msg_ptr->grid_map_ptr == nullptr) return;

    // ground & non ground
    // TODO：close pub temporarily
    #if 0
    if(display_module == DisplayModule::GROUND_FILTER){
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr non_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        ground_cloud_ptr->points.reserve(msg_ptr->valid_indices.size());
        non_ground_cloud_ptr->points.reserve(msg_ptr->valid_indices.size());
        for (auto &id : msg_ptr->grid_map_ptr->roi_cell_id_vec) {
            CellInfoPtr cell_info_ptr = msg_ptr->grid_map_ptr->getCellInfo(id);
            if (cell_info_ptr->cart_ground_flag_) {
                for (auto &indices : cell_info_ptr->ground_indices_) {
                    ground_cloud_ptr->points.emplace_back(scan_ptr->points.at(indices));
                }
            } else {
                for (auto &indices : cell_info_ptr->non_ground_indices_) {
                    non_ground_cloud_ptr->points.emplace_back(scan_ptr->points.at(indices));
                }
            }
        }
        ground_cloud_ptr->header.frame_id = options_.frame_id;
        ground_cloud_ptr->header.stamp = msg_ptr->timestamp * 1e+6;
        pub_ground.publish(ground_cloud_ptr);

        non_ground_cloud_ptr->header.frame_id = options_.frame_id;
        non_ground_cloud_ptr->header.stamp = msg_ptr->timestamp * 1e+6;
        pub_non_ground.publish(non_ground_cloud_ptr);
        //semantic map
        if(msg_ptr->is_semantic_map){
            pcl::PointCloud<pcl::PointXYZI>::Ptr semantic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
            semantic_cloud_ptr->points.reserve(msg_ptr->grid_map_ptr->cart_grid_count);
            for (auto &index : msg_ptr->pointcloud_map) {
                    pcl::PointXYZI pt;
                    pt.x = index.x();
                    pt.y = index.y();
                    pt.z = index.z();
                    pt.intensity = 255;
                    semantic_cloud_ptr->points.emplace_back(pt);
            }
            semantic_cloud_ptr->header.frame_id = options_.frame_id;
            semantic_cloud_ptr->header.stamp = msg_ptr->timestamp * 1e+6;
            pub_map.publish(semantic_cloud_ptr);
        }
    }
    #endif

    // noise
    if(display_module == DisplayModule::DENOISE){
        pcl::PointCloud<pcl::PointXYZI>::Ptr noise_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        const auto &noise_indices = msg_ptr->noise_indices;
        noise_ptr->points.reserve(noise_indices.size());
        for (size_t i = 0; i < noise_indices.size(); ++i) {
            noise_ptr->points.emplace_back(scan_ptr->points[noise_indices[i]]);
        }
        noise_ptr->header.frame_id = options_.frame_id;
        noise_ptr->header.stamp = msg_ptr->timestamp * 1e+6;
        pub_noise.publish(noise_ptr);
    }

    // refiner
    if(display_module == DisplayModule::REFINER){
        const auto &lshape_ptr = msg_ptr->shape_indices;
        pcl::PointCloud<pcl::PointXYZI>::Ptr shape_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        shape_ptr->reserve(lshape_ptr.size());
        if (lshape_ptr.size() > 2) {
            for (size_t i = 0; i < lshape_ptr.size(); ++i) {
                pcl::PointXYZI o;
                o.x = lshape_ptr.at(i).at(0);
                o.y = lshape_ptr.at(i).at(1);
                o.z = 1;
                o.intensity = 10;
                shape_ptr->points.emplace_back(o);
            }
            shape_ptr->header.frame_id = options_.frame_id;
            pub_lshape.publish(shape_ptr);
        }
    }

}
} // namespace robosense