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
#include "rviz_display.h"
#include <nav_msgs/GridCells.h>
#include <nav_msgs/OccupancyGrid.h>
namespace robosense {

RvizDisplay::RvizDisplay(const Rs_YAMLReader &configParse) {
    node_ptr.reset(new ros::NodeHandle);
    params_ = configParse.getValue<RvizParam>("rviz");
    ground_enable = configParse.getValue<bool>("ground_filter.enable");

    MarkerPubOptions marker_options;
    marker_options.pre_fix = params_.prefix;
    marker_options.frame_id = params_.frame_id;
    marker_options.node_ptr = node_ptr;
    box_pub_.reset(new BoxMarkerPub);
    box_pub_->init(marker_options);

    cube_pub_.reset(new CubeMarkerPub);
    cube_pub_->init(marker_options);

    denoise_pub_.reset(new DenoiseMarkerPub);
    denoise_pub_->init(marker_options);

    arrow_pub_.reset(new ArrowMarkerPub);
    arrow_pub_->init(marker_options);

    label_pub_.reset(new LabelInfosMarkerPub);
    label_pub_->init(marker_options);

    polygon_pub_.reset(new PolygonMarkerPub);
    polygon_pub_->init(marker_options);

    track_pub_.reset(new TrackInfoPubMarker);
    track_pub_->init(marker_options);

    refine_pub_.reset(new RefineMarkerPub);
    refine_pub_->init(marker_options);

    CloudPubOptions cloud_options;
    cloud_options.pre_fix = params_.prefix;
    cloud_options.frame_id = params_.frame_id;
    cloud_options.node_ptr = node_ptr;
    cloud_pub_ptr_.reset(new CloudPub);
    cloud_pub_ptr_->init(cloud_options);

    marker_array_ptr_.reset(new ROS_VISUALIZATION_MARKERARRAY);

    pub_perception_ = node_ptr->advertise<ROS_VISUALIZATION_MARKERARRAY>(params_.prefix + "percept_info_rviz", 1, true);
    pub_binmap_ = node_ptr->advertise<nav_msgs::GridCells>(params_.prefix + "binmap", 1, true);
    pub_roadmap_ = node_ptr->advertise<nav_msgs::GridCells>(params_.prefix + "roadmap", 1, true);
    pub_denoisemap_object_ = node_ptr->advertise<nav_msgs::GridCells>(params_.prefix + "denoise_obj", 1, true);
    pub_denoisemap_noise_ = node_ptr->advertise<nav_msgs::GridCells>(params_.prefix + "denoise_nos", 1, true);
}

void RvizDisplay::display(const LidarFrameMsg::Ptr &msg_ptr, const DisplayModule &display_module) {
    if (msg_ptr == nullptr || msg_ptr->grid_map_ptr == nullptr)
        return;

    if (!params_.enable) {
        return;
    }

    if(!ground_enable && display_module == DisplayModule::GROUND_FILTER) 
        return;

    TRY_CATCH
    {
        marker_array_ptr_->markers.clear();
        marker_array_ptr_->markers.shrink_to_fit();

        cloud_pub_ptr_->pubCloud(msg_ptr, display_module);

        if(display_module == DisplayModule::TRACKER){
            auto marker_list = cube_pub_->display(msg_ptr);
            marker_array_ptr_->markers.reserve(marker_array_ptr_->markers.size() + marker_list.size());
            for (const auto &tmp_marker : marker_list) {
                marker_array_ptr_->markers.emplace_back(tmp_marker);
            }

            auto denoise_marker_list = denoise_pub_->display(msg_ptr);
            marker_array_ptr_->markers.reserve(marker_array_ptr_->markers.size() + denoise_marker_list.size());
            for (const auto &tmp_marker : denoise_marker_list) {
                marker_array_ptr_->markers.emplace_back(tmp_marker);
            }

            auto denoise_deleted_marker_list = denoise_pub_->display_deleted(msg_ptr);
            marker_array_ptr_->markers.reserve(marker_array_ptr_->markers.size() + denoise_deleted_marker_list.size());
            for (const auto &tmp_marker : denoise_deleted_marker_list) {
                marker_array_ptr_->markers.emplace_back(tmp_marker);
            }

            auto arrow_marker_list = arrow_pub_->display(msg_ptr);
            marker_array_ptr_->markers.reserve(marker_array_ptr_->markers.size() + arrow_marker_list.size());
            for (const auto &tmp_marker : arrow_marker_list) {
                marker_array_ptr_->markers.emplace_back(tmp_marker);
            }

            auto marker_list_label = label_pub_->display(msg_ptr);
            marker_array_ptr_->markers.reserve(marker_array_ptr_->markers.size() + marker_list_label.size());
            for (const auto &tmp_marker : marker_list_label) {
                marker_array_ptr_->markers.emplace_back(tmp_marker);
            }

            auto marker_list_polygon = polygon_pub_->display(msg_ptr);
            marker_array_ptr_->markers.reserve(marker_array_ptr_->markers.size() + marker_list_polygon.size());
            for (const auto &tmp_marker : marker_list_polygon) {
                marker_array_ptr_->markers.emplace_back(tmp_marker);
            }

            auto marker_list_track = track_pub_->display(msg_ptr);
            marker_array_ptr_->markers.reserve(marker_array_ptr_->markers.size() + marker_list_track.size());
            for (const auto &tmp_marker : marker_list_track) {
                marker_array_ptr_->markers.emplace_back(tmp_marker);
            }

            if (params_.debug_mode) {
                auto marker_list_box_ai = box_pub_->display_ai(msg_ptr);
                marker_array_ptr_->markers.reserve(marker_array_ptr_->markers.size() + marker_list_box_ai.size());
                for (const auto &tmp_marker : marker_list_box_ai) {
                    marker_array_ptr_->markers.emplace_back(tmp_marker);
                }

                auto marker_list_polygon_ori = polygon_pub_->display_ori(msg_ptr);
                marker_array_ptr_->markers.reserve(marker_array_ptr_->markers.size() + marker_list_polygon_ori.size());
                for (const auto &tmp_marker : marker_list_polygon_ori) {
                    marker_array_ptr_->markers.emplace_back(tmp_marker);
                }

                auto marker_list_box_refine = refine_pub_->display(msg_ptr);
                marker_array_ptr_->markers.reserve(marker_array_ptr_->markers.size() + marker_list_box_refine.size());
                for (const auto &tmp_marker : marker_list_box_refine) {
                    marker_array_ptr_->markers.emplace_back(tmp_marker);
                }

                auto marker_list_polygon_ori_noise = polygon_pub_->display_ori_noise(msg_ptr);
                marker_array_ptr_->markers.reserve(marker_array_ptr_->markers.size() + marker_list_polygon_ori_noise.size());
                for (const auto &tmp_marker : marker_list_polygon_ori_noise) {
                    marker_array_ptr_->markers.emplace_back(tmp_marker);
                }
                auto marker_list_polygon_ori_object = polygon_pub_->display_ori_object(msg_ptr);
                marker_array_ptr_->markers.reserve(marker_array_ptr_->markers.size() + marker_list_polygon_ori_object.size());
                for (const auto &tmp_marker : marker_list_polygon_ori_object) {
                    marker_array_ptr_->markers.emplace_back(tmp_marker);
                }

                auto marker_list_denoise_cur = label_pub_->display_denoise_current(msg_ptr);
                marker_array_ptr_->markers.reserve(marker_array_ptr_->markers.size() + marker_list_denoise_cur.size());
                for (const auto &tmp_marker : marker_list_denoise_cur) {
                    marker_array_ptr_->markers.emplace_back(tmp_marker);
                }

                auto marker_list_denoise_history = label_pub_->display_denoise_history(msg_ptr);
                marker_array_ptr_->markers.reserve(marker_array_ptr_->markers.size() + marker_list_denoise_history.size());
                for (const auto &tmp_marker : marker_list_denoise_history) {
                    marker_array_ptr_->markers.emplace_back(tmp_marker);
                }

            }
            // std::cout << "ai size: " << msg_ptr->objects_ai.size() << ", rb size: " << msg_ptr->objects_rule.size() << ", refine size: " << msg_ptr->objects_refine.size() << ", tracker size: " << msg_ptr->objects.size() << std::endl;
            pub_perception_.publish(*marker_array_ptr_);
        }

        if(display_module == DisplayModule::DENOISE){
            // show denoise map--object
            float unit_size = 1.0; // 1.0*1.0m
            auto &noisemapobj = msg_ptr->denoise_grid.noisemapobj;

            nav_msgs::GridCells cells_obj;
            cells_obj.header.frame_id = "base_link";
            cells_obj.cell_height = unit_size;
            cells_obj.cell_width = unit_size;
            if (noisemapobj.size() < 1) {
                geometry_msgs::Point obstacle;
                obstacle.x = 0;
                obstacle.y = 0;
                obstacle.z = 0;
                cells_obj.cells.push_back(obstacle);
                ROS_DEBUG_STREAM("noisemapobj.size() < 1");
            } else {
                for (auto mi : noisemapobj) {
                    geometry_msgs::Point obstacle;
                    obstacle.x = mi.x();
                    obstacle.y = mi.y();
                    obstacle.z = 0;
                    cells_obj.cells.push_back(obstacle);
                }
            }
            pub_denoisemap_object_.publish(cells_obj);
    
            // show denoise map--noise
            auto &noisemapnos = msg_ptr->denoise_grid.noisemapnos;

            nav_msgs::GridCells cells_nos;
            cells_nos.header.frame_id = "base_link";
            cells_nos.cell_height = unit_size;
            cells_nos.cell_width = unit_size;
            if (noisemapnos.size() < 1) {
                geometry_msgs::Point obstacle;
                obstacle.x = 0;
                obstacle.y = 0;
                obstacle.z = 0;
                cells_nos.cells.push_back(obstacle);
                ROS_DEBUG_STREAM("noisemapnos.size() < 1");
            } else {
                for (auto mi : noisemapnos) {
                    geometry_msgs::Point obstacle;
                    obstacle.x = mi.x();
                    obstacle.y = mi.y();
                    obstacle.z = 0;
                    cells_nos.cells.push_back(obstacle);
                }
            }
            pub_denoisemap_noise_.publish(cells_nos);
        }

        //  show road map
        if(display_module == DisplayModule::ROI_FILTER){
            if(msg_ptr->grid_map_ptr == nullptr) return;
            float unit_size = msg_ptr->grid_map_ptr->cell_size;
            auto &roadmap = msg_ptr->roadmap;

            nav_msgs::GridCells cells;
            cells.header.frame_id = "base_link";
            cells.cell_height = unit_size;
            cells.cell_width = unit_size;
            if (roadmap.size() < 1) {
                geometry_msgs::Point obstacle;
                obstacle.x = 0;
                obstacle.y = 0;
                obstacle.z = 0;
                cells.cells.push_back(obstacle);
                ROS_DEBUG_STREAM("roadmap.size() < 1");
            } else {
                for (auto mi : roadmap) {
                    geometry_msgs::Point obstacle;
                    obstacle.x = mi.x();
                    obstacle.y = mi.y();
                    obstacle.z = 0;
                    cells.cells.push_back(obstacle);
                }
            }
            pub_roadmap_.publish(cells);
            roadmap.clear();
        }

        //  show bin map
        if(display_module == DisplayModule::GROUND_FILTER){
            if(msg_ptr->grid_map_ptr == nullptr) return;
            float unit_size = msg_ptr->grid_map_ptr->cell_size;
            auto &binmap = msg_ptr->binmap;

            nav_msgs::GridCells cells;
            cells.header.frame_id = "base_link";
            cells.cell_height = unit_size;
            cells.cell_width = unit_size;
            if (binmap.size() < 1) {
                geometry_msgs::Point obstacle;
                obstacle.x = 0;
                obstacle.y = 0;
                obstacle.z = 0;
                cells.cells.push_back(obstacle);
            }
            for (auto mi : binmap) {
                geometry_msgs::Point obstacle;
                obstacle.x = mi.x();
                obstacle.y = mi.y();
                obstacle.z = 0;
                cells.cells.push_back(obstacle);
            }
            pub_binmap_.publish(cells);
            binmap.clear();
        }

    }
    END_TRY_CATCH
}

} // namespace robosense