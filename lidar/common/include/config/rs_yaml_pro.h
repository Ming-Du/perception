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
#ifndef RS_COMMON_RS_YAML_PRO_H_
#define RS_COMMON_RS_YAML_PRO_H_

#include "ros/ros.h"
#include "yaml-cpp/yaml.h"
#include "rs_yaml_param.h"
#include "basic_type/range.h"

#include "common/include/rs_define.h"

#define YAML_NODE_REQUIRED(node, key, type) \
    do { \
        if (!node[key]) { \
            throw std::runtime_error(std::string(key) + "[not found]"); \
        } else { \
            try { \
                type = node[key].as<decltype(type)>(); \
            } catch(const YAML::Exception& e) { \
                throw std::runtime_error(std::string(key) + "[" + std::string(e.what()) + "]"); \
            } catch (const std::runtime_error& e) { \
                throw std::runtime_error(std::string(key) + "." + std::string(e.what())); \
            } \
        } \
    } while (false)


namespace YAML {
    template<>
    struct convert<robosense::Range3D> {
        static bool decode(const Node& node, robosense::Range3D& rhs) {
            YAML_NODE_REQUIRED(node, "xmin", rhs.xmin);
            YAML_NODE_REQUIRED(node, "xmax", rhs.xmax);
            YAML_NODE_REQUIRED(node, "ymin", rhs.ymin);
            YAML_NODE_REQUIRED(node, "ymax", rhs.ymax);
            YAML_NODE_REQUIRED(node, "zmin", rhs.zmin);
            YAML_NODE_REQUIRED(node, "zmax", rhs.zmax);
            return true;
        }
    };
    template<>
    struct convert<robosense::LocalPointParam> {
        static bool decode(const Node& node, robosense::LocalPointParam& rhs) {
            if (!node.IsMap()) {
                return false;
            }
            YAML_NODE_REQUIRED(node, "hack_area_x", rhs.hack_area_x);
            YAML_NODE_REQUIRED(node, "hack_area_y", rhs.hack_area_y);
            YAML_NODE_REQUIRED(node, "hack_area_distance", rhs.hack_area_distance);
            return true;
        }
    };
    template<>
    struct convert<robosense::PreprocessingParam::NoisePoint> {
        static bool decode(const Node& node, robosense::PreprocessingParam::NoisePoint& rhs) {
            if (!node.IsMap()) {
                return false;
            }
            YAML_NODE_REQUIRED(node, "polyfit_a", rhs.polyfit_a);
            YAML_NODE_REQUIRED(node, "polyfit_c", rhs.polyfit_c);
            YAML_NODE_REQUIRED(node, "grid_size", rhs.grid_size);
            YAML_NODE_REQUIRED(node, "todel_ingrid_percept", rhs.todel_ingrid_percept);
            YAML_NODE_REQUIRED(node, "noise_ingrid_percent", rhs.noise_ingrid_percent);
            return true;
        }
    };

    template<>
    struct convert<robosense::PreprocessingParam> {
        static bool decode(const Node& node, robosense::PreprocessingParam& rhs) {
            if (!node.IsMap()) {
                return false;
            }
            YAML_NODE_REQUIRED(node["vehicle_filter"], "enable", rhs.enable_vehicle_filter);
            if (rhs.enable_vehicle_filter)
            {
                YAML_NODE_REQUIRED(node, "vehicle_filter", rhs.vehicle_filter);
            }
            YAML_NODE_REQUIRED(node["range_filter"], "enable", rhs.enable_range_filter);
            if (rhs.enable_range_filter) {
                YAML_NODE_REQUIRED(node, "range_filter", rhs.range_filter);
            }
            YAML_NODE_REQUIRED(node, "noise_point", rhs.noise_point_yaml);
            return true;
        }
    };

    template<>
    struct convert<robosense::GridMapParam> {
        static bool decode(const Node& node, robosense::GridMapParam& rhs) {
            if (!node.IsMap()) {
                return false;
            }
            YAML_NODE_REQUIRED(node, "enable", rhs.enable_);
            if (rhs.enable_){
                YAML_NODE_REQUIRED(node, "unit_size", rhs.unit_size_);
                YAML_NODE_REQUIRED(node, "range_filter", rhs.range_);
            }
            return true;
        }
    };

    template<>
    struct convert<robosense::DenoiseParam> {
        static bool decode(const Node& node, robosense::DenoiseParam& rhs) {
            if (!node.IsMap()) {
                return false;
            }
            YAML_NODE_REQUIRED(node, "enable", rhs.enable);
            YAML_NODE_REQUIRED(node, "model", rhs.model);
            YAML_NODE_REQUIRED(node, "confidence", rhs.confidence);
            YAML_NODE_REQUIRED(node, "obj_model", rhs.obj_model);
            YAML_NODE_REQUIRED(node, "obj_confidence", rhs.obj_confidence);
            YAML_NODE_REQUIRED(node, "detect_range", rhs.range);
            YAML_NODE_REQUIRED(node, "unit_size", rhs.unit_size);
            rhs.xsize = static_cast<int>((rhs.range.xmax - rhs.range.xmin) / rhs.unit_size);
            rhs.ysize = static_cast<int>((rhs.range.ymax - rhs.range.ymin) / rhs.unit_size);
            rhs.grid_num = rhs.xsize * rhs.ysize;
            return true;
        }
    };

    template<>
    struct convert<robosense::GroundFilterParam> {
        static bool decode(const Node& node, robosense::GroundFilterParam& rhs) {
            if (!node.IsMap()) {
                return false;
            }
            YAML_NODE_REQUIRED(node, "enable", rhs.enable);
            if (rhs.enable){
                YAML_NODE_REQUIRED(node, "setDistanceThreshold", rhs.setDistanceThreshold);
                YAML_NODE_REQUIRED(node, "use_falcon_distance", rhs.use_falcon_distance);
                YAML_NODE_REQUIRED(node, "cell_distance_ranges", rhs.cell_distance_ranges);
                YAML_NODE_REQUIRED(node, "ground_height_ranges", rhs.ground_height_ranges);
                YAML_NODE_REQUIRED(node, "inner_cell_diff_ranges", rhs.inner_cell_diff_ranges);
                YAML_NODE_REQUIRED(node, "adjacent_cell_diff_ranges", rhs.adjacent_cell_diff_ranges);
                YAML_NODE_REQUIRED(node, "cell_zmin_ranges", rhs.cell_zmin_ranges);
                YAML_NODE_REQUIRED(node, "cell_zmax_ranges", rhs.cell_zmax_ranges);
            }
            return true;
        }
    };
    template<>
    struct convert<robosense::RoiFilterParam> {
        static bool decode(const Node& node, robosense::RoiFilterParam& rhs) {
            if (!node.IsMap()) {
                return false;
            }
            YAML_NODE_REQUIRED(node, "enable", rhs.enable);
            if (rhs.enable) {
                YAML_NODE_REQUIRED(node, "unit_size", rhs.unit_size);
                YAML_NODE_REQUIRED(node, "range_filter", rhs.range);
                YAML_NODE_REQUIRED(node, "show_element_value", rhs.show_element_value);
                YAML_NODE_REQUIRED(node, "roadside_valid_flag", rhs.roadside_valid_flag);
                YAML_NODE_REQUIRED(node, "curload_allowance_dist", rhs.curload_allowance_dist);
            }
            return true;
        }
    };
    template<>
    struct convert<robosense::mapDefine> {
        static bool decode(const Node& node, robosense::mapDefine& rhs) {
            if (!node.IsMap()) {
                ROS_WARN("CHECK ARGS TYPE FAIL, PLEASE CHECK YOUR YAML");
                return false;
            }
            YAML_NODE_REQUIRED(node, "city_name", rhs.city_name);
            YAML_NODE_REQUIRED(node, "mappath", rhs.mappath);
            YAML_NODE_REQUIRED(node, "tilesize", rhs.tilesize);
            YAML_NODE_REQUIRED(node, "gridsize", rhs.gridsize);
            YAML_NODE_REQUIRED(node, "lon_png_min", rhs.lon_png_min);
            YAML_NODE_REQUIRED(node, "lon_png_max", rhs.lon_png_max);
            YAML_NODE_REQUIRED(node, "lat_png_min", rhs.lat_png_min);
            YAML_NODE_REQUIRED(node, "lat_png_max", rhs.lat_png_max);
            YAML_NODE_REQUIRED(node, "lonzero", rhs.lonzero);
            YAML_NODE_REQUIRED(node, "latzero", rhs.latzero);
            YAML_NODE_REQUIRED(node, "utm_start_X", rhs.utm_start_X);
            YAML_NODE_REQUIRED(node, "utm_start_Y", rhs.utm_start_Y);
            return true;
        }
    };
    template<>
    struct convert<std::map<int, robosense::YamlRoadMap> >{
        static bool decode(const Node& node, std::map<int, robosense::YamlRoadMap>& rhs) {
            if (!node.IsSequence()) {
                ROS_WARN("CHECK ARGS TYPE FAIL, PLEASE CHECK YOUR YAML");
                return false;
            }
            for(auto i: node)
            {
                robosense::YamlRoadMap mapValue;
                YAML_NODE_REQUIRED(i, "city", mapValue.city);
                YAML_NODE_REQUIRED(i, "utm_zone", mapValue.utm_zone);
                YAML_NODE_REQUIRED(i, "mappath", mapValue.mappath);
                YAML_NODE_REQUIRED(i, "version", mapValue.version);
                rhs[mapValue.utm_zone] = mapValue;
            }
            return true;
        }
    };
    template<>
    struct convert<std::vector<robosense::YamlRoadMap> > {
        static bool decode(const Node& node, std::vector<robosense::YamlRoadMap>& rhs) {
            if (!node.IsSequence()) {
                ROS_WARN("CHECK ARGS TYPE FAIL, PLEASE CHECK YOUR YAML");
                return false;
            }
            for(auto i: node)
            {
                robosense::YamlRoadMap mapValue;
                YAML_NODE_REQUIRED(i, "city", mapValue.city);
                YAML_NODE_REQUIRED(i, "utm_zone", mapValue.utm_zone);
                YAML_NODE_REQUIRED(i, "mappath", mapValue.mappath);
                YAML_NODE_REQUIRED(i, "version", mapValue.version);
                rhs.push_back(mapValue);
            }
            return true;
        }
    };

    template<>
    struct convert<robosense::SegmentorParam> {
        static bool decode(const Node& node, robosense::SegmentorParam& rhs) {
            if (!node.IsMap()) {
                return false;
            }
            YAML_NODE_REQUIRED(node, "enable", rhs.enable);
            if (rhs.enable){
                YAML_NODE_REQUIRED(node, "split", rhs.enable_split_);
                YAML_NODE_REQUIRED(node, "split_floating_obj", rhs.enable_split_floating_);
                YAML_NODE_REQUIRED(node, "seg_min_pts", rhs.seg_min_pts);
                YAML_NODE_REQUIRED(node, "xgrid_thre", rhs.xgrid_thre_);
                YAML_NODE_REQUIRED(node, "ygrid_thre", rhs.ygrid_thre_);
                YAML_NODE_REQUIRED(node, "init_thre", rhs.init_thre_);
                YAML_NODE_REQUIRED(node, "split_thre_road", rhs.split_thre_road);
                YAML_NODE_REQUIRED(node, "split_thre_roadside", rhs.split_thre_roadside);
                YAML_NODE_REQUIRED(node, "split_floating_zthre", rhs.split_floating_zthre);
            }
            return true;
        }
    };

    template<>
    struct convert<robosense::RefinerParam> {
        static bool decode(const Node& node, robosense::RefinerParam& rhs) {
            if (!node.IsMap()) {
                return false;
            }
            YAML_NODE_REQUIRED(node, "enable", rhs.enable);
            if (rhs.enable){
                YAML_NODE_REQUIRED(node, "floating_filter", rhs.floating_filter);
                YAML_NODE_REQUIRED(node, "floating_range", rhs.floating_range);
                YAML_NODE_REQUIRED(node, "floating_object_size", rhs.floating_object_size);
                YAML_NODE_REQUIRED(node, "floating_height_limited", rhs.floating_height_limited);
                YAML_NODE_REQUIRED(node, "ghost_filter", rhs.ghost_filter);
                YAML_NODE_REQUIRED(node, "ghost_range", rhs.ghost_range);
                YAML_NODE_REQUIRED(node, "ghost_size", rhs.ghost_size);
                YAML_NODE_REQUIRED(node, "flowerbed_filter", rhs.flowerbed_filter);
                YAML_NODE_REQUIRED(node, "flowerbed_range", rhs.flowerbed_range);
                YAML_NODE_REQUIRED(node, "flowerbed_object_size", rhs.flowerbed_object_size);
                YAML_NODE_REQUIRED(node, "flowerbed_height_limited", rhs.flowerbed_height_limited);
                YAML_NODE_REQUIRED(node, "pps_isfalcon", rhs.pps_isfalcon);
                YAML_NODE_REQUIRED(node, "airefine_batch_size", rhs.airefine_batch_size);
                YAML_NODE_REQUIRED(node, "expand_proposal_meter", rhs.expand_proposal_meter);
                YAML_NODE_REQUIRED(node, "proposal_cls_thres", rhs.proposal_cls_thres);
                YAML_NODE_REQUIRED(node, "other_expand_ratio", rhs.other_expand_ratio);
                YAML_NODE_REQUIRED(node, "enable_AIRoiFilter", rhs.enable_AIRoiFilter);
                YAML_NODE_REQUIRED(node, "enable_VegetTag", rhs.enable_VegetTag);
                YAML_NODE_REQUIRED(node, "enable_PedFilter", rhs.enable_PedFilter);
                YAML_NODE_REQUIRED(node, "enable_BoxRefine", rhs.enable_BoxRefine);
            }
            return true;
        }
    };

    template<>
    struct convert<robosense::TrackerParam> {
        static bool decode(const Node& node, robosense::TrackerParam& rhs) {
            if (!node.IsMap()) {
                return false;
            }
            YAML_NODE_REQUIRED(node, "enable", rhs.enable);
            if (rhs.enable){
                YAML_NODE_REQUIRED(node, "respective_match", rhs.respective_match);
                YAML_NODE_REQUIRED(node, "thredist", rhs.thredist);
                YAML_NODE_REQUIRED(node, "thredistmax", rhs.thredistmax);
                YAML_NODE_REQUIRED(node, "thredistmin", rhs.thredistmin);
                YAML_NODE_REQUIRED(node, "threheading", rhs.threheading);
                YAML_NODE_REQUIRED(node, "det2actthre", rhs.det2actthre);
                YAML_NODE_REQUIRED(node, "det2freethre", rhs.det2freethre);
                YAML_NODE_REQUIRED(node, "active2freethre", rhs.active2freethre);
                YAML_NODE_REQUIRED(node, "maintainAItimes", rhs.maintainAItimes);
            }
            return true;
        }
    };

    template<>
    struct convert<robosense::RvizParam> {
        static bool decode(const Node& node, robosense::RvizParam& rhs) {
            YAML_NODE_REQUIRED(node, "enable", rhs.enable);
            YAML_NODE_REQUIRED(node, "prefix", rhs.prefix);
            YAML_NODE_REQUIRED(node, "debug_mode", rhs.debug_mode);
            YAML_NODE_REQUIRED(node, "frame_id", rhs.frame_id);
            return true;
        }
    };

    template<>
    struct convert<robosense::debugInfoParam> {
        static bool decode(const Node& node, robosense::debugInfoParam& rhs) {
            YAML_NODE_REQUIRED(node, "perception", rhs.enable_perception);
            YAML_NODE_REQUIRED(node, "tracking", rhs.enable_tracking);
            YAML_NODE_REQUIRED(node, "collect", rhs.enable_collect);
            YAML_NODE_REQUIRED(node, "xmin", rhs.xmin);
            YAML_NODE_REQUIRED(node, "xmax", rhs.xmax);
            YAML_NODE_REQUIRED(node, "ymin", rhs.ymin);
            YAML_NODE_REQUIRED(node, "ymax", rhs.ymax);
            return true;
        }
    };
}
#endif  // RS_COMMON_RS_YAML_PRO_H_
