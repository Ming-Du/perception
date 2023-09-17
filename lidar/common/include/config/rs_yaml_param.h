#ifndef RS_COMMON_RS_YAML_PARAM_H_
#define RS_COMMON_RS_YAML_PARAM_H_
#include "basic_type/range.h"
namespace robosense{
    struct LocalPointParam {
        double hack_area_x;
        double hack_area_y;
        double hack_area_distance;
    };

    struct PreprocessingParam {
        bool enable_vehicle_filter = false;
        Range3D vehicle_filter = {0.,0.,0.,0.,0.,0.};
        bool enable_range_filter = false;
        Range3D range_filter = {-100., 100., -100., 100., -100., 100.};
        struct NoisePoint{
            double polyfit_a;
            double polyfit_c;
            double grid_size;
            double todel_ingrid_percept;
            double noise_ingrid_percent;
        }noise_point_yaml = {-0.2,45,0.2,0.1,0.25};

        std::string infos() {
            std::ostringstream os;
            os << "preprocessing" << std::endl <<
               "  enable_vehicle_filter:" <<  enable_vehicle_filter << std::endl <<
               "  vehicle_filter:" << vehicle_filter.infos() <<
               "  enable_range_filter:" << enable_range_filter << std::endl <<
               "  range_filter" << range_filter.infos();
            return os.str();
        }
    };
    struct GridMapParam {
        bool enable_ = false;
        Range3D range_;
        double unit_size_ = 0.2;
        std::string infos(const std::string &name) {
            std::ostringstream os;
            os << name << ": range_enable: " << enable_ << std::endl;
            os << name << ": range:" << range_.infos() << std::endl;
            return os.str();
        }
    };
    struct DenoiseParam {
        bool enable = false;
        std::string model;
        std::string xgb_model_path;
        double confidence = 0.5;
        std::string obj_model;
        double obj_confidence = 0.5;
        Range3D range = {-60., 60., -30., 30., -1, 2.};
        double unit_size = 1;
        double xsize = 1;
        double ysize = 1;
        int grid_num = 1;
        std::string infos() {
            std::ostringstream os;
            os << "denoise:" << std::endl << "  enable:" << enable << std::endl <<
               "  model:" << model << std::endl << "  xgb_model_path:" <<  xgb_model_path << std::endl <<
               "  confidence:" << confidence << std::endl << "  detect_range:" << range.infos() <<
               "  unit_size:" << unit_size << std::endl << "  xsize:" << xsize << std::endl <<
               "  ysize:" << ysize << std::endl << "  grid_num:" << grid_num;
            return os.str();
        }
    };
    struct GroundFilterParam {
        bool enable = false;
        double setDistanceThreshold = 0.2;
        double use_falcon_distance = 50;
        std::vector<double> cell_distance_ranges;
        std::vector<double> ground_height_ranges;
        std::vector<double> inner_cell_diff_ranges;
        std::vector<double> adjacent_cell_diff_ranges;
        std::vector<double> cell_zmin_ranges;
        std::vector<double> cell_zmax_ranges;
    };

    struct mapDefine {
        std::string city_name;
        std::string mappath;
        int tilesize;
        double gridsize;
        double lon_png_min;
        double lon_png_max;
        double lat_png_min;
        double lat_png_max;
        double lonzero;
        double latzero;
        double utm_start_X;
        double utm_start_Y;
    };
    struct YamlRoadMap {
        std::string city;
        int utm_zone;
        std::string mappath;
        std::string version;
    };
    struct roadmapDefine {
        std::vector<YamlRoadMap> map_list;
    };

    struct RoiFilterParam {
        bool enable = false;
        bool is_use_roadmap = false;
        std::string map_path;
        Range3D range = {-100., 100., -100., 100., -100., 100.};
        double unit_size = 0.2;
        int show_element_value = 1;
        bool roadside_valid_flag = true;
        double curload_allowance_dist = 20;
        double search_range_curload_;
        
        double getMaxDist() {
            double dist[4];
            dist[0] = range.xmin;
            dist[1] = range.xmax;
            dist[2] = range.ymin;
            dist[3] = range.ymax;
            double abs_dist_max = 0;
            double abs_x_max = 0;
            double abs_y_max = 0;
            for (int i = 0; i < 2; i++) {
                if (abs_x_max < fabs(dist[i])) {
                    abs_x_max = fabs(dist[i]);
                }
            }
            for (int i = 2; i < 4; i++) {
                if (abs_y_max < fabs(dist[i])) {
                    abs_y_max = fabs(dist[i]);
                }
            }
            abs_dist_max = sqrt(abs_x_max * abs_x_max + abs_y_max * abs_y_max);
            return abs_dist_max;
        }
        void setSearchRange() {
            search_range_curload_ = getMaxDist() + curload_allowance_dist; //80;//60*1.5=90;bus 36km/h = 10m/s; 2 frames time. 90+10*2=110
        }
    };

    struct SegmentorParam {
        std::string frame_id;
        bool enable = false;
        bool enable_split_ = false;
        bool enable_split_floating_ = false;
        int seg_min_pts = 3;
        int xgrid_thre_ = 5;
        int ygrid_thre_ = 5;
        double init_thre_ = 2;
        int split_thre_road = 60;
        int split_thre_roadside = 50;
        double split_floating_zthre = 2.0;
    };
    
    struct RefinerParam {
        bool enable = false;
        bool floating_filter = false;
        Range3D floating_range = {-30., 30., -30., 30., 0., 0.};
        std::vector<double> floating_object_size;
        double floating_height_limited = 0;
        bool ghost_filter = false;
        Range3D ghost_range = {0., 0., 0., 0., 0., 0.};
        std::vector<double> ghost_size;
        bool flowerbed_filter = false;
        Range3D flowerbed_range = {0., 0., 0., 0., 0., 0.};
        std::vector<double> flowerbed_object_size;
        double flowerbed_height_limited = 0;
        bool pps_isfalcon = false;
        int airefine_batch_size = 4;
        double expand_proposal_meter = 0.8;
        double proposal_cls_thres = 1.0;
        double other_expand_ratio = 0.5;
        bool enable_AIRoiFilter = false;
        bool enable_VegetTag = true;
        bool enable_BoxRefine = false;
        bool enable_PedFilter = true;
    };

    struct TrackerParam {
        bool enable = false;
        // respectively match ai and rb
        bool respective_match = false;
        // association params
        double thredist;
        double thredistmax;
        double thredistmin;
        double threheading;

        // state machine params
        uint16_t det2actthre;
        uint16_t det2freethre;
        uint16_t active2freethre;
        uint16_t maintainAItimes;
    };
    
    struct RvizParam {
        bool enable;
        std::string prefix;
        bool debug_mode;
        std::string frame_id;
    };


    struct debugInfoParam {
        bool enable_perception = false;
        bool enable_tracking = false;
        bool enable_collect = false;
        double xmax = 35.;
        double xmin = 5.;
        double ymax = 2.;
        double ymin = -2.;
    };
}



#endif //RS_COMMON_RS_YAML_PARAM_H_
