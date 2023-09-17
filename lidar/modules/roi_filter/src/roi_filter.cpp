#include "roi_filter.h"

#include "common/include/config/rs_yamlReader.h"
#include <sys/stat.h>
#include <sys/types.h>

namespace robosense {

RoiFilter::RoiFilter() 
    : exit_flag_(false)
    , valid_img_loaded_flag_(false)
{

}

RoiFilter::~RoiFilter() {
    if (t_img_load_ == nullptr)
        return;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        exit_flag_ = true;
    }
    img_load_cv_.notify_one();
    t_img_load_->join();
}

void RoiFilter::init(const Rs_YAMLReader &configParse) {
    configParse_ = configParse;
    params_ = configParse.getValue<RoiFilterParam>("roi_filter");
    map_info_ = configParse_.getValue<mapDefine>("map");
    map_list_ = configParse_.getValue<std::map<int, YamlRoadMap>>("roadmap.map_list");

    params_.map_path = configParse.getMapPath();
    params_.setSearchRange();

    img_path_ = params_.map_path + map_info_.mappath;

    t_img_load_ = std::make_shared<std::thread>(std::bind(&RoiFilter::pngLoadThread, this));
}

void RoiFilter::updataLocalization(const localization::Localization &local_current, bool is_parser_map_info) {
    int cur_utm_zone = local_current.utm_zone();
    if (map_list_.find(cur_utm_zone) == map_list_.end()) {
        ROS_WARN_STREAM("Can not find zone in config files. cur_utm_zone = " << cur_utm_zone);
    } else {
        std::string cur_city_name = map_list_.at(cur_utm_zone).city;
        if (is_parser_map_info && map_info_.city_name != cur_city_name) {
            ROS_INFO_STREAM("updata map info: utm_zone = " << cur_utm_zone
                                                           << "; city = " << cur_city_name
                                                           << "; launch proj = " << map_info_.city_name);
            std::string map_config = params_.map_path + cur_city_name + "/" + cur_city_name + "_config.yaml";
            struct stat dir_info;
            if (!(stat(params_.map_path.c_str(), &dir_info) == 0 && (dir_info.st_mode & S_IFDIR))) {
                ROS_ERROR_STREAM("Fail to get map directory. path = " << map_config);
            }
            Rs_YAMLReader yamlReader(map_config);
            map_info_ = yamlReader.getValue<mapDefine>();

            img_path_ = params_.map_path + map_info_.mappath;
        }
    }

    local_current_ = local_current;
    img_load_cv_.notify_one();
}

void RoiFilter::perception(const LidarFrameMsg::Ptr &msg_ptr) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto &cloud_ptr = msg_ptr->scan_ptr;
    auto &valid_indices = msg_ptr->valid_indices;
    auto &roifiltered_indices = msg_ptr->roifiltered_indices;
    auto &out_map_indices = msg_ptr->out_map_indices;

    roifiltered_indices.reserve(valid_indices.size());
    if (!params_.enable || valid_img_loaded_flag_ == false) {
        roifiltered_indices = valid_indices;
        out_map_indices = valid_indices;
        msg_ptr->msg_pub_flag = true;
        msg_ptr->is_roifilter_processed = false;
        if (!params_.enable) {
            ROS_DEBUG_STREAM(" RoiFilter disabled.");
        } else {
            ROS_DEBUG_STREAM(" RoiFilter::perception Waiting png load.");
        }
        return;
    }

    TRY_CATCH
    {
        double utm_center_x = local_current_.position().x();
        double utm_center_y = local_current_.position().y();
        int mat_xid = static_cast<int>((utm_center_x - map_info_.utm_start_X) / map_info_.tilesize);
        int mat_yid = static_cast<int>((utm_center_y - map_info_.utm_start_Y) / map_info_.tilesize);
        int mat_row = static_cast<int>((utm_center_y - map_info_.utm_start_Y - mat_yid * map_info_.tilesize) / map_info_.gridsize);
        int mat_col = static_cast<int>((utm_center_x - map_info_.utm_start_X - mat_xid * map_info_.tilesize) / map_info_.gridsize);
        ROS_DEBUG_STREAM("Mat info: (x, y) = (" << mat_xid << ", " << mat_yid << "); (row, col) = (" << mat_row << ", " << mat_col << ")");

        double matrix_center_row_id = (mat_yid - idy_min_) * 500 + mat_row;
        double matrix_center_col_id = (mat_xid - idx_min_) * 500 + mat_col;
        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd((M_PI_2 - local_current_.yaw()), Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
        Eigen::Transform<double, 3, Eigen::Affine> t = Eigen::Translation3d(matrix_center_row_id, matrix_center_col_id, 0) *
                                                       Eigen::Quaternion<double>(q.w(), q.x(), q.y(), q.z());

        TicToc timer("perception/roi filter/road map");
        int type_ground = 0 << 5;
        int type_road = 1 << 5;
        int type_greenbelt = 2 << 5;
        int type_splitwall = 3 << 5;
        msg_ptr->roadmap.clear();
        msg_ptr->roadmap.reserve(msg_ptr->grid_map_ptr->pt_cell_id_vec.size());
        msg_ptr->road_map_mat_info = road_map_mat;
        msg_ptr->t_transform = t;
        for (const auto &cell_id : msg_ptr->grid_map_ptr->pt_cell_id_vec) {
            // center cell
            CellInfoPtr cell_info_ptr = msg_ptr->grid_map_ptr->getCellInfo(cell_id);
            if (cell_info_ptr == nullptr || !cell_info_ptr->cart_valid_flag_) continue;

            double rx = cell_info_ptr->local_x_;
            double ry = cell_info_ptr->local_y_;
            Eigen::Vector3d center_road_map_index = t * Eigen::Vector3d(rx / params_.unit_size, ry / params_.unit_size, 0);
            int center_matrix_row_id = static_cast<int>(center_road_map_index.x());
            int center_matrix_col_id = static_cast<int>(center_road_map_index.y());
            if (center_matrix_row_id < 0 || center_matrix_row_id > road_map_mat.rows - 1 
                || center_matrix_col_id < 0 || center_matrix_col_id > road_map_mat.cols - 1)
                continue;

            int road_center_cell_type = road_map_mat.at<uchar>(center_matrix_row_id, center_matrix_col_id);

            int final_road_type = 0;
            for (int r = -1; r < 2; r += 2) {
                for (int c = -1; c < 2; c += 2) {
                    if (center_matrix_row_id + r < 0) continue;
                    if (center_matrix_col_id + c < 0) continue;
                    if (center_matrix_row_id + r > road_map_mat.rows - 1) continue;
                    if (center_matrix_col_id + c > road_map_mat.cols - 1) continue;

                    int temp_cell_type = road_map_mat.at<uchar>(center_matrix_row_id + r, center_matrix_col_id + c);
                    if (road_center_cell_type == type_road) {
                        if (temp_cell_type != type_road) final_road_type = 10;
                    } else if (road_center_cell_type == type_greenbelt) {
                        if (temp_cell_type != type_greenbelt) final_road_type = 11;
                    } else if (road_center_cell_type == type_splitwall) {
                        if (temp_cell_type != type_splitwall) final_road_type = 12;
                    }

                    if (final_road_type > 0) break;
                    else
                        final_road_type = road_center_cell_type >> 5;
                }
            }

            if (final_road_type == params_.show_element_value)
                msg_ptr->roadmap.emplace_back(rx, ry);

            cell_info_ptr->cell_type_ = final_road_type;
            if (final_road_type == 1 || (params_.roadside_valid_flag && final_road_type == 10)) {
                msg_ptr->grid_map_ptr->roi_cell_id_vec.emplace_back(cell_id);
                // calculator UTM position
                double utm_x, utm_y;
                int road_map_position_x = center_matrix_col_id / 500 + idx_min_ + center_matrix_col_id % 500;
                int road_map_position_y = center_matrix_row_id / 500 + idy_min_ + center_matrix_row_id % 500;
                idxyToUtm(road_map_position_x, road_map_position_y, utm_x, utm_y);
                
                cell_info_ptr->global_x_ = utm_x;
                cell_info_ptr->global_y_ = utm_y;
                cell_info_ptr->cart_roi_flag_ = true;

                for (auto &vi : cell_info_ptr->points_indices_) {
                    PointT &pt = cloud_ptr->points.at(vi);
                    if (pt.z - cell_info_ptr->cart_highest_value_ > 0) {
                        cell_info_ptr->highest_indices_ = vi;
                        cell_info_ptr->cart_highest_value_ = pt.z;
                    }
                    if (pt.z - cell_info_ptr->cart_lowest_value_ < 0) {
                        cell_info_ptr->lowest_indices_ = vi;
                        cell_info_ptr->cart_lowest_value_ = pt.z;
                    }
                }

                roifiltered_indices.insert(roifiltered_indices.end(), cell_info_ptr->points_indices_.begin(), cell_info_ptr->points_indices_.end());
            } else {
                msg_ptr->grid_map_ptr->valid_cell_id_vec.at(cell_id) = false;
                out_map_indices.insert(out_map_indices.end(), cell_info_ptr->points_indices_.begin(), cell_info_ptr->points_indices_.end());
            }
        }
        valid_img_loaded_flag_ = false;

        msg_ptr->msg_pub_flag = true;
        msg_ptr->is_roifilter_processed = true;
    }     
    END_TRY_CATCH
}

void RoiFilter::idxyToUtm(const int x_id, const int y_id, double &center_utm_x, double &center_utm_y) {
    center_utm_x = (x_id + 0.5) * map_info_.tilesize + map_info_.utm_start_X;
    center_utm_y = (y_id + 0.5) * map_info_.tilesize + map_info_.utm_start_Y;
}

void RoiFilter::imreadPng(std::string png_path, cv::Mat &img) {
    img = cv::imread(png_path, cv::IMREAD_GRAYSCALE);
}

void RoiFilter::imreadOnes(cv::Mat &img, unsigned char value) {
    img = cv::Mat::ones(500, 500, CV_8UC1) * value;
}

bool RoiFilter::checkAndLoadImgs(const std::vector<std::pair<int, int>> &idx_idy_vec) {
    TRY_CATCH
    std::vector<std::map<std::pair<int, int>, cv::Mat>::iterator> delete_iterators;
    for (auto img_it = valid_img_map_.begin(); img_it != valid_img_map_.end(); img_it++) {
        auto it = std::find(idx_idy_vec.begin(), idx_idy_vec.end(), img_it->first);
        if (it == idx_idy_vec.end())
            delete_iterators.emplace_back(img_it);
    }
    for (auto &it : delete_iterators) {
        valid_img_map_.erase(it);
    }

    int idx_min = INT_MAX;
    int idx_max = INT_MIN;
    int idy_min = INT_MAX;
    int idy_max = INT_MIN;
    bool cat_img_flag = false;
    for (auto id : idx_idy_vec) {
        if (valid_img_map_.find(id) == valid_img_map_.end()) {
            cv::Mat img;
            std::string png_name = img_path_ + "/" + std::to_string(id.first) + "_" + std::to_string(id.second) + ".png";
            if (access(png_name.c_str(), F_OK) == 0) {
                imreadPng(png_name, img);
            } else {
                unsigned char r_1 = 1 << 5; // 1:road
                imreadOnes(img, r_1);
            }
            valid_img_map_.emplace(id, img);
            cat_img_flag = true;
        }
        idx_min = idx_min > id.first ? id.first : idx_min;
        idx_max = idx_max < id.first ? id.first : idx_max;
        idy_min = idy_min > id.second ? id.second : idy_min;
        idy_max = idy_max < id.second ? id.second : idy_max;
    }

    cv::Mat temp_img;
    if (cat_img_flag) {
        for (size_t y = idy_min; y < idy_max + 1; y++) {
            cv::Mat h_concat_img;
            std::vector<cv::Mat> imgs;
            for (size_t x = idx_min; x < idx_max + 1; x++) {
                imgs.push_back(valid_img_map_.at(std::make_pair(x, y)));
            }
            hconcat(imgs, h_concat_img);
            if (temp_img.cols)
                vconcat(temp_img, h_concat_img, temp_img);
            else
                temp_img = h_concat_img;
        }

        idx_min_ = idx_min;
        idy_min_ = idy_min;

        std::lock_guard<std::mutex> lock(mutex_);
        road_map_mat = temp_img;
    }
    return true;
    END_TRY_CATCH

    return false;
}

void RoiFilter::getIdxIdyList(double utm_x, double utm_y, double search_range,
                              std::vector<std::pair<int, int>> &idx_idy_vec) {
    int tile_srt_id_x = (utm_x - search_range - map_info_.utm_start_X) / map_info_.tilesize;
    int tile_end_id_x = (utm_x + search_range - map_info_.utm_start_X) / map_info_.tilesize + 1;
    int tile_srt_id_y = (utm_y - search_range - map_info_.utm_start_Y) / map_info_.tilesize;
    int tile_end_id_y = (utm_y + search_range - map_info_.utm_start_Y) / map_info_.tilesize + 1;
    for (int x_id = tile_srt_id_x; x_id < tile_end_id_x; x_id++) {
        for (int y_id = tile_srt_id_y; y_id < tile_end_id_y; y_id++) {
            idx_idy_vec.emplace_back(x_id, y_id);
        }
    }
}

void RoiFilter::pngLoadThread() {
    while (ros::ok()) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            img_load_cv_.wait(lock);
        }
        if (exit_flag_)
            break;

        if (local_current_.header().seq() < 1) {
            ROS_WARN_STREAM("Invalid Localization;");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }

        // 1.center:lonlat --> utm
        double center_utm_x = local_current_.position().x();
        double center_utm_y = local_current_.position().y();
        { // check lonlat utm issame
            frame_transform frame_utm2latlon;
            const auto current_lon_rad = local_current_.longitude() * DEG_TO_RAD;
            const auto current_lat_rad = local_current_.latitude() * DEG_TO_RAD;
            double convert_center_utm_x = 0;
            double convert_center_utm_y = 0;
            frame_utm2latlon.LatlonToUtmXY(current_lon_rad, current_lat_rad, convert_center_utm_x, convert_center_utm_y);
            double dx = convert_center_utm_x - center_utm_x;
            double dy = convert_center_utm_y - center_utm_y;
            double dist = sqrt(dx * dx + dy * dy);
            if (dist > 0.1) {
                ROS_WARN_STREAM(HDEBUG_R << "HCheck:dist error.dist=" << dist << ",dx=" << dx << ",dy=" << dy << ",convert_utm_x=" << convert_center_utm_x << ",convert_utm_y=" << convert_center_utm_y);
            }
        }
        std::vector<std::pair<int, int>> cur_idx_idy_vec;
        getIdxIdyList(center_utm_x, center_utm_y, params_.search_range_curload_, cur_idx_idy_vec);

        valid_img_loaded_flag_ = checkAndLoadImgs(cur_idx_idy_vec);
    }
}

} // namespace robosense
