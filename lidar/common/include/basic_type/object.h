#ifndef RS_SDK_OBJECT_H_
#define RS_SDK_OBJECT_H_
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
#include "basic_type/object_type.h"
#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>
#include <boost/circular_buffer.hpp>
#include "grid_map/grid_map.h"
namespace robosense {

static unsigned int total_count = 0;

struct alignas(16) ObjectState {
    DetectState detect_state = DetectState::DETECT;
    MotionState motion_state = MotionState::NORMAL;
    NoiseState noise_state = NoiseState::NOISE_OBJECT;
};
struct alignas(16) Object {
    using Ptr = std::shared_ptr<Object>;

    Object() {
        if (total_count >= (std::numeric_limits<unsigned int>::max() - 100)) {
            total_count = 0;
        }
        useful = true;
        total_count++;
        this->unique_id = total_count;
        type = ObjectType::UNKNOW;
        filter_type = FilterObjType::VALID;
        mode = ModeType::BSD;
        source = SourceType::NONE;
    }

    Object(size_t vec_size) {
        if (total_count >= (std::numeric_limits<unsigned int>::max() - 100)) {
            total_count = 0;
        }
        useful = true;
        total_count++;
        this->unique_id = total_count;
        type = ObjectType::UNKNOW;
        filter_type = FilterObjType::VALID;
        mode = ModeType::BSD;
        source = SourceType::NONE;
        cloud_indices.reserve(vec_size);
        cell_indices.reserve(vec_size);
    }

    bool useful;
    AiRefine_State is_ai_refine = AiRefine_State::NO_AIREFINE;

    ObjectType type;
    FilterObjType filter_type;

    SourceType source;
    ModeType mode;
    double type_confidence;
    double exist_confidence = 0;
    ObjectState object_state;
    NoiseState noise_state_obj = NoiseState::NOISE_OBJECT;
    double denoise_obj_weight = 0.0;
    double denoise_grid_weight = 0.0;
    NoiseState denoise_state_frame = NoiseState::NOISE_OBJECT;
    std::vector<NoiseState> noise_history;
    GroupType group = GroupType::SINGLE;
    // NoiseType obj_noise_type = NoiseType::IGNORE;

    double heading;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d size = Eigen::Vector3d::Zero();
    Eigen::Vector3d direction = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    Eigen::MatrixXd corner_point = Eigen::MatrixXd::Zero(4, 2);
    Eigen::Vector3d center_point_vehicle = Eigen::Vector3d::Zero();
    AttentionType attention_type;

    int tracker_id = -1;
    int unique_id;
    RoadType status = RoadType::ROAD;

    // TODO: temporary variable for ai refine, delete or complete them in future
    int measure_id = -1;
    bool ismergelist = false;
    /////////////////////////

    Eigen::Vector3d anchor = Eigen::Vector3d::Zero();
    Eigen::Vector3d nearest_point = Eigen::Vector3d::Zero();

    std::vector<int> cloud_indices;
    std::vector<int> cell_indices;
    std::vector<Eigen::Vector3d> polygons;

    double angle_velocity = 0;

    // should consider vehicle or world coordination when use below params
    std::vector<Eigen::Vector3d> box_pts = std::vector<Eigen::Vector3d>(4);

    inline bool compute_bndbox_points(std::vector<Eigen::Vector3d> &pts) const {
        std::vector<Eigen::Vector3d> tmp_pts = std::vector<Eigen::Vector3d>(4);
        double y_heading = tan(heading);
        Eigen::Vector3d oritation(1, y_heading, 0);
        Eigen::Vector2d dir = oritation.head(2).cast<double>();
        dir.normalize();
        Eigen::Vector2d orth_dir(-dir.y(), dir.x());
        Eigen::Vector2d delta_x = dir * size(0) * 0.5;
        Eigen::Vector2d delta_y = orth_dir * size(1) * 0.5;

        tmp_pts[0] << center(0), center(1), 0.0;
        tmp_pts[0].head(2) += (delta_x + delta_y);
        tmp_pts[1] << center(0), center(1), 0.0;
        tmp_pts[1].head(2) += (delta_x - delta_y);
        tmp_pts[2] << center(0), center(1), 0.0;
        tmp_pts[2].head(2) += (-delta_x - delta_y);
        tmp_pts[3] << center(0), center(1), 0.0;
        tmp_pts[3].head(2) += (-delta_x + delta_y);
        // tmp_pts[0][2] -= size.z() * 0.5;
        // tmp_pts[1][2] -= size.z() * 0.5;
        // tmp_pts[2][2] -= size.z() * 0.5;
        // tmp_pts[3][2] -= size.z() * 0.5;
        if (heading > M_PI/2 || heading < -M_PI/2) {
            pts[0] = tmp_pts[2];
            pts[1] = tmp_pts[3];
            pts[2] = tmp_pts[0];
            pts[3] = tmp_pts[1];
        }
        else {
            pts = tmp_pts;
        }

        return true;
    }

    void clone(const Object &obj) {
        *this = obj;
    }

    void check() {
        auto &direction = this->direction;
        double angle = std::atan2(direction.y(), direction.x());
        while (angle > M_PI) {
            angle -= M_PI * 2;
        }
        while (angle <= -M_PI) {
            angle += M_PI * 2;
        }
        if (angle > M_PI / 2.) {
            angle -= M_PI;
        }
        if (angle <= -M_PI / 2.) {
            angle += M_PI;
        }
        direction.x() = cosf(angle);
        direction.y() = sinf(angle);
        direction.z() = 0;
    }

    bool getAllPointsFromCells(const GridMapPtr &grid_map) {
        cloud_indices.clear();
        for (auto ci : cell_indices) {
            CellInfoPtr cellinfo_ptr = grid_map->getCellInfo(ci);
            if (nullptr == cellinfo_ptr) {
                continue;
            }
            cloud_indices.insert(cloud_indices.end(), cellinfo_ptr->points_indices_.begin(), cellinfo_ptr->points_indices_.end());
        }
        if (0 == cloud_indices.size()) {
            return false;
        }
        return true;
    }
    // std::vector<int> Merge_points_by_condi(height_filter? semantic_failter? xx_filter?){
    //     for(cell){
    //         if(cell_info->height || 3D_box.height || ground.height){
    //             cloud_indices.insert(cell->get_points);
    //         }
    //     }
    //     return points
    // }

    // get_boundary_cell_points(){
    //     for(cell){
    //         if(cell.is_ba_cell){
    //             cloud_indices.insert(cell->get_points);
    //         }
    //     }
    // }

    void transform(const Eigen::Matrix4d& trans_mat);
};

} // namespace robosense

#endif // RS_SDK_OBJECT_H_
