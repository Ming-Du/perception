#include "bev_aggregation/bev_aggregation.h"

namespace robosense {

BevAggregation::BevAggregation()
    : rule_object_filter_ptr_(new RuleObjectFilter) 
{

}

void BevAggregation::init(const RefinerParam &param) {
    params_ = param;

    rule_object_filter_ptr_->init(param);
}

void BevAggregation::perception(const LidarFrameMsg::Ptr &msg_ptr) {
    auto &objs_refine = msg_ptr->objects_refine;
    std::map<int, std::map<int, bool>> rb_ais;
    size_t ai_size = msg_ptr->objects_ai.size();
    double dist, dx, dy;

    {
        TicToc timer0("perception/refiner/bev aggregation/rb object filter");
        TRY_CATCH
        rule_object_filter_ptr_->perception(msg_ptr);
        END_TRY_CATCH
    }

    TicToc timer1("perception/refiner/bev aggregation/clipe rb");
    TRY_CATCH
    if (msg_ptr->denoise_enable) {
        for (size_t rb_id = 0; rb_id < msg_ptr->objects_rule.size(); rb_id++) {
            auto rb_obj = msg_ptr->objects_rule.at(rb_id);
            for (size_t ai_id = 0; ai_id < ai_size; ai_id++) {
                auto ai_obj = msg_ptr->objects_ai.at(ai_id);
                dx = rb_obj->center(0) - ai_obj->center(0);
                dy = rb_obj->center(1) - ai_obj->center(1);
                dist = sqrt(dx*dx + dy*dy);
                if (dist > 10) {continue;}
                bool rst = isPolygonIntersection(rb_obj, ai_obj);
                if (rst) {
                    clipeRBbyCellinfo(msg_ptr, rb_obj, ai_obj, rb_id);
                    rb_ais[rb_id][ai_id] = true;
                }
            }
        }
    }
    END_TRY_CATCH
    timer1.print();

    TicToc timer2("perception/refiner/bev aggregation/objects refine");
    TRY_CATCH
    for (auto &obj_ai : msg_ptr->objects_ai) {
        if (msg_ptr->is_roifilter_processed && params_.enable_AIRoiFilter) {
            bool is_roadmap = false;
            int grid_idx = msg_ptr->grid_map_ptr->calCellId(obj_ai->center.x(), obj_ai->center.y());
            if (grid_idx < 0) continue;
            if (msg_ptr->grid_map_ptr->valid_cell_id_vec.at(grid_idx)) {
                auto cell_info_ptr = msg_ptr->grid_map_ptr->getCellInfo(grid_idx);
                if (cell_info_ptr != nullptr && cell_info_ptr->cell_type_ == 1) {
                    is_roadmap = true;
                }
            }
            if (!is_roadmap) {
                auto &dir = obj_ai->direction;
                auto &center = obj_ai->center;
                auto &size = obj_ai->size;
                Eigen::Vector3d ortho_dir = Eigen::Vector3d(-dir.y(), dir.x(), 0);
                int edge_sign = 1;
                Eigen::Vector3d tmp_dir;
                bool obj_filter = true;
                for (int i = 0; i < 2; i++) {
                    for (double v = 0; v <= size.y() * 0.5; v += 0.2) {
                        for (double h = -size.x() * 0.5; h <= size.x() * 0.5; h += 0.2) {
                            if (h > (0.2 - size.x() * 0.5) && h < (size.x() * 0.5 - 0.2) && v > 0.4 && v < (size.y() * 0.5 - 0.2))
                                continue;
                            Eigen::Vector3d selected_grid = center + dir * h + edge_sign * ortho_dir * (size.y() * 0.5 - v);
                            int grid_idx = msg_ptr->grid_map_ptr->calCellId(selected_grid.x(), selected_grid.y());
                            auto cell_info_ptr = msg_ptr->grid_map_ptr->getCellInfo(grid_idx);
                            if (cell_info_ptr != nullptr && cell_info_ptr->cell_type_ == 1 && msg_ptr->grid_map_ptr->valid_cell_id_vec.at(grid_idx)) {
                                obj_filter = false;
                                break;
                            }
                        }
                        if (!obj_filter)
                            break;
                    }
                    if (!obj_filter)
                        break;

                    edge_sign *= -1;
                }
                if (obj_filter && obj_ai->type != ObjectType::PED)
                    continue;
            }
        }
        if(msg_ptr->is_roifilter_processed && params_.enable_VegetTag && obj_ai->type == ObjectType::CAR){
            bool is_roadside = false;
            Eigen::Vector3d center_road_map_index = msg_ptr->t_transform * Eigen::Vector3d(obj_ai->center.x() / 0.2, obj_ai->center.y() / 0.2, 0);
            int center_matrix_row_id = static_cast<int>(center_road_map_index.x());
            int center_matrix_col_id = static_cast<int>(center_road_map_index.y());
            if (center_matrix_row_id < 0 || center_matrix_row_id > msg_ptr->road_map_mat_info.rows - 1 
                || center_matrix_col_id < 0 || center_matrix_col_id > msg_ptr->road_map_mat_info.cols - 1)
                continue;

            int road_center_cell_type = msg_ptr->road_map_mat_info.at<uchar>(center_matrix_row_id, center_matrix_col_id);
            if(road_center_cell_type >> 5 != 1){
                obj_ai->object_state.noise_state = NoiseState::NOISE_FLOWERBEDS;
            }
        }


        Object::Ptr tmp_refine_obj(new Object);
        tmp_refine_obj->useful = true;
        tmp_refine_obj->clone(*obj_ai);
        tmp_refine_obj->source = SourceType::SINGLE_AI;
        objs_refine.push_back(tmp_refine_obj);
    }
    END_TRY_CATCH
    
    Object::Ptr tmp;
    std::map<int, bool> save_map;
    int  ai_size_process = objs_refine.size();
    TRY_CATCH
    for (size_t i = 0; i < msg_ptr->objects_rule.size(); i++) {
        auto rb_obj = msg_ptr->objects_rule.at(i);
        int match_num = 0;
        for (size_t j = 0; j < ai_size_process; j++) {
            auto refine_obj = objs_refine.at(j);
            if (msg_ptr->denoise_enable) {
                if (rb_ais.find(i) != rb_ais.end() && rb_ais.at(i).find(j) != rb_ais.at(i).end()) {
                    tmp = refine_obj;
                    match_num++;
                    save_map[j] = true;
                    // if (match_num > 1) {
                    //     break;
                    // }
                }
            } else {
                dx = rb_obj->center(0) - refine_obj->center(0);
                dy = rb_obj->center(1) - refine_obj->center(1);
                dist = sqrt(dx*dx + dy*dy);
                if (dist > 10) {continue;}
                if (isPolygonIntersection(rb_obj, refine_obj)) {
                    tmp = refine_obj;
                    match_num++;
                    save_map[j] = true;
                    // if (match_num > 1) {
                    //     break;
                    // }
                }
            }
        }

        if (match_num == 0) { // no ai
            Object::Ptr tmp_refine_obj(new Object);
            tmp_refine_obj->useful = true;
            tmp_refine_obj->clone(*rb_obj);
            tmp_refine_obj->source = SourceType::SINGLE_RB;
            objs_refine.push_back(tmp_refine_obj);
        } else if (match_num == 1) { // one ai, one or multi rb
            // add bev info into tmp
            tmp->source = SourceType::AI_RB;
            tmp->cell_indices.insert(tmp->cell_indices.end(), rb_obj->cell_indices.begin(), rb_obj->cell_indices.end());
            tmp->polygons.insert(tmp->polygons.end(), rb_obj->polygons.begin(), rb_obj->polygons.end());
        } else { // one rb, multi ai
            Object::Ptr tmp_refine_obj(new Object);
            tmp_refine_obj->useful = true;
            tmp_refine_obj->clone(*rb_obj);
            tmp_refine_obj->source = SourceType::SINGLE_RB;
            tmp_refine_obj->group = GroupType::GROUP;
            objs_refine.push_back(tmp_refine_obj);
        }
    }
    END_TRY_CATCH
    timer2.print();

    // delete ai objs out of roi (for Ped, add switch, dali keep, other map del)
    // TODO !
    // TODO -> Roadside state-machine, check AI target
    TicToc timer3("perception/refiner/bev aggregation/objects erase");
    TRY_CATCH
    if (!params_.enable_AIRoiFilter) {
        std::vector<int> erase_object;
        for (size_t i = 0; i < ai_size_process; i++) {
            if (save_map.find(i) == save_map.end()) {
                if (objs_refine[i]->type != ObjectType::PED || params_.enable_PedFilter) {
                    erase_object.emplace_back(i);
                }
            }
        }
        if (erase_object.size() != 0) {
            reverse(erase_object.begin(), erase_object.end());
            for (const auto &i : erase_object) {
                objs_refine.erase(objs_refine.begin() + i);
            }
        }
    }
    END_TRY_CATCH
    timer3.print();
}

void BevAggregation::clipeRBbyCellinfo(const LidarFrameMsg::Ptr &msg_ptr, Object::Ptr &rb_obj, Object::Ptr &ai_obj, size_t rb_id) {
    if (!isNeedSplit(msg_ptr, rb_obj, ai_obj)) {
        return;
    }
    Object::Ptr rb_inAIpart(new Object);
    rb_inAIpart->clone(*rb_obj);
    Object::Ptr rb_outAIpart(new Object);
    rb_outAIpart->clone(*rb_obj);
    splitRb(msg_ptr, rb_obj, ai_obj, rb_inAIpart, rb_outAIpart, rb_id);
}

bool BevAggregation::splitRb(const LidarFrameMsg::Ptr &msg_ptr, Object::Ptr &rb_obj, Object::Ptr &ai_obj,
                             Object::Ptr &rb_inAIpart, Object::Ptr &rb_outAIpart, size_t rb_id) {
    GridMapPtr grid_map_ptr = msg_ptr->grid_map_ptr;
    const auto &cell_indices = rb_obj->cell_indices;
    int rb_obj_cellnum = rb_obj->cell_indices.size();
    rb_inAIpart->cell_indices.clear();
    rb_inAIpart->cell_indices.reserve(rb_obj_cellnum);
    rb_inAIpart->polygons.clear();
    rb_inAIpart->polygons.reserve(rb_obj_cellnum);
    rb_outAIpart->cell_indices.clear();
    rb_outAIpart->cell_indices.reserve(rb_obj_cellnum);
    rb_outAIpart->polygons.clear();
    rb_outAIpart->polygons.reserve(rb_obj_cellnum);

    for(auto id : cell_indices){
        CellInfoPtr cell_info_ptr = grid_map_ptr->getCellInfo(id);
        if (isInAibox(cell_info_ptr->local_x_, cell_info_ptr->local_y_, ai_obj)) {
            rb_inAIpart->cell_indices.emplace_back(id);
        } else {
            rb_outAIpart->cell_indices.emplace_back(id);
        }
    }
    int rebuild_num = 0;
    const auto &cloud_ptr = msg_ptr->scan_ptr;
    auto &objects_rule = msg_ptr->objects_rule;
    if (rb_inAIpart->getAllPointsFromCells(grid_map_ptr)) {
        rebuild_num++;
        if (buildClippedObject(cloud_ptr, rb_inAIpart)) {
            static int num = 0;
            objects_rule.at(rb_id) = rb_inAIpart;
            num++;
        }
    }
    if (rb_outAIpart->getAllPointsFromCells(grid_map_ptr)) {
        rebuild_num++;
        if (buildClippedObject(cloud_ptr, rb_outAIpart)) {
            objects_rule.push_back(rb_outAIpart);
        }
    }
}

bool BevAggregation::buildClippedObject(const PointCloud::Ptr &cloud_ptr, const Object::Ptr &obj_ptr) {
    bool is_debugobj = false;
    RsObjectBuilderInitOptions build_option;
    RsObjectBuilder builder(build_option);
    if (builder.buildObject(cloud_ptr, obj_ptr, is_debugobj)) {
        return true;
    } else {
        return false;
    }
}

bool BevAggregation::isNeedSplit(const LidarFrameMsg::Ptr &msg_ptr, Object::Ptr &rb_obj, Object::Ptr &ai_obj) {
    int noises_count = 0;
    int noises_inAI_count = 0;
    int noises_outAI_count = 0;
    for (auto it = rb_obj->cell_indices.begin(); it != rb_obj->cell_indices.end(); ++it) {
        CellInfoPtr cell_info_ptr = msg_ptr->grid_map_ptr->getCellInfo(*it);
        if(cell_info_ptr->noise_cell_id_ < 0) {
            cell_info_ptr->noise_type_ = NoiseType::OBJECT;
            continue;
        }

        cell_info_ptr->noise_type_ = msg_ptr->denoise_grid.noise_type.at(cell_info_ptr->noise_cell_id_);
        if (!isInAibox(cell_info_ptr->local_x_, cell_info_ptr->local_y_, ai_obj)){
            noises_outAI_count++;
            if(cell_info_ptr->noise_type_ == NoiseType::NOISE)
                noises_count++;
        }
        else
            noises_inAI_count++;
    }

    if(noises_outAI_count < 1)
        return false;
    
    double noise_ratio = (double)noises_count / noises_outAI_count;
    double arearatio_inAI = noises_inAI_count * 1.0 / (0.1 + noises_inAI_count + noises_outAI_count);
    if (noise_ratio > 0.1 && arearatio_inAI > 0.1) {
        return true;
    } else {
        return false;
    }
}

bool BevAggregation::isInAibox(double input_x, double input_y, Object::Ptr &ai_obj) {
    const auto &center = ai_obj->center;
    const auto &size = ai_obj->size;
    const auto &direction = ai_obj->direction;
    RotateBox box(center, size, direction);
    std::vector<Eigen::Vector3d> corners;
    box.corners(corners);

    int left_cross = 0, right_cross = 0;
    for (int i = 0; i < 4; ++i) {
        auto &p1 = corners[i];           // 当前节点
        auto &p2 = corners[(i + 1) % 4]; // 下一个节点

        if (p1.y() == p2.y() && std::fabs(input_y - p1.y()) < 0.0001 && input_x < std::max(p1.x(), p2.x()) && input_x > std::min(p1.x(), p2.x()))
            return true;

        if (p1.y() == p2.y()) // p1p2 与 y平行，无交点
            continue;

        if (input_y <= std::min(p1.y(), p2.y())) // 线段在上方，无交点
            continue;

        if (input_y > std::max(p1.y(), p2.y()))
            continue;

        // 从P发射一条水平射线 求交点的 X 坐标 ------原理: ((p2.y-p1.y)/(p2.x-p1.x))=((y-p1.y)/(x-p1.x))
        // 直线k值相等 交点y=p.y
        double x = (input_y - p1.y()) * (p2.x() - p1.x()) / (p2.y() - p1.y()) + p1.x();

        if (std::fabs(x - input_x) < 0.005) {
            return true;
        } else if (x > input_x)
            right_cross++;
        else
            left_cross++;
    }
    // 两边都是单数
    if (right_cross % 2 == 1 && left_cross % 2 == 1) {
        return true;
    } else {
        return false;
    }
}

bool BevAggregation::isPolygonIntersection(Object::Ptr &rb_obj, Object::Ptr &ai_obj) {
    const auto &center = ai_obj->center;
    const auto &size = ai_obj->size;
    const auto &direction = ai_obj->direction;
    RotateBox box(center, size, direction);
    std::vector<Eigen::Vector3d> corners;
    box.corners(corners);
    // 遍历rb的每个顶点
    for (size_t n = 0; n < rb_obj->polygons.size(); n++) {
        auto &input_x = rb_obj->polygons[n][0];
        auto &input_y = rb_obj->polygons[n][1];
        int left_cross = 0, right_cross = 0;
        for (int i = 0; i < 4; ++i) {
            auto &p1 = corners[i];           // 当前节点
            auto &p2 = corners[(i + 1) % 4]; // 下一个节点

            if (p1.y() == p2.y() && std::fabs(input_y - p1.y()) < 0.0001 && input_x < std::max(p1.x(), p2.x()) && input_x > std::min(p1.x(), p2.x()))
                return true;

            if (p1.y() == p2.y()) // p1p2 与 y平行，无交点
                continue;

            if (input_y <= std::min(p1.y(), p2.y())) // 线段在上方，无交点
                continue;

            if (input_y > std::max(p1.y(), p2.y()))
                continue;

            // 从P发射一条水平射线 求交点的 X 坐标 ------原理: ((p2.y-p1.y)/(p2.x-p1.x))=((y-p1.y)/(x-p1.x))
            // 直线k值相等 交点y=p.y
            double x = (input_y - p1.y()) * (p2.x() - p1.x()) / (p2.y() - p1.y()) + p1.x();

            if (std::fabs(x - input_x) < 0.005) {
                return true;
            } else if (x > input_x)
                right_cross++;
            else
                left_cross++;
        }
        // 两边都是单数
        if (right_cross % 2 == 1 && left_cross % 2 == 1) {
            return true;
        }
    }

    // 遍历ai的每个顶点
    for (int i = 0; i < 4; ++i) {
        auto &input_x = corners[i].x();
        auto &input_y = corners[i].y();
        int left_cross = 0, right_cross = 0;
        for (size_t n = 0; n < rb_obj->polygons.size(); n++) {
            auto &p1 = rb_obj->polygons[n];                                 // 当前节点
            auto &p2 = rb_obj->polygons[(n + 1) % rb_obj->polygons.size()]; // 下一个节点

            if (p1.y() == p2.y() && std::fabs(input_y - p1.y()) < 0.0001 && input_x < std::max(p1.x(), p2.x()) && input_x > std::min(p1.x(), p2.x()))
                return true;

            if (p1.y() == p2.y()) // p1p2 与 y平行，无交点
                continue;

            if (input_y <= std::min(p1.y(), p2.y())) // 线段在上方，无交点
                continue;

            if (input_y > std::max(p1.y(), p2.y()))
                continue;

            // 从P发射一条水平射线 求交点的 X 坐标 ------原理: ((p2.y-p1.y)/(p2.x-p1.x))=((y-p1.y)/(x-p1.x))
            // 直线k值相等 交点y=p.y
            double x = (input_y - p1.y()) * (p2.x() - p1.x()) / (p2.y() - p1.y()) + p1.x();

            if (std::fabs(x - input_x) < 0.005) {
                return true;
            } else if (x > input_x)
                right_cross++;
            else
                left_cross++;
        }
        // 两边都是单数
        if (right_cross % 2 == 1 && left_cross % 2 == 1) {
            return true;
        }
    }

    return false;
}

} // namespace robosense