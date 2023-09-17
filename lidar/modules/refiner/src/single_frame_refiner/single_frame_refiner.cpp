#include "single_frame_refiner/single_frame_refiner.h"

namespace robosense {

SingleFrameRefiner::SingleFrameRefiner()
{
    
}

void SingleFrameRefiner::perception(const LidarFrameMsg::Ptr &msg_ptr){
    TicToc timer("perception/refiner/single box refine");
    if(params_.enable_BoxRefine){
        single_box_refine(msg_ptr);
    }
}

void SingleFrameRefiner::init(const RefinerParam &param) {
    params_ = param;
}


void SingleFrameRefiner::single_box_refine(const LidarFrameMsg::Ptr &msg_ptr){
    msg_ptr->shape_indices.clear();
    auto &obj_refine = msg_ptr->objects_refine;
    for (size_t i = 0; i < obj_refine.size(); ++i) {
        auto &obj = obj_refine[i];
        if (obj->is_ai_refine != AiRefine_State::NO_AIREFINE) {
            continue;
        }
        
        const auto &center = obj->center;
        const auto &size = obj->size;
        const auto &direction = obj->direction;
        RotateBox box(center, size, direction);
        std::vector<Eigen::Vector3d> corners;
        box.corners(corners);
        bool needSplit = false;
        std::vector<Eigen::Vector3d> out_point;
        for (int i = 0; i < 4; ++i) {
            obj->corner_point(i, 0) = corners[i].x();
            obj->corner_point(i, 1) = corners[i].y();
        }
        if(obj->type == ObjectType::UNKNOW){
            if(obj->size.x() > 1 && obj->size.y() > 1){
                std::vector<Eigen::Vector2d> bounding_point;
                box_dist =0;
                double box_yaw =0;
                getBoundingInfo(obj, msg_ptr, bounding_point, box_dist, false);
                lShapeBoxFitting(obj, bounding_point, box_yaw);
                obj->direction.x() = cosf(box_yaw);
                obj->direction.y() = sinf(box_yaw);
                obj->size.x() = vertex_pts_[4].x();
                obj->size.y() = vertex_pts_[4].y();
                Eigen::Vector2f edge_1((vertex_pts_[0].x() + vertex_pts_[1].x()) / 2, (vertex_pts_[0].y() + vertex_pts_[1].y()) / 2);
                Eigen::Vector2f edge_2((vertex_pts_[2].x() + vertex_pts_[3].x()) / 2, (vertex_pts_[2].y() + vertex_pts_[3].y()) / 2);
                obj->center.x() = (edge_1.x() + edge_2.x()) / 2;
                obj->center.y() = (edge_1.y() + edge_2.y()) / 2;
                continue;
            }
            else
                continue;
        } 
        for(int i =0; i < obj->polygons.size(); i++){
            Eigen::Vector3d point = obj->polygons[i];
            bool inBox = point_in_polygon(point, obj->corner_point);
            if(!inBox){
                needSplit = true;
                out_point.push_back(point);
            }
        }
        if(!needSplit && obj->type != ObjectType::PED && obj->type != ObjectType::BIC){
            rbInBoxRefine(obj, msg_ptr);
        }
        else if (needSplit){
            rbOutBoxRefine(obj, msg_ptr, out_point);
        }
    }
}

bool SingleFrameRefiner::point_in_polygon(const Eigen::Vector3d &p, Eigen::MatrixXd &poly) {
    int num = poly.rows();
    bool is_inside = false;
    int count = 0;
    for (size_t i = 0; i < num; i++) {
        Eigen::Vector2d p1(poly(i,0) , poly(i,1));
        Eigen::Vector2d p2 (poly(((i + 1) % num) ,0) , poly(((i + 1) % num) ,1));
        // 判断是否在多边形边界上
        if ((p.x() - p1.x()) * (p2.y() - p1.y()) == (p2.x() - p1.x()) * (p.y() - p1.y())
            && std::min(p1.x(), p2.x()) <= p.x() && p.x() <= std::max(p1.x(), p2.x())
            && std::min(p1.y(), p2.y()) <= p.y() && p.y() <= std::max(p1.y(), p2.y())) {
            is_inside = true;
            break;
        }
        if (p1.y() != p2.y() && std::min(p1.y(), p2.y()) < p.y() && p.y() <= std::max(p1.y(), p2.y())) {
            double xinters = (p.y() - p1.y()) * (p2.x() - p1.x()) / (p2.y() - p1.y()) + p1.x();
            if (p.x() == xinters) { // 在多边形的某个顶点上
                is_inside = true;
                break;
            }
            if (p.x() < xinters) count++;
        }
    }
    if (count % 2 == 1) is_inside = !is_inside;
    return is_inside;
}

/*
Eigen::Vector2d SingleFrameRefiner::calc_vector_byparalle(Eigen::Vector2d in_vector, double norm_vector) {
    Eigen::Vector2d res;
    double tmp = sqrt(pow(in_vector.x(), 2) + pow(in_vector.y(), 2));
    res.x() = in_vector.x() * norm_vector / tmp;
    res.y() = in_vector.y() * norm_vector / tmp;
    return res;
}
*/
void SingleFrameRefiner::rbInBoxRefine(Object::Ptr &curr_obj, const LidarFrameMsg::Ptr &msg_ptr) {
    // size do not change, center&dir change
    std::vector<Eigen::Vector2d> bounding_point;
    box_dist = 0;
    double box_yaw = 0;
    bool modify_cart = false;
    if (curr_obj->type == ObjectType::TRUCK || curr_obj->type == ObjectType::BUS)
        modify_cart = true;
    getBoundingInfo(curr_obj, msg_ptr, bounding_point, box_dist, modify_cart);
    if (modify_cart || bounding_point.size() < 3)
        return;
    if (box_dist > dist_th) {
        lShapeBoxFitting(curr_obj, bounding_point, box_yaw);
        if(vertex_pts_.size() < 4){
            return;
        }
        int corner_index = 0;
        double ai_distance = std::numeric_limits<double>::max();
        double rb_distance = std::numeric_limits<double>::max();
        Eigen::Vector2d ai_corner;
        for (int i = 0; i < 4; ++i) {
            std::vector<float> point;
            point.emplace_back(vertex_pts_[i].x());
            point.emplace_back(vertex_pts_[i].y());
            // msg_ptr->shape_indices.emplace_back(point);
            double tmp_dist = sqrtf(pow(curr_obj->corner_point(i, 0), 2) + pow(curr_obj->corner_point(i, 1), 2));
            if (tmp_dist < ai_distance) {
                ai_distance = tmp_dist;
                ai_corner << curr_obj->corner_point(i, 0), curr_obj->corner_point(i, 1);
            }
        }

        for (int i = 0; i < 4; i++) {
            Eigen::Vector2d tmp_diff;
            tmp_diff << (vertex_pts_[i].x() - ai_corner.x()), (vertex_pts_[i].y() - ai_corner.y());
            double tmp_dist = sqrtf(pow(tmp_diff.x(), 2) + pow(tmp_diff.y(), 2));
            if (tmp_dist < rb_distance) {
                rb_distance = tmp_dist;
                corner_index = i;
            }
        }

        //(x,y)单位向量，AB与之平行，B = A + L*(x, y)
        Eigen::Vector2d ai_to_rb;
        ai_to_rb << vertex_pts_[corner_index].x() - ai_corner.x(), vertex_pts_[corner_index].y() - ai_corner.y();
        curr_obj->center.x() += ai_to_rb.x();
        curr_obj->center.y() += ai_to_rb.y();
        std::vector<float> point_;
        point_.emplace_back(ai_corner.x()); // curr_obj->center.x());
        point_.emplace_back(ai_corner.y()); // curr_obj->center.y());
        // msg_ptr->shape_indices.emplace_back(point_);
        // std::cout<<"..box dist.. "<<box_dist<<" "<<curr_obj->center.x()<<" "<<curr_obj->center.y()<<std::endl;
    }
}

void SingleFrameRefiner::rbOutBoxRefine(Object::Ptr &curr_obj, const LidarFrameMsg::Ptr &msg_ptr, const std::vector<Eigen::Vector3d> &out_point) {
    // size change, center change
    std::vector<Eigen::Vector2d> bounding_point;
    box_dist = 0;
    double box_yaw = 0;
    bool modify_cart = false;
    if (curr_obj->type == ObjectType::TRUCK || curr_obj->type == ObjectType::BUS)
        modify_cart = true;
    getBoundingInfo(curr_obj, msg_ptr, bounding_point, box_dist, modify_cart);
    /*
    auto& dir = curr_obj->direction;
    auto& center = curr_obj->center;
    auto& size = curr_obj->size;
    Eigen::Vector3d ortho_dir = Eigen::Vector3d(-dir.y(), dir.x(), 0);
    Eigen::Vector3d tmp_dir_left;
    Eigen::Vector3d tmp_dir_right;
    Eigen::Vector3d tmp_dir_top;
    Eigen::Vector3d tmp_dir_down;
    double max_dist_left;
    double max_dist_right;
    double max_dist_top;
    double max_dist_down;
    for (int index=0; index < out_point.size(); index++ ) {
        tmp_dir_left = out_point[index] - (center + ortho_dir * size.y() * 0.5);
        tmp_dir_right = out_point[index] - (center - ortho_dir * size.y() * 0.5);
        double dist_left = fabs(tmp_dir_left.head(2).dot(ortho_dir.head(2)));
        double dist_right = fabs(tmp_dir_right.head(2).dot(ortho_dir.head(2)));
        if(dist_left < dist_right){
            if (dist_left > max_dist_left) {
                max_dist_left = dist_left;
            }
        }
        else{
            if (dist_right > max_dist_right) {
                max_dist_right = dist_right;
            }
        }
        tmp_dir_top = out_point[index] - (center + dir * size.x() * 0.5);
        tmp_dir_down = out_point[index] - (center - dir * size.x() * 0.5);
        double dist_top =  fabs(tmp_dir_top.head(2).dot(dir.head(2)));
        double dist_down =  fabs(tmp_dir_down.head(2).dot(dir.head(2)));
        if(dist_top < dist_down){
            if (dist_top > max_dist_top) {
                max_dist_top = dist_top;
            }
        }
        else{
            if (dist_down > max_dist_down) {
                max_dist_down = dist_down;
            }
        }

    }
    size.y() = size.y() + max_dist_left + max_dist_right;
    size.x() = size.x() + max_dist_down + max_dist_top;
    */
}

void SingleFrameRefiner::getBoundingInfo(Object::Ptr &curr_obj, const LidarFrameMsg::Ptr &msg_ptr, std::vector<Eigen::Vector2d> &bounding_point, double &box_dist, bool modify_cart) {
    auto &bird_view_ = msg_ptr->grid_map_ptr;
    auto &dir = curr_obj->direction;
    auto &center = curr_obj->center;
    auto &size = curr_obj->size;
    auto &valid_cell = msg_ptr->grid_map_ptr->valid_cell_id_vec;
    Eigen::Vector3d ortho_dir = Eigen::Vector3d(-dir.y(), dir.x(), 0);
    int edge_sign = 1;
    double tmp_dist = 0;
    Eigen::Vector3d tmp_dir;
    for (int i = 0; i < 2; i++) {
        double nearest_dist = std::numeric_limits<double>::max();
        for (double h = -0.2 - size.x() * 0.5; h <= size.x() * 0.5 + 0.2; h += 0.2) {
            for (double v = 0; v <= size.y() * 0.5; v += 0.2) {
                Eigen::Vector3d selected_grid = center + dir * h + edge_sign * ortho_dir * (size.y() * 0.5 - v);
                int grid_idx = bird_view_->calCellId(selected_grid.x(), selected_grid.y());
                auto bird_view_cell = bird_view_->getCellInfo(grid_idx);
                if (bird_view_cell != nullptr && bird_view_cell->cell_type_ == 1 && bird_view_cell->cart_bin_flag_ && valid_cell[grid_idx]) {
                    tmp_dir << selected_grid.x(), selected_grid.y(), 0;
                    tmp_dir = tmp_dir - (center + edge_sign * ortho_dir * size.y() * 0.5);
                    if (h > (0.4 - size.x() * 0.5) && h < (size.x() * 0.5 - 0.4)) {
                        box_dist += fabs(tmp_dir.head(2).dot(ortho_dir.head(2)));
                    }
                    Eigen::Vector2d p_bouding = Eigen::Vector2d(selected_grid.x(), selected_grid.y());
                    bounding_point.push_back(p_bouding);
                    std::vector<float> point;
                    point.emplace_back(selected_grid.x());
                    point.emplace_back(selected_grid.y());
                    // msg_ptr->shape_indices.emplace_back(point);

                    tmp_dist = fabs(tmp_dir.head(2).dot(ortho_dir.head(2)));
                    if (tmp_dist < nearest_dist) {
                        nearest_dist = tmp_dist;
                    }
                    if (h > (0.4 - size.x() * 0.5) && h < (size.x() * 0.5 - 0.4))
                        break;
                }
            } // end one line
        }     // end for one edge

        if (nearest_dist < size.y() * 0.4 && modify_cart) {
            size.y() = size.y() - nearest_dist;
            center = center - edge_sign * ortho_dir * nearest_dist * 0.5f;
        }

        edge_sign *= -1;
    }
}

void SingleFrameRefiner::lShapeBoxFitting(Object::Ptr &curr_obj, const std::vector<Eigen::Vector2d> &bounding_point, double &yaw) {
    double box_dist = 0;
    Eigen::MatrixXd Matrix_pts = Eigen::MatrixXd::Zero(bounding_point.size(), 2);
    for (size_t i = 0; i < bounding_point.size(); ++i) {
        Matrix_pts(i, 0) = bounding_point[i].x();
        Matrix_pts(i, 1) = bounding_point[i].y();
    }
    double dtheta = dtheta_deg_for_search_ * M_PI / 180;
    double max_cost = (-1.0) * std::numeric_limits<double>::max();
    int best_degree = std::numeric_limits<int>::max();
    double best_theta = std::numeric_limits<double>::max();
    double min_c1_s = std::numeric_limits<double>::max();
    double max_c1_s = (-1.0) * std::numeric_limits<double>::max();
    double min_c2_s = std::numeric_limits<double>::max();
    double max_c2_s = (-1.0) * std::numeric_limits<double>::max();
    double cost_vec[90];
    omp_lock_t omp_lock;
    omp_init_lock(&omp_lock);

    #pragma omp parallel for num_threads(num_threads)
    for (int k = 0; k < 90; ++k) {
        Eigen::MatrixXd e1 = Eigen::MatrixXd::Zero(1, 2);
        Eigen::MatrixXd e2 = Eigen::MatrixXd::Zero(1, 2);
        int theta = k;
        double cost = std::numeric_limits<double>::min();
        e1(0, 0) = cos_theta[theta]; // cos(theta);
        e1(0, 1) = sin_theta[theta];
        e2(0, 0) = -1 * sin_theta[theta];
        e2(0, 1) = cos_theta[theta];
        Eigen::MatrixXd c1 = Matrix_pts * e1.transpose();
        Eigen::MatrixXd c2 = Matrix_pts * e2.transpose();
        double c1_min = std::numeric_limits<double>::max();
        double c2_min = std::numeric_limits<double>::max();
        double c1_max = INT_MIN;
        double c2_max = INT_MIN;
        cost = calc_nearest_criterion(c1, c2, c1_min, c1_max, c2_min, c2_max);
        cost_vec[theta] = cost;
        if (max_cost < cost) {
            omp_set_lock(&omp_lock);
            min_c1_s = c1_min;
            max_c1_s = c1_max;
            min_c2_s = c2_min;
            max_c2_s = c2_max;
            max_cost = cost;
            best_theta = theta;
            best_degree = k;
            omp_unset_lock(&omp_lock);
        }
        // std::cout<<std::fixed<<"degree "<<theta<<" "<<k<<" "<<cost<<" "<<c1_max<<std::endl;
    }
    if (max_cost > (-1.0) * std::numeric_limits<double>::max() &&
        best_theta < std::numeric_limits<double>::max()) {
        ;
    } else {
        std::cout << "Fit Failed." << std::endl;
    }
    double sin_s = sin_theta[best_degree];
    double cos_s = cos_theta[best_degree];
    a_.clear();
    b_.clear();
    c_.clear();
    if (min_c1_s < std::numeric_limits<double>::max() &&
        min_c2_s < std::numeric_limits<double>::max() &&
        max_c1_s > (-1.0) * std::numeric_limits<double>::max() &&
        max_c2_s > (-1.0) * std::numeric_limits<double>::max()) {
        a_.push_back(cos_s);
        b_.push_back(sin_s);
        c_.push_back(min_c1_s);

        a_.push_back(-sin_s);
        b_.push_back(cos_s);
        c_.push_back(min_c2_s);

        a_.push_back(cos_s);
        b_.push_back(sin_s);
        c_.push_back(max_c1_s);

        a_.push_back(-sin_s);
        b_.push_back(cos_s);
        c_.push_back(max_c2_s);

        calc_rect_contour();
    }
    yaw = best_degree * RS_M_PI / 180;
    for (int i = 0; i < vertex_pts_.size() - 1; ++i) {
        std::vector<float> point;
        point.emplace_back(vertex_pts_[i].x());
        point.emplace_back(vertex_pts_[i].y());
        // msg_ptr->shape_indices.emplace_back(point);
    }
}

double SingleFrameRefiner::calc_nearest_criterion(const Eigen::MatrixXd &c1,
                                                  const Eigen::MatrixXd &c2, double &c1_min, double &c1_max, double &c2_min, double &c2_max) {
    std::vector<double> c1_deep; // c1 N*1
    std::vector<double> c2_deep;
    for (int i = 0; i < c1.rows(); i++) {
        for (int j = 0; j < c1.cols(); j++) {
            if (c1(i, j) < c1_min) {
                c1_min = c1(i, j);
            }
            if (c1(i, j) > c1_max) {
                c1_max = c1(i, j);
            }
            if (c2(i, j) < c2_min) {
                c2_min = c2(i, j);
            }
            if (c2(i, j) > c2_max) {
                c2_max = c2(i, j);
            }
            c1_deep.push_back(c1(i, j));
            c2_deep.push_back(c2(i, j));
        }
    }
    int n_c1 = c1_deep.size();
    int n_c2 = c2_deep.size();
    std::vector<double> d1;
    std::vector<double> d2;
    double beta = 0;
    double d_value[n_c1];
    for (int i = 0; i < n_c1; i++) {
        double temp_1 = std::min(sqrt(pow((c1_max - c1_deep[i]), 2)), sqrt(pow((c1_deep[i] - c1_min), 2)));
        double temp_2 = std::min(sqrt(pow((c2_max - c2_deep[i]), 2)), sqrt(pow((c2_deep[i] - c2_min), 2)));
        double d = std::max(std::min(temp_1, temp_2), min_dist_of_nearest_crit_);
        beta += 1.0 / d;
    }
    return beta;
}

void SingleFrameRefiner::calc_rect_contour() {
    vertex_pts_.clear();
    double top_left_x = 0.0, top_left_y = 0.0;
    calc_cross_point(a_[0], a_[1], b_[0], b_[1], c_[0], c_[1], top_left_x,
                     top_left_y);
    vertex_pts_.push_back(Eigen::Vector2d(top_left_x, top_left_y));

    double top_right_x = 0.0, top_right_y = 0.0;
    calc_cross_point(a_[1], a_[2], b_[1], b_[2], c_[1], c_[2], top_right_x,
                     top_right_y);
    vertex_pts_.push_back(Eigen::Vector2d(top_right_x, top_right_y));

    double bottom_left_x = 0.0, bottom_left_y = 0.0;
    calc_cross_point(a_[2], a_[3], b_[2], b_[3], c_[2], c_[3], bottom_left_x,
                     bottom_left_y);
    vertex_pts_.push_back(Eigen::Vector2d(bottom_left_x, bottom_left_y));

    double bottom_right_x = 0.0, bottom_right_y = 0.0;
    calc_cross_point(a_[3], a_[0], b_[3], b_[0], c_[3], c_[0], bottom_right_x,
                     bottom_right_y);
    vertex_pts_.push_back(Eigen::Vector2d(bottom_right_x, bottom_right_y));
    double w = c_[2] - c_[0];
    double h = c_[3] - c_[1];
    vertex_pts_.push_back(Eigen::Vector2d(w, h));
    return;
}

} // namespace robosense