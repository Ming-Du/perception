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
#include "build_object/object_build.h"
#include <ros/ros.h>
namespace robosense {

bool RsObjectBuilder::buildObject(const PointCloud::Ptr& cloud_ptr,
                                  const Object::Ptr& obj_ptr,bool is_debugobj) {
    // step1. 基本信息计算
    if (!baseBuild(cloud_ptr, obj_ptr,is_debugobj)) {
        if(is_debugobj){
            ROS_DEBUG_STREAM(HDEBUG_B<<"delete in <baseBuild>");
        }
        return false;
    }
    // std::cout<<"  baseBuild  ";
    // step2. 使用几何方法计算朝向
    if(!dirCalculationByPolygon(obj_ptr->polygons, obj_ptr->direction)){
        if(is_debugobj){
            ROS_DEBUG_STREAM(HDEBUG_B<<"delete in <dirCalculationByPolygon>");
        }
        return false;
    }
    // step3. 计算几何信息
    geoCalculationByDir(cloud_ptr, obj_ptr);
    obj_ptr->check();
    return true;
}

bool RsObjectBuilder::buildObjectByPolygon(std::vector<Eigen::Vector3d> &multi_polygon,
                                  const Object::Ptr& obj_ptr,bool is_debugobj) {
    // step1. 基本信息计算
    ContourHull::ContourInfos contour_infos;
    if (multi_polygon.size() < 3) {
        return false;
    }
    contour_ptr_->buildByPolygon(multi_polygon, contour_infos, false);
    obj_ptr->polygons = contour_infos.contours;
    // step2. 使用几何方法计算朝向
    if(!dirCalculationByPolygon(obj_ptr->polygons, obj_ptr->direction)){
        if(is_debugobj){
            ROS_DEBUG_STREAM(HDEBUG_B<<"delete in <dirCalculationByPolygon>");
        }
        return false;
    }
    // step3. 计算几何信息
    const auto& polygon = obj_ptr->polygons;
    auto& direction = obj_ptr->direction;
    auto& geo_center = obj_ptr->center;
    auto& geo_size = obj_ptr->size;
    double max_z, min_z;
    max_z = 1.5;
    min_z = 0;
    calcuGeoAlongDir(polygon, direction, geo_size, geo_center, false);
    geo_size.z() = max_z - min_z;
    geo_center.z() = (min_z + max_z) / 2.;
    obj_ptr->check();
    return true;
}

bool RsObjectBuilder::baseBuild(const PointCloud::Ptr& cloud_ptr, const Object::Ptr& obj_ptr,bool is_debugobj) {
    if (obj_ptr == nullptr) {
        return false;
    }
    if (cloud_ptr == nullptr) {
        return false;
    }
    if (obj_ptr->cloud_indices.empty()) {
        return false;
    }
    // step1. 使用contour_hull计算基本信息
    ContourHull::ContourInfos contour_infos;
    // step1.1 如果是ai出来的目标,只需要计算凸包,如果不是ai出来的目标,则需要计算凹凸包
    if (ModeType::FOCUS == obj_ptr->mode) {
        contour_ptr_->buildSimple(cloud_ptr, obj_ptr->cloud_indices, contour_infos);
    } else {
        contour_ptr_->build(cloud_ptr, obj_ptr->cloud_indices, contour_infos,is_debugobj);
    }
    // if (contour_ptr_->is_four_quadrant_)
    //     return false;
    obj_ptr->polygons = contour_infos.contours;
    
    return true;
}

void RsObjectBuilder::geoCalculationByDir(const PointCloud::Ptr& cloud_ptr,
                                          const Object::Ptr& obj_ptr) {
    const auto& polygon = obj_ptr->polygons;
    auto& direction = obj_ptr->direction;
    const auto& cloud_indices = obj_ptr->cloud_indices;
    auto& geo_center = obj_ptr->center;
    auto& geo_size = obj_ptr->size;

    // step1. 计算几何中心以及几何大小
    double max_z, min_z;
    calcuMinMaxZ(cloud_ptr, cloud_indices, max_z, min_z);
    // calcuGeoAlongDir(polygon, direction, geo_size, geo_center, true);
    calcuGeoAlongDir(polygon, direction, geo_size, geo_center, false);
    geo_size.z() = max_z - min_z;
    geo_center.z() = (min_z + max_z) / 2.;
}

bool RsObjectBuilder::dirCalculationByPolygon(const std::vector<Eigen::Vector3d> &polygons,
                                              Eigen::Vector3d& dir) {
    // step1. 处理异常情况,小于3个点的polygon不需要计算朝向
    dir = Eigen::Vector3d(1., 0., 0.);
    if (polygons.size() < 3) {
        return false;
    }

    // step2. 计算polygon的周长以及最长的边
    double total_polygon_len = 0;
    double max_edge_len = 0;
    size_t max_len_idx = 0;
    for (size_t i = 0; i < polygons.size() - 1; ++i) {
        double dist = (polygons[i] - polygons[i + 1]).norm();
        total_polygon_len += dist;
        if (dist > max_edge_len) {
            max_edge_len = dist;
            max_len_idx = i;
        }
    }

    double dist = (polygons[polygons.size()-1] - polygons[0]).norm();
    total_polygon_len += dist;
    if (dist > max_edge_len) {
        max_edge_len = dist;
        max_len_idx = polygons.size();
    }

    // step3. 通过计算最小面积的那个边,得到polygon的朝向
    double min_area = std::numeric_limits<double>::max();
    for (size_t i = 0; i < polygons.size() - 1; ++i) {
        Eigen::Vector3d ray = (polygons[i] - polygons[i + 1]);
        double ray_len = ray.norm();
        ray.normalize();

        // if (i == max_len_idx) {
        //     dir = ray;
        // }

        if (i == max_len_idx ||
            (ray_len / max_edge_len) > 0.7f ||
            ray_len > total_polygon_len * 0.25f) {
            Eigen::Vector3d center, size;
            calcuGeoAlongDir(polygons, ray, size, center, false);
            double tmp_area = size.x() * size.y();
            if (min_area > tmp_area) {
                min_area = tmp_area;
                dir = ray;
            }
        }
    }
    return true;
}

void RsObjectBuilder::calcuGeoAlongDir(const std::vector<Eigen::Vector3d> &polygon2D,
                                       Eigen::Vector3d &dir, Eigen::Vector3d &size,
                                       Eigen::Vector3d &center, bool fix_dir) {

    // Add 1, Check all polygon points is out block area
    bool polygon_in_block_range = false;

    for (Eigen::Vector3d p : polygon2D) {
        if (polygon_block_area_.inRange2D(p.x(), p.y()))
            polygon_in_block_range = true;
    }

    Eigen::Vector3d ortho_dir(-dir.y(), dir.x(), 0.);
//    ortho_dir.normalize();

    Eigen::Vector3d min_pt(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.);
    Eigen::Vector3d max_pt(-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(), 0.);

    std::vector<double> loc_pt_x(polygon2D.size());
    std::vector<double> loc_pt_y(polygon2D.size());

    for (size_t i = 0; i < polygon2D.size(); i++) {
        const auto &pt = polygon2D[i];
        loc_pt_x[i] = pt.dot(dir);
        loc_pt_y[i] = pt.dot(ortho_dir);
    }

    auto max_idx = std::max_element(loc_pt_x.begin(), loc_pt_x.end());
    auto min_idx = std::min_element(loc_pt_x.begin(), loc_pt_x.end());
    min_pt.x() = (*min_idx);
    max_pt.x() = (*max_idx);

    max_idx = std::max_element(loc_pt_y.begin(), loc_pt_y.end());
    min_idx = std::min_element(loc_pt_y.begin(), loc_pt_y.end());
    min_pt.y() = (*min_idx);
    max_pt.y() = (*max_idx);

    size = max_pt - min_pt;
    center = dir * ((max_pt.x() + min_pt.x()) * 0.5f) + ortho_dir * ((max_pt.y() + min_pt.y()) * 0.5f);

    if (!fix_dir)
        return;

    if (std::max(size.x(), size.y()) / std::min(size.x(), size.y()) > 20.0)
        return;

    // Add 2, check corners is out block area
    std::vector<Eigen::Vector3d> corners;
    corners.resize(4);

    Eigen::Vector2d dir2d(dir.x(), dir.y());
    Eigen::Vector2d ortho_dir2d(-dir.y(), dir.x());

    Eigen::Vector3d z_dir = Eigen::Vector3d(0, 0, 1);
    corners[0] = center + dir * size.x() * 0.5f + ortho_dir * size.y() * 0.5f;
    corners[1] = center - dir * size.x() * 0.5f + ortho_dir * size.y() * 0.5f;
    corners[2] = center - dir * size.x() * 0.5f - ortho_dir * size.y() * 0.5f;
    corners[3] = center + dir * size.x() * 0.5f - ortho_dir * size.y() * 0.5f;

    bool corner_in_block_range = false;

    for (Eigen::Vector3d c : corners) {
        if (box_block_area_.inRange2D(c.x(), c.y()))
            corner_in_block_range = true;
    }
    // std::cout << "polygon_in_block_range = " << polygon_in_block_range << ", corner_in_block_range = " << corner_in_block_range << std::endl;
    // Add 3, if all polygon out but corner in, fix dir!
    if (polygon_in_block_range == false && corner_in_block_range == true) {
        // std::cout << " => size = " << size.x() << ", " << size.y() << std::endl;
        // std::cout << " => center = " << center.x() << ", " << center.y() << std::endl;
        // std::cout << " => dir = " << dir.x() << ", " << dir.y() << std::endl;
        // std::cout << " => corners = (" << corners[0].x() << ", " << corners[0].y() << ")" << std::endl;
        // std::cout << " => corners = (" << corners[1].x() << ", " << corners[1].y() << ")" << std::endl;
        // std::cout << " => corners = (" << corners[2].x() << ", " << corners[2].y() << ")" << std::endl;
        // std::cout << " => corners = (" << corners[3].x() << ", " << corners[3].y() << ")" << std::endl;
        dir = (size.x() > size.y()) ? Eigen::Vector3d(1., 0., 0.) : Eigen::Vector3d(0., 1., 0.);
    }
}

void RsObjectBuilder::calcuMinMaxZ(const PointCloud::Ptr& cloud_ptr,
                                   const std::vector<int> &cloud_indices,
                                   double &max_z, double &min_z) {
    max_z = std::numeric_limits<double>::min();
    min_z = std::numeric_limits<double>::max();
    for (const auto& pt_idx : cloud_indices) {
        const auto& pt = cloud_ptr->points[pt_idx];
        max_z = max_z > pt.z ? max_z : pt.z;
        min_z = min_z < pt.z ? min_z : pt.z;
    }
}

}   // namespace robosense
