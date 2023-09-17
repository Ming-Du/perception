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
#include "build_object/contour_full.h"

namespace robosense {

ContourHull::ContourHull(const ContourHullOptions& options) {
    options_ = options;
    Simple2DGridOptions grid_filter_options;
    grid_filter_options.leaf = options_.leaf;
    grid_filter_ptr_.reset(new Simple2DGrid(grid_filter_options));

    res_ang_rad_ = options_.res_angle / 180. * RS_M_PI;
}

void ContourHull::buildSimple(const PointCloud::Ptr& cloud_ptr,
                              const std::vector<int>& pc_indices, ContourInfos& contour) {
    // std::vector<int> voxel_indice;
    // // step1. 进行栅格化,对点云进行上采样
    // grid_filter_ptr_->filter(cloud_ptr, pc_indices, voxel_indice);
    // // step2. 对栅格化的点计算polar信息
    // std::vector<PolarData> polar_vec(voxel_indice.size());
    // for (size_t i = 0; i < voxel_indice.size(); ++i) {
    //     const auto& pt_idx = voxel_indice[i];
    //     const auto& pt = cloud_ptr->points[pt_idx];
    //     auto& polar = polar_vec[i];
    //     polar.x = pt.x;
    //     polar.y = pt.y;
    //     polar.range = std::sqrt(pt.x * pt.x + pt.y * pt.y);
    //     polar.hori = std::atan2(pt.y, pt.x);
    // }
    // // 使用polar信息计算轮廓信息
    // build(polar_vec, contour, true);
}

void ContourHull::build(const PointCloud::Ptr& cloud_ptr,
                        const std::vector<int>& pc_indices, ContourInfos& contour,bool is_debugobj) {
    std::vector<int> voxel_indice;
    // step1. 进行栅格化,对点云进行上采样
    grid_filter_ptr_->filter(cloud_ptr, pc_indices, voxel_indice,is_debugobj);
    // step2. 计算上采样之后的点的polar信息
    // std::cout<<"voxel_indice.size():"<< voxel_indice.size()<<std::endl;
    std::vector<PolarData> polar_vec(voxel_indice.size());
    for (size_t i = 0; i < voxel_indice.size(); ++i) {
        const auto& pt_idx = voxel_indice[i];
        const auto& pt = cloud_ptr->points[pt_idx];
        auto& polar = polar_vec[i];
        polar.x = pt.x;
        polar.y = pt.y;
        polar.range = std::sqrt(pt.x * pt.x + pt.y * pt.y);
        polar.hori = std::atan2(pt.y, pt.x);
    }
    // step3. 使用polar信息来计算轮廓
    build(polar_vec, contour, true,is_debugobj);
}

void ContourHull::buildByPolygon(std::vector<Eigen::Vector3d> &multi_polygon, ContourInfos& contour,bool is_debugobj) {
    std::vector<PolarData> polar_vec(multi_polygon.size());
    for (size_t i = 0; i < multi_polygon.size(); ++i) {
        auto& polar = polar_vec[i];
        polar.x = multi_polygon[i](0);
        polar.y = multi_polygon[i](1);
        polar.range = std::sqrt(polar.x * polar.x + polar.y * polar.y);
        polar.hori = std::atan2(polar.y, polar.x);
    }
    // step3. 使用polar信息来计算轮廓
    build(polar_vec, contour, true,is_debugobj);
}

void ContourHull::build(const std::vector<PolarData>& in_data, ContourInfos& contour, const bool& simple, bool is_debugobj) {
    std::vector<PolarData> in_tmp_data = in_data;
    //===================================================================
    // @brief 通过极坐标点来计算目标的轮廓信息,如果是简单模式,则轮廓为凸包,
    // 如果不是简单模式,则轮廓为面向lidar的一面是凹包,背向lidar的一面是凸包的轮廓点
    //===================================================================

    // step1. 处理异常情况,包括小于三个点,或者是四象限点集,以及奇异点集的情况
    // 如果点数小于三个点,为异常情况odd_situation
    // 如果是四象限点,为异常情况odd_situation
    // 如果是simple和odd_dituation的情况,则直接计算好凸包,完事
    // 否则,则需要计算凹凸包轮廓
    bool odd_situation = false;
    if (static_cast<int>(in_tmp_data.size()) < 3) {
        odd_situation = true;
    }
    const auto& quadrant_type = isFourQuadrant(in_tmp_data);
    if (QuadrantPtsType::FOURQUA == quadrant_type) {
        odd_situation = true;
        is_four_quadrant_ = true;
    } else if (QuadrantPtsType::ODD == quadrant_type) {
        // 如果是奇异点集,则将角度转换到非奇异点集区域
        for (auto& polar : in_tmp_data) {
            polar.hori += RS_M_PI;
            if (polar.hori >= RS_M_PI) {
                polar.hori -= RS_M_PI * 2;
            }
        }
    }
    // step2. 计算凸包
    std::vector<PolarData> polygon;
    convexHull(in_tmp_data, polygon,is_debugobj);
    // step3. 如果是odd以及简单模式,则已完成计算,退出
    // 简单模式则轮廓直接为凸包
    if (odd_situation || simple) {
        // if (odd_situation)
        //     return;
        contour.contours.resize(polygon.size());
        for (size_t i = 0; i < polygon.size(); ++i) {
            contour.contours[i].x() = polygon[i].x;
            contour.contours[i].y() = polygon[i].y;
            contour.contours[i].z() = 0;
        }
        comLrn(polygon, contour.l_pt_idx, contour.r_pt_idx, contour.nearest_pt_idx);

        return;
    }
    // step4. 计算轮廓,背向lidar的点集为凸包点,面向lidar的点集为轮廓点
    // step4.1 构建sector_vec,通过极坐标采样得到面向lidar的最近点
    int l_pt_idx = 0;
    int r_pt_idx = 0;
    int neareast_pt_idx = 0;
    comLrn(in_tmp_data, l_pt_idx, r_pt_idx, neareast_pt_idx);
    double max_hori = in_tmp_data[r_pt_idx].hori;
    double min_hori = in_tmp_data[l_pt_idx].hori;
    int sector_num = (max_hori - min_hori) / res_ang_rad_ + 1;
    std::vector<PolarData> sector_vec(sector_num);
    for (auto& sector : sector_vec) {
        sector.range = -1;
    }
    for (size_t i = 0; i < in_tmp_data.size(); ++i) {
        const auto& tmp_data = in_tmp_data[i];
        int idx = (tmp_data.hori - min_hori) / res_ang_rad_;
        auto& sector = sector_vec[idx];
        if (sector.range < 0) {
            sector = tmp_data;
        } else {
            if (tmp_data.range < sector.range) {
                sector = tmp_data;
            }
        }
    }
    // step4.3 refine sector_vec,通过driveable_width阈值去除尖刺,得到一个相对平滑的面向lidar的面
    bool flag = true;
    while (flag) {
        flag = false;
        std::vector<bool> travel(sector_num, false);
        for (size_t i = 1; i < sector_vec.size() - 1; ++i) {
            if (sector_vec[i].range < 0 || travel[i]) {
                continue;
            }
            travel[i] = true;
            int l_idx = i - 1;
            int r_idx = i + 1;
            for (size_t j = 0; j < sector_vec.size(); ++j) {
                if (sector_vec[l_idx].range >= 0 || l_idx < 0) {
                    break;
                }
                l_idx--;
            }
            for (size_t j = 0; j < sector_vec.size(); ++j) {
                if (sector_vec[r_idx].range >= 0 || r_idx >= sector_num) {
                    break;
                }
                r_idx++;
            }
            if (l_idx < 0 || r_idx >= sector_num) {
                continue;
            }
            if (sector_vec[l_idx].range <= sector_vec[i].range && sector_vec[r_idx].range <= sector_vec[i].range) {
                double range = std::max(sector_vec[l_idx].range, sector_vec[r_idx].range);
                double ang_rad = sector_vec[r_idx].hori - sector_vec[l_idx].hori;
                double tmp = range * std::sin(ang_rad / 2.) * 2.;
                if (tmp < options_.driveable_width) {
                    sector_vec[i].range = -1;
                    flag = true;
                }
            }
            i = r_idx - 1;
        }
    }
    // step4.4 将凸包点集和轮廓点集合成在一起
    std::vector<PolarData> refernece_pts;
    refernece_pts.reserve(sector_vec.size() + polygon.size());
    for (int i = static_cast<int>(sector_vec.size()) - 1; i >= 0 ; --i) {
        const auto& polar = sector_vec[i];
        if (polar.range < 0) {
            continue;
        }
        refernece_pts.emplace_back(polar);
    }
    comLrn(polygon, l_pt_idx, r_pt_idx, neareast_pt_idx);
    for (size_t i = l_pt_idx; ; ++i) {
        int tmp_idx = i % polygon.size();
        const auto& poly = polygon[tmp_idx];
        refernece_pts.emplace_back(poly);

        if (tmp_idx == r_pt_idx) {
            break;
        }
    }
    refernece_pts.resize(refernece_pts.size());
    // step4.5 简化点集
    RamerDouglasPeucker(refernece_pts, refernece_pts, 0.08);
    comLrn(refernece_pts, contour.l_pt_idx, contour.r_pt_idx, contour.nearest_pt_idx);
    contour.contours.resize(refernece_pts.size());
    for (size_t i = 0; i < refernece_pts.size(); ++i) {
        const auto& refer = refernece_pts[i];
        auto& contr = contour.contours[i];
        contr.x() = refer.x;
        contr.y() = refer.y;
    }
}

void ContourHull::comLrn(const std::vector<PolarData>& polar_vec, int& l_idx, int& r_idx, int& n_idx) {
    double l_ang_rad = RS_M_PI;
    double r_ang_rad = -RS_M_PI;
    double n_range = std::numeric_limits<double>::max();

    for (size_t i = 0; i < polar_vec.size(); ++i) {
        const auto& poly = polar_vec[i];
        if (poly.hori < l_ang_rad) {
            l_ang_rad = poly.hori;
            l_idx = i;
        }
        if (poly.hori > r_ang_rad) {
            r_ang_rad = poly.hori;
            r_idx = i;
        }
        if (poly.range < n_range) {
            n_range = poly.range;
            n_idx = i;
        }
    }
}

double ContourHull::cross(const PolarData &O, const PolarData &A, const PolarData &B) {
    return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

void ContourHull::convexHull(std::vector<PolarData> P, std::vector<PolarData> &polygon,bool is_debugobj) {
    int n = static_cast<int>(P.size()), m = 0;
    if (n < 3) {
        polygon = P;
        return;
    }

    polygon.resize(2 * n);
    // 首先将原始点按照x大小排序
    std::sort(P.begin(), P.end(), [&](const PolarData &lhs, const PolarData &rhs) {
        return (lhs.x < rhs.x || (lhs.x == rhs.x && lhs.y < rhs.y));
    });

    // 从x最小点(也就是最左边点)开始,处理下半部的凸包
    for (int i = 0; i < n; ++i) {
        while (m >= 2 && cross(polygon[m - 2], polygon[m - 1], P[i]) <= 0) {
            m--;
        }
        polygon[m++] = P[i];
    }
    // 计算了一半的凸包之后,到了x的最大点处,再处理上半部的凸包
    for (int i = n - 2, t = m + 1; i >= 0; --i) {
        while (m >= t && cross(polygon[m - 2], polygon[m - 1], P[i]) <= 0) {
            m--;
        }
        polygon[m++] = P[i];
    }
    m--;

    polygon.resize(m);
}

QuadrantPtsType ContourHull::isFourQuadrant(const std::vector<PolarData>& pts) {
    // 四象限标记,0象限:x>0,y>0;1象限:x<0,y>0;2象限:x<0,y<0;3象限:x>0,y<0
    std::vector<bool> four_quadrant(4, false);
    for (const auto& pt : pts) {
        const auto& x = pt.x;
        const auto& y = pt.y;
        if (x > 0) {
            if (y > 0) {
                four_quadrant[0] = true;
            } else if (y == 0) {
                four_quadrant[0] = true;
                four_quadrant[3] = true;
            } else {
                four_quadrant[3] = true;
            }
        } else if (x == 0) {
            if (y > 0) {
                four_quadrant[0] = true;
                four_quadrant[1] = true;
            } else if (y < 0) {
                four_quadrant[2] = true;
                four_quadrant[3] = true;
            }
        } else {
            if (y > 0) {
                four_quadrant[1] = true;
            } else if (y == 0) {
                four_quadrant[1] = true;
                four_quadrant[2] = true;
            } else {
                four_quadrant[2] = true;
            }
        }
    }
    if (four_quadrant[0] && four_quadrant[1] && four_quadrant[2] && four_quadrant[3]) {
        return QuadrantPtsType::FOURQUA;
    }
    if (four_quadrant[1] && four_quadrant[2]) {
        return QuadrantPtsType::ODD;
    }
    return QuadrantPtsType::NORMAL;
}

double ContourHull::PerpendicularDistance(const PolarData &pt, const PolarData &lineStart, const PolarData &lineEnd) {
    double dx = lineEnd.x - lineStart.x;
    double dy = lineEnd.y - lineStart.y;

    // Normalise
    double mag = pow(pow(dx, 2.0) + pow(dy, 2.0), 0.5);
    if (mag > 0.0) {
        dx /= mag; dy /= mag;
    }

    double pvx = pt.x - lineStart.x;
    double pvy = pt.y - lineStart.y;

    // Get dot product (project pv onto normalized direction)
    double pvdot = dx * pvx + dy * pvy;

    // Scale line direction vector
    double dsx = pvdot * dx;
    double dsy = pvdot * dy;

    // Subtract this from pv
    double ax = pvx - dsx;
    double ay = pvy - dsy;

    return pow(pow(ax, 2.0) + pow(ay, 2.0), 0.5);
}

void ContourHull::RamerDouglasPeucker(const std::vector<PolarData> &in_pts,
                                      std::vector<PolarData> &out_simplify, double epsilon) {
    if (in_pts.size() < 2) {
        out_simplify = in_pts;
    }
    // Find the point with the maximum distance from line between start and end
    double dmax = 0.0;
    size_t index = 0;
    size_t end = in_pts.size() - 1;
    for (size_t i = 1; i < end; i++) {
        double d = PerpendicularDistance(in_pts[i], in_pts[0], in_pts[end]);
        if (d > dmax) {
            index = i;
            dmax = d;
        }
    }

    // If max distance is greater than epsilon, recursively simplify
    if (dmax > epsilon) {
        // Recursive call
        std::vector<PolarData> recResults1;
        std::vector<PolarData> recResults2;
        std::vector<PolarData> firstLine(in_pts.begin(), in_pts.begin() + index + 1);
        std::vector<PolarData> lastLine(in_pts.begin() + index, in_pts.end());
        RamerDouglasPeucker(firstLine, recResults1, epsilon);
        RamerDouglasPeucker(lastLine, recResults2, epsilon);

        // Build the result list
        out_simplify.assign(recResults1.begin(), recResults1.end() - 1);
        out_simplify.insert(out_simplify.end(), recResults2.begin(), recResults2.end());
    } else {
        // Just return start and end points
        out_simplify.clear();
        out_simplify.push_back(in_pts[0]);
        out_simplify.push_back(in_pts[end]);
    }
}


}   // namespace robosense
