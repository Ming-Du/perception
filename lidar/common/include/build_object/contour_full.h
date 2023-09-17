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

#ifndef RS_SDK_CONTOUR_HULL_H
#define RS_SDK_CONTOUR_HULL_H

#include <numeric>
#include "rs_define.h"
#include "build_object/simple_2d_grid.h"

namespace robosense {

enum class QuadrantPtsType {
    NORMAL = 0,
    FOURQUA = 1,
    ODD = 2,
};

//===================================================================
// @brief 判断一个点集是否为4象限点集或者奇异点集,所谓四象限点集,
// 就是点集范围覆盖四个象限,而奇异点集则是点集范围不是四象限点集,但是点集覆盖1,2象限
// @param pts
// @return NORMAL:表示正常,FOURQUA表示四象限点集,ODD,表示奇异点集
//===================================================================
inline QuadrantPtsType isFourQuadrant(const std::vector<Eigen::Vector3d>& pts) {
    // 四象限标记,0象限:x>0,y>0;1象限:x<0,y>0;2象限:x<0,y<0;3象限:x>0,y<0
    std::vector<bool> four_quadrant(4, false);
    for (const auto& pt : pts) {
        const auto& x = pt.x();
        const auto& y = pt.y();
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

struct ContourHullOptions {
    Eigen::Vector2d leaf = Eigen::Vector2d(0.2, 0.2);
    double driveable_width = 2.;
    double res_angle = 1.;
};

//=================================================================================
// @brief 计算一个点集的轮廓信息的函数,输出包括轮廓点集,逆时针旋转最左点,最右点以及最近点索引
// @attention 输出的轮廓为逆时针轮廓,最左点为点集沿着坐标轴逆时针得到的第一个点,最右点为最后一个点
//=================================================================================

class ContourHull {
public:
    using Ptr = std::shared_ptr<ContourHull>;

    struct ContourInfos {
        std::vector<Eigen::Vector3d> contours;
        bool is_four_quadrant_polygons = false;
        int nearest_pt_idx = -1;
        int l_pt_idx = -1;
        int r_pt_idx = -1;
    };

    explicit ContourHull(const ContourHullOptions& options = ContourHullOptions());

    void build(const PointCloud::Ptr& cloud_ptr, ContourInfos& contour) {
        // std::vector<int> pc_indices(cloud_ptr->size());
        // std::iota(pc_indices.begin(), pc_indices.end(), 0);
        // build(cloud_ptr, pc_indices, contour);
    }
    // 计算polygon,contour以及lpt_idx,rpt_idx在polygon和contour中的索引
    void build(const PointCloud::Ptr& cloud_ptr, const std::vector<int>& pc_indices, ContourInfos& contour,bool is_debugobj);

    void buildSimple(const PointCloud::Ptr& cloud_ptr, ContourInfos& contour) {
        std::vector<int> pc_indices(cloud_ptr->size());
        std::iota(pc_indices.begin(), pc_indices.end(), 0);
        buildSimple(cloud_ptr, pc_indices, contour);
    }
    // polygon与contour一致,无须再额外计算contour
    void buildSimple(const PointCloud::Ptr& cloud_ptr, const std::vector<int>& pc_indices, ContourInfos& contour);

    void buildByPolygon(std::vector<Eigen::Vector3d> &multi_polygon, ContourInfos& contour,bool is_debugobj);

    struct PolarData {
        double x, y, range, hori;
    };

    void build(const std::vector<PolarData>& in_data, ContourInfos& contour, const bool& simple,bool is_debugobj);

    bool is_four_quadrant_ = false;

private:
    std::string name() {
        return "ContourHull";
    }

    //================================================================
    // @brief 计算一个输入点集的最左最右最近点索引
    // @param[in] polar_vec,输入点集
    // @param[out] l_idx,r_idx,n_idx,输出最左点,最右点以及最近点索引
    //================================================================
    void comLrn(const std::vector<PolarData>& polar_vec, int& l_idx, int& r_idx, int& n_idx);

    double cross(const PolarData &O, const PolarData &A, const PolarData &B);

    // 输出的polygon是顺时针
    void convexHull(std::vector<PolarData> P, std::vector<PolarData> &polygon,bool is_debugobj);

    //======================================================================
    // @brief 判断一个点集是否为四象限点集,或者是奇异点集,四象限点集表示点集涵盖了坐标
    // 的四个象限,奇异点集表示不是四象限点集但是点集覆盖了二,三象限
    // @param[in] pts,输入的点集
    // @return 输出类型
    //======================================================================
    QuadrantPtsType isFourQuadrant(const std::vector<PolarData>& pts);

    double PerpendicularDistance(const PolarData &pt, const PolarData &lineStart, const PolarData &lineEnd);

    //=======================================
    // @brief 道格拉斯普度算法,用于简化点集
    // @param[in] in_pts,输入点集
    // @param[out] out_simplify,输出的简化点集
    // @param[in] epsilon,阈值参数
    //=======================================
    void RamerDouglasPeucker(const std::vector<PolarData> &in_pts,
                             std::vector<PolarData> &out_simplify, double epsilon = 0.2);

    ContourHullOptions options_;
    Simple2DGrid::Ptr grid_filter_ptr_;
    double res_ang_rad_;
};

}

#endif //RS_SDK_CONTOUR_HULL_H
