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

#ifndef RS_SDK_OBJECT_BUILD_H
#define RS_SDK_OBJECT_BUILD_H

#include "common.h"
#include "rs_define.h"
#include "basic_type/rotate_box.h"
#include "basic_type/object.h"
#include "basic_type/range.h"
#include "build_object/contour_full.h"

namespace robosense {

struct RsObjectBuilderInitOptions {
    Eigen::Vector2d leaf = Eigen::Vector2d(0.04, 0.04);
    // Eigen::Vector2d leaf = Eigen::Vector2d(0.2, 0.2);
};

//=========================================================
// @brief 计算一个object的基础信息
// 输入为点集,可以增加额外信息如direction或者bbox
// 输出包括:
// 1. direction,朝向
// 2. polygon(为一个背向lidar为凸包,面向lidar为凹包的多边形)
// 3. 最近点,最做点在polygon上的索引,最右点在polygon上的索引
// 4. geo_size,geo_center,为根据点集得到的实际大小和中心点
// 5. anchor,点集的重心
// 6. size,center,如果没有输入,则大小和geo_size,geo_center一致
// @attention 最基本依赖cloud_indices以及mode_type
//=========================================================

class RsObjectBuilder {
public:
    using Ptr = std::shared_ptr<RsObjectBuilder>;

    explicit RsObjectBuilder(const RsObjectBuilderInitOptions& init_options =
    RsObjectBuilderInitOptions()) {
        init_options_ = init_options;
        ContourHullOptions contour_options;
        contour_options.leaf = init_options_.leaf;
        contour_ptr_.reset(new ContourHull(contour_options));
        polygon_block_area_ = Range3D(-1.0, 20., -1., 1., 0., 0.);
        box_block_area_ = Range3D(-2.0, 30., -2., 2., 0., 0.);
    }

    //============================================================
    // @brief 根据点云的形状计算几何信息
    // @param[in] cloud_ptr,输入的整体点云
    // @param[in,out] obj_ptr,
    // 1. 根据输入的点云索引,得到点云集合
    // 2. 再根据其来源是否为ai来源,来选择计算的输出轮廓为凸包还是凹凸包,
    // 3. 再根据点云集合计算几何信息,几何信息包括最左最右点的索引,
    // 最近点,重心点,中心和长宽高,以及朝向
    //============================================================
    virtual bool buildObject(const PointCloud::Ptr& cloud_ptr,
                             const Object::Ptr& obj_ptr,bool is_debugobj);

    virtual bool buildObjectByPolygon(std::vector<Eigen::Vector3d> &multi_polygon,
                                      const Object::Ptr& obj_ptr,bool is_debugobj);

private:
    virtual std::string name() {
        return "RsObjectBuilder";
    }

    virtual bool baseBuild(const PointCloud::Ptr& cloud_ptr,
                           const Object::Ptr& obj_ptr, bool is_debugobj);

    //===================================================================
    // @brief 通过点云,polygon,以及direction计算几何信息,
    // 几何信息包括geo_center,geo_size,anchor
    // @param[in] cloud_ptr,输入的点云
    // @param[in,out] obj_ptr,输入的点云索引,polygon,以及direction,输入包括
    // 1. geo_center,geo_size,几何重心以及几何大小
    // 2. anchor,重心点
    //===================================================================
    virtual void geoCalculationByDir(const PointCloud::Ptr& cloud_ptr,
                                     const Object::Ptr& obj_ptr);

    //=================================================================
    // @brief 通过polygon计算这个polygon的朝向,
    // 主要的思路就是找到能够让polygon面积最小的边
    // @param[in] polygons,多边形点集
    // @param[out] dir,朝向
    //=================================================================
    virtual bool dirCalculationByPolygon(const std::vector<Eigen::Vector3d> &polygons,
                                         Eigen::Vector3d& dir);

    //===============================================
    // @brief 根据朝向计算polygon的大小和中心点
    // @param[in] polygon2D,dir
    // @param[out] size,center
    //===============================================
    void calcuGeoAlongDir(const std::vector<Eigen::Vector3d> &polygon2D, Eigen::Vector3d &dir,
                          Eigen::Vector3d &size, Eigen::Vector3d &center, bool fix_dir);

    void calcuAnchor(const PointCloud::Ptr& cloud_ptr,
                const std::vector<int> &cloud_indices, Eigen::Vector3d &anchor);

    void calcuMinMaxZ(const PointCloud::Ptr& cloud_ptr,
                      const std::vector<int> &cloud_indices, double &max_z, double &min_z);

    RsObjectBuilderInitOptions init_options_;
    ContourHull::Ptr contour_ptr_;
    Range3D polygon_block_area_;
    Range3D box_block_area_;
};
}   // namespace robosense
#endif //RS_SDK_OBJECT_BUILD_H
