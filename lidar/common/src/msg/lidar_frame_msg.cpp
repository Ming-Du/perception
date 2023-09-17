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
#include "msg/lidar_frame_msg.h"
#include "rs_util.h"
#include <pcl/common/transforms.h>

namespace robosense{
void LidarFrameMsg::transAxis(const AxisType & s,const std::string& ){
    if (s == status){
        return;
    }

    const auto& cur_pose_ptr = axis_pose_map.at(status);
    const auto& next_pose_ptr = axis_pose_map.at(s);
    if (*cur_pose_ptr == *next_pose_ptr){
        this->status = s;
        return;
    }
    Eigen::Matrix4d cur_mat, next_mat;
    cur_mat = poseToEigenMat(*cur_pose_ptr);
    next_mat = poseToEigenMat(*next_pose_ptr);

    Eigen::Matrix4d transform_mat;
    transform_mat = next_mat * cur_mat.inverse();

    //点云变换
    // pcl::transformPointCloud<pcl::PointXYZI>(*this->scan_ptr,*this->scan_ptr,transform_mat);
    // result 变换
    {
        for (auto& obj : this->objects) {
            obj->transform(transform_mat);
        }
        for (auto& obj : this->objects_refine) {
            obj->transform(transform_mat);
        }        
    }
    this->status = s;
}
//only global to vehicle   Transform_matrix
void LidarFrameMsg::transGlobal2VehicleMat(Eigen::Matrix4d &transform_mat){
    const auto& cur_pose_ptr = axis_pose_map.at(AxisType::GLOBAL_AXIS);
    const auto& next_pose_ptr = axis_pose_map.at(AxisType::VEHICLE_AXIS);

    Eigen::Matrix4d cur_mat, next_mat;
    cur_mat = poseToEigenMat(*cur_pose_ptr);
    next_mat = poseToEigenMat(*next_pose_ptr);

    transform_mat = next_mat * cur_mat.inverse();
}

//only vehicle to global   Transform_matrix
void LidarFrameMsg::transVehicle2GlobalMat(Eigen::Matrix4d &transform_mat){
    const auto& cur_pose_ptr = axis_pose_map.at(AxisType::VEHICLE_AXIS);
    const auto& next_pose_ptr = axis_pose_map.at(AxisType::GLOBAL_AXIS);

    Eigen::Matrix4d cur_mat, next_mat;
    cur_mat = poseToEigenMat(*cur_pose_ptr);
    next_mat = poseToEigenMat(*next_pose_ptr);

    transform_mat = next_mat * cur_mat.inverse();
}
}
