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
#include "lidarrcnn/postprocess.h"

namespace robosense {

Bndbox LiDARRCNNPostprocess::decodeLidarrcnn(std::vector<float> &centers, std::vector<float> &sizes,
                                             float headings, const Object::Ptr &proposal, int class_id, float score) {
    float center_x = centers[0];
    float center_y = centers[1];
    float center_z = centers[2];
    center_x *= proposal->size.x();
    center_y *= proposal->size.y();
    center_z *= proposal->size.z();

    float cos_yaw = proposal->direction[0];
    float sin_yaw = proposal->direction[1];
    float angle = std::atan2(sin_yaw, cos_yaw);

    float center_x_rot = cos_yaw * center_x - sin_yaw * center_y;
    float center_y_rot = sin_yaw * center_x + cos_yaw * center_y;

    center_x = center_x_rot + proposal->center.x();
    center_y = center_y_rot + proposal->center.y();
    center_z += proposal->center.z();

    float length = sizes[0] * proposal->size.x();
    float width = sizes[1] * proposal->size.y();
    float height = sizes[2] * proposal->size.z();

    return Bndbox(center_x, center_y, center_z, length, width, height, headings + angle, class_id, score);
}

void LiDARRCNNPostprocess::postprocess(const LidarFrameMsg::Ptr &lidar_msg_ptr, const int start_idx) {
    const auto &batch_size = params_ptr_->batch_size;
    const auto &nms_thresh = params_ptr_->nms_thres;
    const auto &ior_thresh = params_ptr_->ior_thres;

    auto output_centers = std::shared_ptr<float>(new float[batch_size * 3]);
    TV_CHECK_CUDA_ERR_V3(cudaMemcpy(output_centers.get(), centers_cuda_ptr_, batch_size * 3 * sizeof(float),
                                    cudaMemcpyDeviceToHost));
    auto output_sizes = std::shared_ptr<float>(new float[batch_size * 3]);
    TV_CHECK_CUDA_ERR_V3(cudaMemcpy(output_sizes.get(), sizes_cuda_ptr_, batch_size * 3 * sizeof(float),
                                    cudaMemcpyDeviceToHost));
    auto output_headings = std::shared_ptr<float>(new float[batch_size * 1]);
    TV_CHECK_CUDA_ERR_V3(cudaMemcpy(output_headings.get(), headings_cuda_ptr_, batch_size * 1 * sizeof(float),
                                    cudaMemcpyDeviceToHost));
    auto output_scores = std::shared_ptr<float>(new float[batch_size]);
    TV_CHECK_CUDA_ERR_V3(cudaMemcpy(output_scores.get(), scores_cuda_ptr_, batch_size * sizeof(float),
                                    cudaMemcpyDeviceToHost));
    auto output_class_ids = std::shared_ptr<int>(new int[batch_size]);
    TV_CHECK_CUDA_ERR_V3(cudaMemcpy(output_class_ids.get(), class_ids_cuda_ptr_, batch_size * sizeof(float),
                                    cudaMemcpyDeviceToHost));

    std::vector<Bndbox> res;
    res.reserve(batch_size);
    for (int i = 0; i < batch_size; i++) {
        if ((i + start_idx) == lidar_msg_ptr->objects_proposals.size()) {
            break;
        }
        std::vector<float> centers(output_centers.get() + 3 * i, output_centers.get() + 3 * (i + 1));
        std::vector<float> sizes(output_sizes.get() + 3 * i, output_sizes.get() + 3 * (i + 1));
        float headings = output_headings.get()[i];
        int class_id = output_class_ids.get()[i];
        float score = output_scores.get()[i];

        Bndbox bbox = decodeLidarrcnn(centers, sizes, headings, lidar_msg_ptr->objects_proposals[start_idx + i], class_id, score);

        // //TODO class_id
        // if (class_id == 0){
        //     continue;
        // }

        res.emplace_back(bbox);
    }
    // nms
    // TV_CHECK_CUDA_ERR_V3(cudaGetLastError())
    // auto res = nms_cpu(res, nms_thresh, ior_thresh);

    std::lock_guard<std::mutex> lock(mutex_);
    auto &proposals = lidar_msg_ptr->objects_proposals;
    for (size_t i = 0; i < res.size(); ++i) {
        auto &obj = proposals.at(start_idx + i);
        RotateBox box;
        Eigen::Vector3d center, size;
        size.x() = res[i].w;
        size.y() = res[i].l;
        size.z() = res[i].h;
        center.x() = res[i].x;
        center.y() = res[i].y;
        center.z() = res[i].z;
        box = RotateBox(center, size, res[i].rt);
        if (/*!obj->ismergelist*/true) {
            if (res[i].id <= 0) {
                obj->type = ObjectType::UNKNOW;
                if (obj->type_confidence == 0 && obj->ismergelist == false) {
                    obj->clone(*lidar_msg_ptr->objects_refine[obj->measure_id]);
                }
                obj->is_ai_refine = AiRefine_State::DISCARD_AIREFINE;
                continue;
            }
            if (fabs(size(0) - obj->size(0)) > 1) {
                if (obj->type_confidence == 0 && obj->ismergelist == false) {
                    obj->clone(*lidar_msg_ptr->objects_refine[obj->measure_id]);
                }
                obj->is_ai_refine = AiRefine_State::DISCARD_AIREFINE;
                continue;
            }
            double tmpheading = std::atan2(box.heading(1), box.heading(0));
            double difheading = fabs(control_psi(tmpheading - obj->heading));
            if (difheading > 3.14159265 / 2) {
                difheading -= 3.14159265;
            }
            if (fabs(difheading) > 3.14159265 / 45) {
                if (obj->type_confidence == 0 && obj->ismergelist == false) {
                    obj->clone(*lidar_msg_ptr->objects_refine[obj->measure_id]);
                }
                obj->is_ai_refine = AiRefine_State::DISCARD_AIREFINE;
                continue;
            }
        }
        obj->type_confidence = res[i].score;
        obj->exist_confidence = res[i].score;
        obj->is_ai_refine = AiRefine_State::AIREFINE;
        // airefinedebug
        if (res[i].id > 0) {
            res[i].id -= 1;
            obj->type = kIdxTypeMap.at(res[i].id);
        } else {
            if (obj->ismergelist) {
                obj->type = proposals.at(start_idx + i)->type;
            }
            else {
                obj->type = ObjectType::UNKNOW;
            }
        }
        obj->center = center;
        obj->size = size;
        obj->direction = box.heading;
    }
}

} // namespace robosense
