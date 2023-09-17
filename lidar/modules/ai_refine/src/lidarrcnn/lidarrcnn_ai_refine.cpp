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

#include "lidarrcnn/lidarrcnn_ai_refine.h"
#include "common/include/tic_toc.h"
#include "common/include/rs_util.h"
#include <sys/time.h>
#include "mogo_track/mogo_track_logger.h"
#include "mogo_track/gpu_track.h"
namespace robosense {

void LiDARRCNNAIRefine::init(const Rs_YAMLReader &configParse) {
    params_ptr_.reset(new LiDARRCNNParams);
    params_ptr_->load(configParse);
    initComponent();
}

void LiDARRCNNAIRefine::initComponent() {
    {
        {  // init infer and check
            RsInferConfig init_options;
            init_options.height = params_ptr_->rows;
            init_options.width = params_ptr_->cols;
            init_options.model_buffer = params_ptr_->model_buffer;
            init_options.trt_path = params_ptr_->trt_path;
            init_options.max_workspace = params_ptr_->max_workspace;
            init_options.enable_fp16 = params_ptr_->enable_fp_16;
            init_options.gie_path = params_ptr_->gie_path;
            init_options.device_id = params_ptr_->device_id;
            infer_ptr_ = std::shared_ptr<BaseInfer>(new RsTrtInfer);
            infer_ptr_->init(init_options);

        }

        {  // init preprocess,postprocess
            if (params_ptr_->use_cuda_acc) {
                TV_CHECK_CUDA_ERR_V3(cudaSetDevice(params_ptr_->device_id));
                preprocess_cuda_ptr_.reset(new LiDARRCNNPreprocess(params_ptr_));
                const auto& network_inputs = infer_ptr_->getInputs();
                assert(network_inputs.size() == 1);
                LiDARRCNNPreprocessData input_data;
                input_data.feature_data = reinterpret_cast<float*>(network_inputs[0]);
                preprocess_cuda_ptr_->init(input_data);

                const auto& network_outputs = infer_ptr_->getOutputs();			
                assert(network_outputs.size() == 5);
                LiDARRCNNPostprocessData output_data;                
                output_data.centers = reinterpret_cast<float*>(network_outputs[0]);
                output_data.headings = reinterpret_cast<float*>(network_outputs[1]);
                output_data.scores = reinterpret_cast<float*>(network_outputs[2]);
                output_data.class_ids = reinterpret_cast<int*>(network_outputs[3]);
                output_data.sizes = reinterpret_cast<float*>(network_outputs[4]);
                postprocess_ptr_.reset(new LiDARRCNNPostprocess(params_ptr_));
                postprocess_ptr_->init(output_data);

            } else {
                // const auto& feature_data = infer_ptr_->getInputs();
                // preprocess_ptr_.reset(new PointPillarsTwoHeadPreprocess(params_ptr_));
                // postprocess_ptr_.reset(new PointPillarsTwoHeadPostprocess(params_ptr_));
            }
        }
    }
}

void LiDARRCNNAIRefine::perception(const LidarFrameMsg::Ptr& msg_ptr) {
    if (!params_ptr_->enable) {
        return;
    }
    
    if (msg_ptr->valid_indices.empty()) {
        std::cerr << ": receive empty data!" << std::endl;
        return;
    }

    TRY_CATCH
    msg_ptr->transAxis(AxisType::VEHICLE_AXIS);
    if (params_ptr_->use_cuda_acc) {
        const auto& batch_size = params_ptr_->batch_size;
        TV_CHECK_CUDA_ERR_V3(cudaSetDevice(params_ptr_->device_id));
        for (int idx = 0; idx < msg_ptr->objects_proposals.size(); idx += batch_size){
            {
                mogo_track::GpuTrackGuard guard("ai_refine/preprocess","perception");
                TicToc timer("perception/AI refine/preprocess");
                preprocess_cuda_ptr_->preprocess(msg_ptr, idx);
            }
            {
                mogo_track::GpuTrackGuard guard("ai_refine/infer","perception");
                TicToc timer("perception/AI refine/infer");
                infer_ptr_->infer();
            }
            {
                mogo_track::GpuTrackGuard guard("ai_refine/postprocess","perception");
                TicToc timer("perception/AI refine/postprocess");
                postprocess_ptr_->postprocess(msg_ptr, idx);
            }
        }
        
    } else {
        // preprocess_ptr_->preprocess(msg_ptr);

        // std::vector<float *> inputs(4);
        // std::vector<float *> outputs(3);
        // inputs[0] = preprocess_ptr_->getFeatureData();
        // inputs[1] = preprocess_ptr_->getXcoordinateData();
        // inputs[2] = preprocess_ptr_->getYcoordinateData();
        // inputs[3] = preprocess_ptr_->getValidPillarCount();
        // outputs[0] = postprocess_ptr_->getClassificationData();
        // outputs[1] = postprocess_ptr_->getBoundingBoxesData();
        // outputs[2] = postprocess_ptr_->getDirectionClassData();

        // infer_ptr_->inferWithCpus(inputs, outputs);

        // postprocess_ptr_->postprocess(msg_ptr);
    }
    END_TRY_CATCH
}

}  // namespace robosense
