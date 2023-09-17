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

#pragma once

#include "common/include/basic_type/range.h"
#include "common/include/config/rs_yaml_pro.h"
#include "common/include/msg/lidar_frame_msg.h"
#include "common/include/rs_define.h"
#include "common/include/rs_details.h"

#include <iostream>
#include <fstream>
#include <memory>
#include <vector>

namespace robosense {

struct LiDARRCNNParams {
    using Ptr = std::shared_ptr<LiDARRCNNParams>;
    std::string model_file = "";
    bool enable = false;
    std::string frame_id = "";
    RsLidarType lidar_type = RsLidarType::RS128;
    int device_id = 0;
    bool use_cuda_acc = true;
    bool enable_fp_16 = true; // diff with bus
    int max_workspace = 30;
    float nms_thres = 0.02;
    float ior_thres = 0.5;
    int rows = 0;
    int cols = 0;
    
    const static int num_classes = 6;
    int pts_feature_num = 9;
    int num_points_per_proposal = 512;
    int batch_size = 10;


    std::string model_buffer = "";
    std::string gie_path = "";
    // trt_path   @add_by_lyl
    std::string trt_path = "";
    std::map<std::string, std::string> infos_map;
    void load(const Rs_YAMLReader &configParse);

private:
    // load params from yaml
    void loadParams(const Rs_YAMLReader &configParse);

    bool loadFile(const std::string &in_file, std::vector<char> &out_infos) {
        std::ifstream cache_infos(in_file.c_str(), std::ios::binary);
        if (cache_infos.good()) {
            // get size of file
            cache_infos.seekg(0, cache_infos.end);
            int size = static_cast<int>(cache_infos.tellg());
            out_infos.clear();
            out_infos.resize(size);

            cache_infos.seekg(0);
            // read content of infile
            cache_infos.read(out_infos.data(), size);
            cache_infos.close();
            return true;
        } else {
            std::cerr << "failed to load model file!" << std::endl;
            return false;
        }
    }

    // decode model file
    void parseModelFile() {
        std::vector<char> buffer;
        std::string decrypt_data;
        if (loadFile(model_file, buffer)) {
            model_buffer = std::string(buffer.begin(), buffer.end());
        }
    }

    // get model cache
    std::string getCacheLocation();
};

} // namespace robosense

