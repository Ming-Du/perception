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

#include "pointpillars/params.h"
#include "common/include/md5.h"

namespace robosense {

//=============================================
//      Params loading
//=============================================

const static char _application[] = "application";
const static char _lidar_type[] = "lidar_type";
const static char _strategy[] = "strategy";
const static char _cur_strategy[] = "PointPillarsCenterHeadCnnDetection";
const static char _date[] = "date";

void PointPillarsCenterHeadParams::load(const Rs_YAMLReader &configParse) {
    loadParams(configParse);
    if (!enable)  return;
    
    parseModelFile();
    checkDetectRang();
    gie_path = getCacheLocation();
}

void PointPillarsCenterHeadParams::loadParams(const Rs_YAMLReader &configParse) {
    enable = configParse.getValue<bool>("ai_detection.enable");
    if (!enable) {
        return;
    }
    use_cuda_acc = configParse.getValue<bool>("ai_detection.use_cuda");
    use_confidence_map = configParse.getValue<bool>("ai_detection.use_confidencemap");
    model_file = configParse.getModelPath() + configParse.getValue<std::string>("ai_detection.model");
    trt_path = model_file ;
    std::cout << "[ INFO ] model_file: " << model_file << std::endl;
    std::string tmp_str = configParse.getValue<std::string>("ai_detection.lidar_type");
    if (kRsLidarTypeName2TypeMap.find(tmp_str) != kRsLidarTypeName2TypeMap.end()) {
        lidar_type = kRsLidarTypeName2TypeMap.at(tmp_str);
    }
    device_id = configParse.getValue<int>("ai_detection.device_id");
#if defined __aarch64__
    enable_fp_16 = true;
#endif
    detection_range = configParse.getValue<Range3D>("ai_detection.detect_range");
    pillar_z_size = configParse.getValue<float>("ai_detection.pillar_z_size");
    max_num_pillars = configParse.getValue<double>("ai_detection.max_num_pillars");
    centerhead_regression_box_confidence_thre = configParse.getValue<double>("ai_detection.default_confidence_thres");
    range_boundary = configParse.getValue<std::vector<float>>("ai_detection.type_range_confidence.range_boundary");
    for (auto i : type_list) {
        vec_type_to_range.emplace_back(configParse.getValue<std::vector<float>>("ai_detection.type_range_confidence." + i));
    }
}

void PointPillarsCenterHeadParams::checkDetectRang() {
    Range3D ori_detection_range = detection_range;
    float x_min = ori_detection_range.xmin;
    float x_max = ori_detection_range.xmax;
    float y_min = ori_detection_range.ymin;
    float y_max = ori_detection_range.ymax;

    float x_middle = x_min + (x_max - x_min) / 2.;
    float y_middle = y_min + (y_max - y_min) / 2.;

    int width = std::ceil((x_max - x_min) / pillar_x_size);
    int height = std::ceil((y_max - y_min) / pillar_y_size);

    if (width % 32 != 0) {
        width = (width / 32 + 1) * 32;
    }
    float tmp_x_length = static_cast<float>(width) * pillar_x_size;
    detection_range.xmax = tmp_x_length / 2. + x_middle;
    detection_range.xmin = x_middle - tmp_x_length / 2.;

    if (height % 32 != 0) {
        height = (height / 32 + 1) * 32;
    }
    float tmp_y_length = static_cast<float>(height) * pillar_y_size;
    detection_range.ymax = tmp_y_length / 2. + y_middle;
    detection_range.ymin = y_middle - tmp_y_length / 2.;

    rows = height;
    cols = width;

    bev_size = cols * rows;
    feature_x_size = cols / downsample_stride;
    feature_y_size = rows / downsample_stride;
}

std::string PointPillarsCenterHeadParams::getCacheLocation() {
    std::string tmp_infos = "";
    // add lidar_type
    tmp_infos += kRsLidarType2NameMap.at(lidar_type);
    tmp_infos += "~";
    // add date
    auto itr = infos_map.find(_date);
    if (itr != infos_map.end()) {
        tmp_infos += infos_map[_date];
    } else {
        tmp_infos += "00";
    }
    tmp_infos += "~";
    // add strategy
    itr = infos_map.find(_strategy);
    if (itr != infos_map.end()) {
        tmp_infos += infos_map[_strategy];
    } else {
        tmp_infos += "00";
    }
    tmp_infos += "~";
    // add height
    tmp_infos += std::to_string(rows);
    tmp_infos += "~";
    // add width
    tmp_infos += std::to_string(cols);
    tmp_infos += "~";
    // add model_buffer
    tmp_infos += model_buffer;
    std::string md5_code = md5(tmp_infos);

#if defined __x86_64__ || defined __arm__
    return "/tmp/." + md5_code + "~";
#else
    return "." + md5_code + "~";
#endif
}

} // namespace robosense
