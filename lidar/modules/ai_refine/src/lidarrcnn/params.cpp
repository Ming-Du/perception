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

#include "lidarrcnn/params.h"
#include "common/include/md5.h"

namespace robosense {

//=============================================
//      Params loading
//=============================================

const static char _application[] = "application";
const static char _lidar_type[] = "lidar_type";
const static char _strategy[] = "strategy";
const static char _date[] = "date";

void LiDARRCNNParams::load(const Rs_YAMLReader &configParse) {
    loadParams(configParse);
    parseModelFile();
    gie_path = getCacheLocation();
}

void LiDARRCNNParams::loadParams(const Rs_YAMLReader &configParse) {
    enable = configParse.getValue<bool>("ai_refine.enable");
    if (!enable) {
        return;
    }
    use_cuda_acc = configParse.getValue<bool>("ai_refine.use_cuda");
    model_file = configParse.getModelPath() + configParse.getValue<std::string>("ai_refine.model");
    trt_path = model_file ;
    std::cout << "[ INFO ] model_file: " << model_file << std::endl;
    std::string tmp_str = configParse.getValue<std::string>("ai_refine.lidar_type");
    if (kRsLidarTypeName2TypeMap.find(tmp_str) != kRsLidarTypeName2TypeMap.end()) {
        lidar_type = kRsLidarTypeName2TypeMap.at(tmp_str);
    }
    device_id = configParse.getValue<int>("ai_refine.device_id");
    batch_size = configParse.getValue<int>("ai_refine.batch_size");
#if defined __aarch64__
    enable_fp_16 = true;
#endif
    
}


std::string LiDARRCNNParams::getCacheLocation() {
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
