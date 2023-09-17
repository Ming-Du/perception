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
#ifndef RS_COMMON_RS_YAMLREADER_H_
#define RS_COMMON_RS_YAMLREADER_H_
#include <string>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include "rs_yaml_pro.h"

namespace robosense{
    class Rs_YAMLReader {
    public:
        Rs_YAMLReader() = default;
        explicit Rs_YAMLReader(const std::string& filename);
        explicit Rs_YAMLReader(const std::string& basePath, const std::string& filename);
        // Copy constructor
        Rs_YAMLReader(const Rs_YAMLReader& other);
        // Copy assignment operator
        Rs_YAMLReader& operator=(const Rs_YAMLReader& other);
        template <typename T>
        T getValue(const std::string& key = "", const T& default_value = T()) const;
        std::string getModelPath() const;
        std::string getMapPath() const;
        std::string getBasePath() const;
        static std::string getVehicleType(const std::string& filename);
    private:
        std::string basePath_;
        YAML::Node data_;
        void loadFile(const std::string& filename);
        void resolve_includes(YAML::Node& node, const std::string& filename);
        void merge_node(YAML::Node node, YAML::Node that);
    };

    template <typename T>
    T Rs_YAMLReader::getValue(const std::string& key, const T& default_value) const {
        YAML::Node node = YAML::Clone(data_);
        // Query points according to hierarchical relationship
        if(!key.empty()){
            std::string::size_type pos = 0;
            while (pos != std::string::npos) {
                std::string::size_type next_pos = key.find('.', pos);
                std::string subkey = key.substr(pos, next_pos - pos);
                if (node[subkey]) {
                    node = node[subkey];
                    pos = next_pos == std::string::npos ? next_pos : next_pos + 1;
                } else {
                    ROS_ERROR("Failed to read YAML node %s: set default value, errmsg: not find key [%s]", key.c_str(), subkey.c_str());
                    return default_value;
                }
            }
        }
        try {
            return node.as<T>();
        } catch (const std::runtime_error& e) {
            ROS_ERROR("Failed to read YAML node %s: set default value, errmsg: please check key [%s]", key.c_str(), e.what());
            return default_value;
        }
    }
}

#endif  // RS_COMMON_RS_YAMLREADER_H_

