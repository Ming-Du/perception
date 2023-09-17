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
#include "config/rs_yamlReader.h"
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <experimental/filesystem>
#include <ros/ros.h>

#define SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING
namespace fs = std::experimental::filesystem;

namespace robosense{
    Rs_YAMLReader::Rs_YAMLReader(const std::string& filename) {
        loadFile(filename);
    }

    Rs_YAMLReader::Rs_YAMLReader(const std::string& basePath, const std::string& filename) {
        // Load YAML data from file
        basePath_ = basePath;
        std::string full_filename = basePath + "/" + filename;
        loadFile(full_filename);
    }

    void Rs_YAMLReader::loadFile(const std::string& filename) {
        // Load YAML data from file
        try {
            data_ = YAML::LoadFile(filename);
            resolve_includes(data_, filename);
        } catch (const YAML::Exception& e) {
            ROS_ERROR("Failed to load YAML file %s", filename.c_str());
        }
        YAML::Emitter emitter;
        emitter << data_;
        ROS_INFO("YAML file %s loaded\nparams:\n%s", filename.c_str(), emitter.c_str());
    }

    Rs_YAMLReader::Rs_YAMLReader(const Rs_YAMLReader& other) {
        this->data_ = YAML::Clone(other.data_);
    }

    Rs_YAMLReader& Rs_YAMLReader::operator=(const Rs_YAMLReader& other) {
        if (this != &other) {
            this->data_ = YAML::Clone(other.data_);
        }
        return *this;
    }
    std::string Rs_YAMLReader::getVehicleType(const std::string& filename) {
        std::ifstream file(filename);
        if (!file) {
            ROS_WARN("Error: Cannot open file %s use default value jinlv", filename.c_str());
            return "jinlv";
        }
        std::stringstream ss;
        ss << file.rdbuf();
        std::string line;
        std::string subtype;
        std::string brand;
        while (std::getline(ss, line)) {
            if (line.find("subtype") != std::string::npos) {
                subtype = line.substr(line.find('\"') + 1, -1);
            }
            if (line.find("brand") != std::string::npos) {
                brand = line.substr(line.find('\"') + 1, -1);
            }
        }

        if (!subtype.empty()) {
            std::transform(subtype.begin(), subtype.end(), subtype.begin(), ::tolower);
            if (subtype.back() == '"') {
                subtype = subtype.substr(0, subtype.find_last_of('\"'));
            }
            return subtype;
        } else {
            std::transform(brand.begin(), brand.end(), brand.begin(), ::tolower);
            if (brand.back() == '"') {
                brand = brand.substr(0, brand.find_last_of('\"'));
            }
            return brand;
        }
    }
    std::string Rs_YAMLReader::getModelPath() const {
        auto model_type = getValue<std::string>("type", "leishen");
        return basePath_ + "/model/" + model_type + "/";
    }

    std::string Rs_YAMLReader::getMapPath() const {
        return basePath_ + "/map/";
    }

    std::string Rs_YAMLReader::getBasePath() const {
        return basePath_;
    }

    void Rs_YAMLReader::resolve_includes(YAML::Node& node, const std::string& filename) {
        if (node.IsMap()) {
            if (node["include"]) {
                std::cout << "[ INFO ] include: " << node["include"] << std::endl;
                // Handle include statement
                auto include_filename = node["include"].as<std::string>();
                fs::path current_path = fs::path(filename).parent_path();
                fs::path include_path = current_path / include_filename;
                std::ifstream ifs(include_path);
                if (!ifs.is_open()) {
                    include_path = fs::path(basePath_) / include_filename;
                    ifs = std::ifstream(include_path);
                    if (!ifs.is_open()) {
                        ROS_ERROR("Failed to open file: %s", include_path.c_str());
                        return;
                    }
                }
                YAML::Node include_node;
                try {
                    include_node = YAML::Load(ifs);
                } catch (const YAML::Exception& e) {
                    ROS_ERROR("Failed to load YAML file %s", include_path.c_str());
                }
                resolve_includes(include_node, include_path.string());
                node.remove("include");  // Remove include key
                merge_node(node, include_node);
            }
            // Recursively process child nodes
            for (auto it = node.begin(); it != node.end(); ++it) {
                resolve_includes(it->second, filename);
            }
        }
    }
    void Rs_YAMLReader::merge_node(YAML::Node node, YAML::Node that) {
        // Merge included YAML data into current YAML data
        for (auto it = that.begin(); it != that.end(); ++it) {
            auto key = it->first.as<std::string>();
            if (!node[key].IsDefined()) {
                node[it->first] = it->second;
            } else {
                if(it->second.IsMap()) {
                    merge_node(node[key], YAML::Clone(that[key]));
                }
            }
        }
    }
}
