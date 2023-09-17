#pragma once

#include "gflags/gflags.h"

// Sensor ================================================
DECLARE_string(sensor_meta_path); // 传感器参数信息路径

// Lidar =================================================
// 点云输入源 与配置文件中的传感器名称相对应
// 默认第一个为reference frame
DECLARE_string(lidar_input);
// 激光雷达障碍物分割初始化配置文件路径
DECLARE_string(lidar_segmentation_conf_path);
DECLARE_string(lidar_filtered_topic); // 过滤后点云输出
DECLARE_string(lidar_obstacle_topic); // 障碍物输出
DECLARE_bool(publish_lidar_cluster);  // lidar节点是否需要可视化输出

// Camera ================================================
DECLARE_string(camera_detection_conf_path); // 视觉检测初始化配置文件路径
DECLARE_string(load_serialize_path);
DECLARE_string(load_class_names_path);

// Fusion ================================================
DECLARE_string(multi_sensor_fusion_conf_path);
DECLARE_bool(project_lidar_to_camera_mode);

// Radar
DECLARE_string(radar_fusion_conf_path);

// V2x
DECLARE_string(v2x_fusion_conf_path);
DECLARE_string(v2x_fusion_config_manager_path);

// config_manager
DECLARE_string(config_manager_path);
DECLARE_string(work_root);

// Mid Fusion
DECLARE_string(fusion_mid_conf_path);
DECLARE_string(fusion_mid_config_manager_path);
