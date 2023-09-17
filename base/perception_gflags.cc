#include "perception/base/perception_gflags.h"

// Sensor
DEFINE_string(sensor_meta_path, "./src/system/config/vehicle/sensor/calibrated_sensor.pb.txt", 
  "The path to register sensor info file");

// Lidar
DEFINE_string(lidar_input, "ch_front;c16_front_left;c16_front_right", 
  "The list of lidar device names for input source");
DEFINE_string(lidar_segmentation_conf_path, "lidar_conf.pb.txt",
  "The path to lidar segmentation configure file");
DEFINE_string(lidar_filtered_topic, "/perception/lidar/filtered_cloud",
  "Topic of filtered lidar point cloud"); // 过滤后点云输出
DEFINE_string(lidar_obstacle_topic, "/perception/lidar/lidar_obstacle",
  "Topic of lidar segmentation obstacle");
DEFINE_bool(publish_lidar_cluster, false,
  "Disable cluster marker publishing when set false");

// Camera
DEFINE_string(camera_detection_conf_path, "camera_conf.pb.txt",
  "Teh path to camera detection configure file");
DEFINE_string(load_serialize_path, "serialize.engine",
  "");
DEFINE_string(load_class_names_path, "class.names",
  "");

DEFINE_string(multi_sensor_fusion_conf_path, "./src/system/config/vehicle/perception/fusion/multi_sensor_fusion.pb.txt",
  "");
DEFINE_bool(project_lidar_to_camera_mode, false,
            "project lidar cloud to camera, do not real fuse");

DEFINE_string(radar_fusion_conf_path, "./src/system/config/vehicle/perception/radar/radar_fusion.pb.txt",
  "");

DEFINE_string(v2x_fusion_conf_path, "./src/system/config/vehicle/perception/v2x/v2x_fusion.pb.txt",
  "");
DEFINE_string(v2x_fusion_config_manager_path, "./perception/v2x/config","The ModelConfig config paths.");
// config_manager
// DEFINE_string(config_manager_path, "production/conf/perception/fusion", "The ModelConfig config paths.");
DEFINE_string(config_manager_path, "", "The ModelConfig config paths.");
DEFINE_string(work_root, "", "Project work root direcotry.");

DEFINE_string(fusion_mid_conf_path, "./src/system/config/vehicle/perception/fusion_mid/fusion_mid_config.pb.txt",
  "");
DEFINE_string(fusion_mid_config_manager_path, "./perception/fusion_mid/config","The ModelConfig config paths.");