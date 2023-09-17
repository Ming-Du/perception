# RS_SDK使用说明

## 1.说明
rs_sdk是由Robosense开发人员设计开发的基于激光雷达的感知软件包，该软件包可以单独使用，也可以作为子模块用于用户自己的Project中。对于没有二次开发需求的用户，可直接使用本软件包获得感知结果，如有二次开发需求请参考[6.二次开发](#6. 二次开发)中的详细说明。

该感知软件包提供以下功能：
- 前景目标(Objects)检测和分类(Pillar方法)
- 一度目标(Attention Objects)检测
- 可行驶区域(Freespace)检测
- 目标跟踪
- 地面检测、分割聚类

## 2.环境依赖
- **x86_64**
  - Linux( Ubuntu >= 16.04)
  - ROS ，推荐安装 `ros-kinetic-desktop-full` for ubantu 16.04  / `ros-melodic-desktop-full` for ubantu 18.04
  - Eigen3
  - NVIDIA软件栈
      - 显卡驱动 >= 410.48
      - CUDA 10.0
      - CuDNN for CUDA 10.0
- **Xavier**
  - Linux(Ubuntu == 18.04)
  - ROS
  - Eigen3 == 3.3.9
  - NVIDIA软件栈
     - CUDA10.0 or CUDA10.2
     - cuDNN for CUDA10.0 or CUDA10.2

## 3.使用方法
### 3.1 demo运行

1.在工程目录下打开一个新终端，运行如下命令编译工程
```shell
$ mkdir build
$ cd build
$ cmake .. 
$ make
```
2.编译完成后输入如下命令运行demo感知程序(运行前请确保配置参数正确)

在Xavier上，编译完成后还需要手动导出TensorRT的路径

```shell
# 使用TensorRT 5.1.6
$ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:($RS_SDK_ROOT)/modules/perception/infer/ai_engine/tensorRT/aarch64/v5/lib

# 使用TensorRT 7.1.3
$ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:($RS_SDK_ROOT)/modules/perception/infer/ai_engine/tensorRT/aarch64/v7/lib
```

现在可以运行demo程序

```shell
$ ./demo/rs_perception_demo
```

### 3.2 坐标系说明
**雷达坐标系:** 雷达传感器自身定义的坐标系
**车体坐标系:** 以车体后轴与地面相交处为中心,x-y平面与地面平行,x轴指向车辆前进方向的坐标系
**全局坐标系:** 指的是在配合定位或者其他有高精度地图的应用中,存在一个全局位置的坐标系,每个雷达相对这个全局位置的pose就是全局坐标系pose

### 3.3 参数说明
**calibration参数**
```yaml
calibration参数位于config/usr_config/calibration.yaml文件中，标定参数格式说明如下：
lidar:
  - parent_frame_id: /base_link  
    frame_id: /middle_lidar   # 对应雷达frame id
    device_type: RS128   # 雷达类型: RS80, RSHELIOS 
    # 该雷达到 parent_frame_id: /base_link 坐标系的变换pose
    x: 0
    y: 0
    z: 1.8
    roll: 0
    pitch: 0
    yaw: 0
```
**perception参数**
```yaml
perception参数位于system_config/perception_config.yaml和usr_config/usr_config.yaml文件中，
system_config/perception_config.yaml和usr_config/usr_config.yaml感知参数格式层级相同，usr_config.yaml的参数会覆盖perception_config.yaml中相应的参数

general:
  # ai detection model root path
  model: /system_config/perception_config/model
  # 感知功能中所使用的标定参数文件，内容与上述标定参数相同
calibration:
  include: /usr_config/calibration.yaml

lidar:
  #* Common *#  
  common:
    data_fusion: false  # 点云数据前融合，如打开，则使用融合点云感知

  #* Multi lidar*#
  sub:
    - config:
      include: /system_config/lidar/middle_lidar.yaml
    - config:
      include: /system_config/lidar/left_lidar.yaml
  ##* Single Lidar*##
  #* PostFusion *#  将perception_msg中的lidar_frame_msg_map拷贝到lidar_result_msg_ptr中，如有多个雷达则进行结果融合
  post_fusion:

  #* BasicDetection *#
  basic_detection:  # 几何检测相关参数
    enable: true    # 几何检测功能开关
    detect_range:   # 检测范围
      xmin: -100
      xmax: 100
      ymin: -100
      ymax: 100
      zmin: 0.
      zmax: 3.
    # 可行驶区域检测角度范围
    min_hori_angle: -180
    max_hori_angle: 180

  #* Tracking *#
  tracking:  # 跟踪功能相关参数
    enable: false  # 跟踪功能开关
    history_num: 10  # 使用历史帧数量
    match_distance_max: 3.0  # 匹配最大距离

#* Rviz display *#
rviz:
  prefix: "" # DEFAULT   # rviz显示topic名称前缀
  frame_id: "base_link"  # rviz显示topic使用的frame id
```

## 4.Msg内容说明
### 4.1 PerceptionMsg

PerceptionMsg包含两部分内容

- **lidar_result_msg_ptr**

自定义数据类LidarFrameMsg


```c++
// string类型 雷达对应frame id
frame_id;
// double类型 雷达时间戳
timestamp;
// 自定义pose结构 车体坐标系到全局坐标系的变换pose参数
global_pose_ptr;
// 雷达坐标系到其他坐标系变换pose map
std::map<AxisType,Pose::Ptr> axis_pose_map;
// 保存当前msg所处的坐标系状态
AxisType  status;
// pcl格式点云
PointCloud::Ptr scan_ptr;

//自定义Object结构的vector 目标检测结果列表
objects;
// 自定义Object结构的vector 一度目标检测结果列表
attention_objects;
// 可行驶区域点集
freespace_ptr;
// 有效点索引列表
valid_indices;
// 地面点云索引列表
ground_indices,
// 非地面点云索引列表
non_ground_indices,
// 背景点云索引列表
background_indices;
```

- **lidar_frame_msg_map** 

用于存储多个雷达点云数据及感知结果map


## 5.API接口说明
### 5.1 config parser

- **读取参数配置**

```c++
// 包含头文件
#include "common/include/rs_config_parser.h"

std::string config_path = (std::string)PROJECT_PATH + "/config"; // 配置文件根目录
YAML::Node config_node = RsConfigParser().parse(config_path); // config_node 为整个yaml node根节点。
```

### 5.2 调用感知功能：

- **感知功能初始化和接口调用**

```c++
// 包含头文件
// rs_perception.cpp中已经组织好所有功能模块的使用逻辑，详情请参考rs_perception.cpp文件
#include "rs_perception/application/rs_perception.h"

// 初始化
LidarPerception::Ptr lidar_perception_ptr;
lidar_perception_ptr.reset(new LidarPerception);
lidar_perception_ptr->init(perception_node);
```

```c++
// 接口调用
PerceptionMsg::Ptr msg_ptr (new PerceptionMsg);
lidar_perception_ptr->perception(msg_ptr);
```

- **Rviz显示**

```c++
// 包含头文件
#include "rs_perception/rviz_display/rviz_display.h"

// 初始化
RvizDisplay::Ptr display_ptr;
display_ptr.reset( new RvizDisplay(node_ptr, rviz_node));

// 接口调用
display_ptr->display(msg_ptr);
```

**可以参考demo/rs_perception_demo.cpp中使用示例**

### 5.3 调用感知功能模块

以预处理模块为例：

```c++
// 包含头文件
#include "rs_perception/preprocessing/rs_preprocessing.h"

// 初始化
RsPreprocessing::Ptr preprocessing_ptr_
preprocessing_ptr_.reset(new RsPreprocessing);
YAML::Node preprocessing_node;
yamlSubNode(config_node, "preprocessing", preprocessing_node);
preprocessing_ptr_->init(preprocessing_node);

// 接口调用
preprocessing_ptr_->perception(msg_ptr);
```

**可以参考rs_perception.cpp和rs_sub_perception.cpp中使用示例**
