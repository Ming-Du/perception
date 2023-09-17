# 点云坐标转换节点



​        这是一个单独的ros节点，功能是将激光雷达扫描到的点云转到车体坐标系下，手动标定激光雷达外参时需要用到。在rviz中可以看到激光雷达实时点云，并可选择点云查看其在雷达坐标系下的xyz值。此节点订阅所有激光雷达消息，分别将其转换到车体坐标系base_link下发布。发布话题为 "/calibration/lidar/ORIENTATION" ，"ORIENTATION"为雷达安装方位。

​        该节点通过订阅 /tf 获取激光雷达外参，调用雷达驱动中设置外参的rosservice，修改雷达外参后，此节点的点云也会实时发生相应变化。

​        此外，由于需要订阅 /tf 获取外参，点云发布的实时性较差，仅用于雷达标定。另外在perception_lidar_node中发布 "/perception/lidar/transformed_pointcloud" 话题，同样发布转换到车体坐标系下的点云，实时完成点云坐标转换，可作为源数据供其他功能使用。

```
cd ~/catkin_ws/src/perception/pointcloud_transform
roslaunch ./pointcloud_transform.launch
```

