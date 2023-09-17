rosbag record --split --size=2048 \
/sensor/camera/6mm/image_raw \
/sensor/lidar/front/point_cloud \
/sensor/lidar/front_left/point_cloud \
/sensor/lidar/front_right/point_cloud \
/sensor/gnss/reference_time \
/localization/global \
/tf
