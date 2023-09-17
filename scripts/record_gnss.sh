rosbag record --split --size=2048 \
/sensor/gnss/odometry \
/sensor/gnss/gps_fix \
/sensor/gnss/imu \
/sensor/gnss/best_gnss_vel \
/sensor/gnss/reference_time \
/tf
