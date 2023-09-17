

//   Eigen::Affine3d camera2world_pose, lidar2world_pose, offset;
//   base::SensorInfo camera_info, lidar_info;
//   sensor_manager->GetSensorInfo("6mm_front", &camera_info);
//   sensor_manager->GetSensorInfo("ch_front", &lidar_info);
//   // base::SetSensorExtrinsics(camera_info.extrinsic(), camera2world_pose);
//   base::SetSensorExtrinsics(lidar_info.extrinsic(), lidar2world_pose);

//   Eigen::Matrix4d world2camera_pose;

//   Eigen::Matrix4d camera2lidar, camera2world;
//   Eigen::Quaterniond quaternion, quaternion_offset(1,0,-0.008,0);

//   offset.setIdentity();
//   offset.rotate(quaternion_offset);

//   camera2lidar << 0.999895, 0.0044684, -0.0137846, 0.045805,
//                   0.0138226, -0.00859093, 0.999868, -1.60161,
//                   0.00434939, -0.999953, -0.0086518, 0.983166,
//                   0, 0, 0, 1;
//   camera2world = lidar2world_pose.matrix() * camera2lidar * offset.matrix();

//   Eigen::Matrix3d aa = camera2world.topLeftCorner(3,3);
//   quaternion = aa;

//   Eigen::Vector3d euler_angle = quaternion.matrix().eulerAngles(2,1,0);

// std::cout << "w," << quaternion.w() 
//           << " x," << quaternion.x() 
//           << " y," << quaternion.y() 
//           << " z," << quaternion.z()  << std::endl;

// std::cout << euler_angle << std::endl;
// std::cout << camera2lidar.matrix() << std::endl;
// std::cout << lidar2world_pose.matrix() << std::endl;
// std::cout << offset.matrix() << std::endl;
// std::cout << camera2world.matrix() << std::endl;
// std::cout << camera2world.matrix().inverse() << std::endl;

