cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_transform)

add_compile_options(
  -std=c++11
  -O3
  -Wno-sign-compare
  -Wno-unused-function
  -Wno-unused-variable
  -Wno-ignored-attributes
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  perception_base
  pcl_ros
  tf2_ros
  common
)

catkin_package(
INCLUDE_DIRS
  ${PROJECT_SOURCE_DIR}
)

include_directories(
  ${PROJECT_SOURCE_DIR}/../..
  ${PROJECT_SOURCE_DIR}/..
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
)

add_executable(${PROJECT_NAME}_node src/pointcloud_transform.cpp)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
	target_link_libraries(${PROJECT_NAME}_node
		${catkin_LIBRARIES}
    gomp
    yaml-cpp
		/usr/lib/aarch64-linux-gnu/libglog.so.0
	)
else()
	target_link_libraries(${PROJECT_NAME}_node
		${catkin_LIBRARIES}
    gomp
    yaml-cpp
	)
endif()
