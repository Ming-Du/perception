cmake_minimum_required(VERSION 3.0.2)
project(perception_v2x)

# set(CMAKE_BUILD_TYPE Debug)
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

add_compile_options(
  -std=c++11
  -O3
  -g
)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  perception_base 
  roscpp
  rospy
  std_msgs
  common
  autopilot_msgs
)

find_package(Eigen3 REQUIRED)
find_package(GEOS REQUIRED)

catkin_package()
add_subdirectory(proto)

catkin_package(
LIBRARIES
${EIGEN3_INCLUDE_DIRS}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/../..
  ${EIGEN3_INCLUDE_DIRS}
)

file(GLOB SRC_PERCEPTION_V2X
  "${PROJECT_SOURCE_DIR}/app/v2x_fusion_node.cc"
  "${PROJECT_SOURCE_DIR}/app/obstacle_v2x_fusion.cc"
  "${PROJECT_SOURCE_DIR}/common/self_veh_filter/self_veh_filter.cc"
  "${PROJECT_SOURCE_DIR}/common/config_manager/v2x_config_manager.cc"
  "${PROJECT_SOURCE_DIR}/common/rviz_display/v2x_display.cc"
)

add_executable(${PROJECT_NAME}_fusion_node ${SRC_PERCEPTION_V2X} )
if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
	target_link_libraries(${PROJECT_NAME}_fusion_node
		${catkin_LIBRARIES}
		${OpenCV_LIBRARIES}
		/usr/lib/aarch64-linux-gnu/libglog.so.0
	)
else()
	target_link_libraries(${PROJECT_NAME}_fusion_node
		${catkin_LIBRARIES}
		${OpenCV_LIBRARIES}
	)
endif()
target_link_libraries(${PROJECT_NAME}_fusion_node ${catkin_LIBRARIES} ${PROJECT_NAME}_proto yaml-cpp GEOS::geos)


install(TARGETS
  ${PROJECT_NAME}_fusion_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)


