cmake_minimum_required(VERSION 2.8.3)
project(perception_radar)

add_compile_options(
  -std=c++11
  -O3
  -g
)


find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  perception_base
  common
  autopilot_msgs
)

find_package(OpenCV REQUIRED)

# catkin_package()

catkin_package(
 LIBRARIES
  ${OpenCV_LIBRARIES}
)
set(RADAR_INCLUDE_DIRS "")     
set(RADAR_LIB_NAME_PREFIX perception_radar)         
#=============================
#  add modules
#=============================
add_subdirectory(common)
add_subdirectory(proto)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/../..
  ${PROJECT_SOURCE_DIR}/../../system/
  ${PROJECT_SOURCE_DIR}/../../system/common/proto
)

set(ALL_LIBS
        ${RADAR_LIB_NAME_PREFIX}_common
        ${RADAR_LIB_NAME_PREFIX}_proto
        )

file(GLOB SRC_RADAR_FUSION
  "${PROJECT_SOURCE_DIR}/fusion/fusion_node.cc"
  "${PROJECT_SOURCE_DIR}/fusion/obstacle_radar_fusion.cc"
  "${PROJECT_SOURCE_DIR}/util/roi_grid/mg_roigrid.cc"
  "${PROJECT_SOURCE_DIR}/rviz_display/*.cc"
)


add_executable(${PROJECT_NAME}_fusion_node ${SRC_RADAR_FUSION})

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

target_link_libraries(${PROJECT_NAME}_fusion_node 
    ${ALL_LIBS}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )


install(TARGETS
  ${PROJECT_NAME}_fusion_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
