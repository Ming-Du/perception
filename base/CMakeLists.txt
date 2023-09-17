cmake_minimum_required(VERSION 2.8.3)
project(perception_base)

add_compile_options(-std=c++11 -O3)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  common
)

find_package(Eigen3)
# find_package(OpenCV REQUIRED)

catkin_package(
 INCLUDE_DIRS
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/../..
 LIBRARIES
  ${PROJECT_NAME}_proto
  ${PROJECT_NAME}_lib
  # ${OpenCV_LIBRARIES}
  gflags
)

add_subdirectory(proto)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/../..
)

file(GLOB SRC_PERCEPTION_BASE
  "${PROJECT_SOURCE_DIR}/*.cc"
  "${PROJECT_SOURCE_DIR}/graph/*.cc"
  "${PROJECT_SOURCE_DIR}/km_solver/*.cc"
  "${PROJECT_SOURCE_DIR}/registerer/*.cc"
  "${PROJECT_SOURCE_DIR}/sensor_manager/*.cc"
  "${PROJECT_SOURCE_DIR}/thread/*.cc"
  "${PROJECT_SOURCE_DIR}/kalman_filter/*.cc"
  "${PROJECT_SOURCE_DIR}/Hungarian/*.cc"
)

add_library(${PROJECT_NAME}_lib ${SRC_PERCEPTION_BASE})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} gflags yaml-cpp)

install(TARGETS ${PROJECT_NAME}_lib
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
