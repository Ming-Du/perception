cmake_minimum_required(VERSION 2.8.3)
project(calib_check)

add_compile_options(-std=c++11 -g -O1)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
pcl_ros
tf2
tf2_geometry_msgs
image_transport
sensor_msgs
cv_bridge
std_msgs
common
perception_base
)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
CATKIN_DEPENDS
pcl_ros
tf2
tf2_geometry_msgs
image_transport
sensor_msgs
cv_bridge
std_msgs
perception_base
common
abseil_cpp
)

include_directories(
${catkin_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/../../..
${EIGEN3_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}
include
)

file(GLOB SRC_PERCEPTION_FUSION_MID
nodes/calib_check_node.cc
nodes/calib_check_ros.cc
${PROJECT_SOURCE_DIR}/../base/proto/perception_component.pb.cc
${PROJECT_SOURCE_DIR}/common/*.cc
${PROJECT_SOURCE_DIR}/common/*.cc
)

add_executable(${PROJECT_NAME}_node
       #  ${SOURCE_FILES}
       ${SRC_PERCEPTION_FUSION_MID}
       )

add_dependencies(${PROJECT_NAME}_node
       ${catkin_EXPORTED_TARGETS}
       )

target_link_libraries(${PROJECT_NAME}_node
       ${catkin_LIBRARIES}
       ${OpenCV_LIBRARIES}
       yaml-cpp
       )


install(TARGETS
       ${PROJECT_NAME}_node
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
       DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
       PATTERN ".svn" EXCLUDE)

install(DIRECTORY include/
       DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${PROJECT_NAME}/
       PATTERN ".svn" EXCLUDE
       )

install(TARGETS
  ${PROJECT_NAME}_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)