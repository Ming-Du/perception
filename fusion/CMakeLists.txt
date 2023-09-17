cmake_minimum_required(VERSION 2.8.3)
project(perception_fusion2)

set(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
#add_compile_options(
#  -std=c++11
#  -O3
#  -g
#)

option(USE_DEBUG_FUSION "whether to use debug fusion node" OFF)
if (USE_DEBUG_FUSION)
  message("debug fusion mode")
  add_definitions(-DUSE_DEBUG_FUSION)
else()
  message("not use debug mode ${USE_DEBUG_FUSION}")
endif(USE_DEBUG_FUSION)

find_package(catkin REQUIRED COMPONENTS
  autopilot_msgs
  cv_bridge
  image_transport
  perception_base
  common
  pcl_ros
  abseil_cpp
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GEOS REQUIRED)

catkin_package(
  LIBRARIES ${OpenCV_LIBRARIES}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/..
  ${EIGEN3_INCLUDE_DIRS}
)
file(GLOB SRC_PERCEPTION_FUSION
  "${PROJECT_SOURCE_DIR}/base/*.cc"
  "${PROJECT_SOURCE_DIR}/common/*.cc"
  "${PROJECT_SOURCE_DIR}/common/extended_kalman_filter/*.cc"
  "${PROJECT_SOURCE_DIR}/common/standard_kalman_filter/*.cc"
  "${PROJECT_SOURCE_DIR}/common/unscented_kalman_filter/*.cc"
  "${PROJECT_SOURCE_DIR}/common/object_convertor/*.cc"

  "${PROJECT_SOURCE_DIR}/lib/data_association/hm_data_association/*.cc"

  "${PROJECT_SOURCE_DIR}/lib/data_fusion/existence_fusion/dst_existence_fusion/*.cc"
  "${PROJECT_SOURCE_DIR}/lib/data_fusion/motion_fusion/kalman_motion_fusion/*.cc"
  "${PROJECT_SOURCE_DIR}/lib/data_fusion/motion_fusion/ekf_motion_fusion/*.cc"
  "${PROJECT_SOURCE_DIR}/lib/data_fusion/shape_fusion/pbf_shape_fusion/*.cc"
  "${PROJECT_SOURCE_DIR}/lib/data_fusion/tracker/pbf_tracker/*.cc"
  "${PROJECT_SOURCE_DIR}/lib/gatekeeper/pbf_gatekeeper/proto/*.cc"
  "${PROJECT_SOURCE_DIR}/lib/data_fusion/type_fusion/dst_type_fusion/dst_type_fusion.cc"
  "${PROJECT_SOURCE_DIR}/lib/data_fusion/type_fusion/bayes_type_fusion/bayes_type_fusion.cc"
  "${PROJECT_SOURCE_DIR}/lib/data_fusion/tracker_optimizer/*.cc"


  "${PROJECT_SOURCE_DIR}/lib/dummy/*.cc"
  "${PROJECT_SOURCE_DIR}/lib/fusion_system/probabilistic_fusion/*.cc"
  "${PROJECT_SOURCE_DIR}/lib/gatekeeper/pbf_gatekeeper/*.cc"
  "${PROJECT_SOURCE_DIR}/lib/global_postprocessor/*.cc"  
  "${PROJECT_SOURCE_DIR}/lib/object_filter/*.cc"  
  "${PROJECT_SOURCE_DIR}/rviz_display/*.cc"
  "${PROJECT_SOURCE_DIR}/lib/event_process/*.cc"  

#  "${PROJECT_SOURCE_DIR}/../proto/*.cc"
  "${PROJECT_SOURCE_DIR}/../lib/config_manager/*.cc"
  "${PROJECT_SOURCE_DIR}/../lib/registerer/*.cc"
  "${PROJECT_SOURCE_DIR}/../lib/thread/*.cc"
  "${PROJECT_SOURCE_DIR}/../common/*.cc"
  "${PROJECT_SOURCE_DIR}/../common/graph/*.cc"
  "${PROJECT_SOURCE_DIR}/../common/io/*.cc"
#  "${PROJECT_SOURCE_DIR}/../../system/common/proto/*.cc"
)
file(GLOB SRC_PERCEPTION_FUSION_MAIN
  "${PROJECT_SOURCE_DIR}/app/*.cc"
)

file(GLOB SRC_PERCEPTION_FUSION_TEST
  "test/unit_test.cc"
  "${PROJECT_SOURCE_DIR}/lib/data_fusion/type_fusion/dst_type_fusion/dst_type_fusion_test.cc"
)

# add_library(${PROJECT_NAME}_lib ${SRC_PERCEPTION_FUSION})

add_executable(${PROJECT_NAME}_node ${SRC_PERCEPTION_FUSION_MAIN} ${SRC_PERCEPTION_FUSION})

#add temporarily - syf
add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")

if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    /usr/lib/aarch64-linux-gnu/libglog.so.0
  )
else()
  target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
  )
endif()

catkin_add_gtest(unit_test ${SRC_PERCEPTION_FUSION_TEST} ${SRC_PERCEPTION_FUSION})
target_link_libraries(unit_test ${catkin_LIBRARIES} GEOS::geos)

# target_link_libraries(${PROJECT_NAME}_lib
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${PCL_LIBRARIES}
#   glog
#   yaml-cpp
#   ${abseil_cpp_LIBRARIES}
# )

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  glog
  yaml-cpp
  ${abseil_cpp_LIBRARIES}
  GEOS::geos
)

# install(TARGETS
#   ${PROJECT_NAME}_lib
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

install(TARGETS
  ${PROJECT_NAME}_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
