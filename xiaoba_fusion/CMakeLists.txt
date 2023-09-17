cmake_minimum_required(VERSION 3.0.2)
project(xiaoba_fusion)

add_compile_options(
    -std=c++17
    -O3
    -Wno-ignored-attributes
    -Wno-deprecated-declarations
)

find_package(catkin REQUIRED COMPONENTS
  perception_base
  sensor_msgs
  autopilot_msgs
  pcl_ros
  pcl_conversions
  common
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../../
  # ../../system
)

add_library(${PROJECT_NAME}
  lidar_data_monitor.h
  lidar_data_monitor.cc
  lidar_data_manager.h
  lidar_data_manager.cc
  lidar_fault_reporter.h
  lidar_fault_reporter.cc
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_node
  lidar_data_fusion.h
  lidar_data_fusion.cc
)
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
#   ${PROJECT_NAME}
#   glog
# )

if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
	target_link_libraries(${PROJECT_NAME}_node
		${catkin_LIBRARIES}
    ${PROJECT_NAME}
    # gomp
    # yaml-cpp
		/usr/lib/aarch64-linux-gnu/libglog.so.0
	)
else()
	target_link_libraries(${PROJECT_NAME}_node
		${catkin_LIBRARIES}
    ${PROJECT_NAME}
    # gomp
    # yaml-cpp
	)
endif()

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_node
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
    lidars_fusion.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)