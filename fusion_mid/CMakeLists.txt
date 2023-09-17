cmake_minimum_required(VERSION 3.18)

project(perception_fusion_mid)
# cuda
FIND_PACKAGE(CUDA REQUIRED)
set(CUDA_DIR "/usr/local/cuda")
INCLUDE_DIRECTORIES(/usr/local/cuda/include)
# set(CMAKE_CUDA_COMPILE /usr/local/cuda/bin)

find_library(CUDA_LIBRARY_CUBLASS cublas
        HINTS ${CUDA_DIR} 
        PATH_SUFFIXES lib lib64 lib/x64)
find_library(CUDA_LIBRARY_CUBLASSLT cublasLt
        HINTS ${CUDA_DIR} 
        PATH_SUFFIXES lib lib64 lib/x64)
find_library(CUDA_LIBRARY_CUDNN cudnn
        HINTS ${CUDA_DIR} 
        PATH_SUFFIXES lib lib64 lib/x64)
find_library(CUDA_LIBRARY_NVTOOLEXT nvToolsExt
        HINTS ${CUDA_DIR} 
        PATH_SUFFIXES lib lib64 lib/x64)
find_library(CUDA_LIBRARY_CUDART cudart
        HINTS ${CUDA_DIR} 
        PATH_SUFFIXES lib lib64 lib/x64)

set(CUDA_LIBRARY ${CUDA_LIBRARY_CUBLASS} 
	         ${CUDA_LIBRARY_CUBLASSLT} 
		 ${CUDA_LIBRARY_CUDNN}
		 ${CUDA_LIBRARY_NVTOOLEXT}
		 ${CUDA_LIBRARY_CUDART}
		 )
MESSAGE(STATUS "Find CUDA_LIBRARY libs at ${CUDA_LIBRARY}")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GEOS REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  autopilot_msgs
  pcl_ros
  tf2
  tf2_geometry_msgs
  image_transport
  sensor_msgs
  cv_bridge
  std_msgs
  perception_base
  common
)

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

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_subdirectory(frustum_rcnn)
add_subdirectory(proto)
set(ALL_LIBS
        trt_plugin
        frcnn
        )
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/..
)

file(GLOB SRC_PERCEPTION_FUSION_MID
${PROJECT_SOURCE_DIR}/nodes/mid_fusion_node.cc
${PROJECT_SOURCE_DIR}/nodes/fpnet_processing_unit.cc
${PROJECT_SOURCE_DIR}/common/*.cc
${PROJECT_SOURCE_DIR}/common/subscriber/*.cc
${PROJECT_SOURCE_DIR}/tracker/base/*.cc
${PROJECT_SOURCE_DIR}/tracker/common/*.cc
${PROJECT_SOURCE_DIR}/tracker/filter/*.cc
${PROJECT_SOURCE_DIR}/tracker/interface/*.cc
${PROJECT_SOURCE_DIR}/tracker/matcher/*.cc
${PROJECT_SOURCE_DIR}/tracker/mid_fusion_tracker/*.cc
${PROJECT_SOURCE_DIR}/../base/proto/*.cc
${PROJECT_SOURCE_DIR}/../lib/registerer/*.cc
${PROJECT_SOURCE_DIR}/../common/graph/*.cc
${PROJECT_SOURCE_DIR}/common/trigger/*.cc
)

add_executable(${PROJECT_NAME}_node
        ${SRC_PERCEPTION_FUSION_MID}
        )

add_dependencies(${PROJECT_NAME}_node
        ${catkin_EXPORTED_TARGETS}
        )
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

# SET(GCC_COVERAGE_COMPILE_FLAGS "-coverage -fprofile-arcs -ftest-coverage")
# SET(GCC_COVERAGE_LINK_FLAGS "-coverage -lgcov")
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
# SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")
# catkin_add_gtest(unit_test_fusion_mid ${PROJECT_SOURCE_DIR}/test/src/mid_fusion_ros_test.cc ${PROJECT_SOURCE_DIR}/test/src/utils_test.cc ${SRC_PERCEPTION_FUSION_MID})
# target_link_libraries(unit_test_fusion_mid ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${ALL_LIBS} gcov ${PROJECT_NAME}_proto)


target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${ALL_LIBS}
        glog
        ${PROJECT_NAME}_proto
        GEOS::geos
        )

install(TARGETS
${ALL_LIBS}
 ${PROJECT_NAME}_node
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
 PATTERN ".svn" EXCLUDE)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(DIRECTORY models/
       DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)
