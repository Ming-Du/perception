cmake_minimum_required(VERSION 3.5)

add_compile_options(-W)
add_compile_options(-g)
add_compile_options(-O2)
add_compile_options(-std=c++11)

# ========================
# Project
# ========================
project(rs_perception_refiner)

set(CUR_SRCS "")
file(GLOB_RECURSE CUR_SRCS
	${PROJECT_SOURCE_DIR}/*.cpp
	${PROJECT_SOURCE_DIR}/*.h
)

include_directories(
	${CMAKE_SOURCE_DIR}/perception/lidar/common/include
	${CMAKE_SOURCE_DIR}/perception/lidar/modules/infer/include
	${CMAKE_SOURCE_DIR}/perception/lidar/modules/ai_refine/include
	${RS_PERCEPTION_GPU_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
	${CUR_SRCS}
)
target_include_directories(${PROJECT_NAME}
	PUBLIC
	${PROJECT_SOURCE_DIR}/include
	${RS_INCLUDE_DIRS}
)
add_dependencies(${PROJECT_NAME} rs_perception_common rs_perception_ai_refine)
target_link_libraries(${PROJECT_NAME}
	${CATKIN_DEVEL_PREFIX}/lib/librs_perception_common.so
	${CATKIN_DEVEL_PREFIX}/lib/librs_perception_ai_refine.so
	${RS_PERCEPTION_GPU_LINKER_LIBS}
)

set(RS_LIBS "")
list(APPEND RS_LIBS ${PROJECT_NAME})
set(RS_PERCEPTION_LIDAR_PERCEPTION_LIBRARY ${RS_LIBS} PARENT_SCOPE)