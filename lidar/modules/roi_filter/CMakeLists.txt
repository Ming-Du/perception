cmake_minimum_required(VERSION 3.5)

add_compile_options(-W)
add_compile_options(-g)
add_compile_options(-O2)
add_compile_options(-std=c++11)

# ========================
# Project
# ========================
project(rs_perception_roi_filter)

set(CUR_SRCS "")
file(GLOB_RECURSE CUR_SRCS
	${PROJECT_SOURCE_DIR}/*.cpp
	${PROJECT_SOURCE_DIR}/*.h
)

find_package(OpenCV REQUIRED)

add_library(${PROJECT_NAME} SHARED
	${CUR_SRCS}
)
target_include_directories(${PROJECT_NAME}
	PUBLIC
	${PROJECT_SOURCE_DIR}/include
	${RS_INCLUDE_DIRS}/common
	${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
	rs_perception_common
	${OpenCV_LIBS}
)

set(RS_LIBS "")
list(APPEND RS_LIBS ${PROJECT_NAME})
set(RS_PERCEPTION_LIDAR_PERCEPTION_LIBRARY ${RS_LIBS} PARENT_SCOPE)
