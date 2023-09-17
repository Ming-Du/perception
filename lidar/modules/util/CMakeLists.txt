cmake_minimum_required(VERSION 3.5)

set(CMAKE_BUILD_TYPE Release)
add_definitions(-std=c++11)
add_compile_options(-W)
add_compile_options(-g)
add_compile_options(-O2)
add_compile_options(-std=c++11)
add_definitions(-DDEBUG_LEVEL=0)

# ========================
# Project
# ========================
project(rs_perception_util)
set(CUR_LIB rs_perception_util)

set(CUR_SRCS "")
set(CUR_INCLUDES "")
set(CUR_SUB_DIR "")

list(APPEND CUR_SUB_DIR pre_fusion)
list(APPEND CUR_SUB_DIR post_fusion)
list(APPEND CUR_SUB_DIR basic_detection)
list(APPEND CUR_SUB_DIR debug_log_info)
list(APPEND CUR_SUB_DIR trigger)
list(APPEND CUR_SUB_DIR util)
list(APPEND CUR_SUB_DIR /usr/local/include)

foreach(dir ${CUR_SUB_DIR})
	file(GLOB_RECURSE tmp_srcs ${dir}/*.cpp ${dir}/*.h)
	list(APPEND CUR_INCLUDES ${dir}/include)
	list(APPEND CUR_SRCS ${tmp_srcs})
endforeach()

find_package(OpenCV REQUIRED)

add_library(${CUR_LIB} SHARED ${CUR_SRCS})
target_include_directories(
	${CUR_LIB}
	PUBLIC
	${PROJECT_SOURCE_DIR}/../tracking/include
	${CUR_INCLUDES}
	${OpenCV_INCLUDE_DIRS}
	${OpenCV_LIBRARY_DIRS}
)
target_link_libraries(${CUR_LIB}
	PUBLIC
	rs_perception_common
	rs_perception_tracker
	${OpenCV_LIBS}
)

set(RS_LIBS "")
list(APPEND RS_LIBS ${CUR_LIB})
set(RS_PERCEPTION_LIDAR_PERCEPTION_LIBRARY ${RS_LIBS} PARENT_SCOPE)
