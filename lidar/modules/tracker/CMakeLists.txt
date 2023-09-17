cmake_minimum_required(VERSION 3.5)

add_compile_options(-W)
add_compile_options(-g)
add_compile_options(-O2)
add_compile_options(-std=c++11)

# ========================
# Project
# ========================
project(rs_perception_tracker)

set(CUR_SRCS "")
set(CUR_INCLUDES "")

set(CUR_SUB_DIR "")
list(APPEND CUR_SUB_DIR include)
list(APPEND CUR_SUB_DIR src)

foreach(dir ${CUR_SUB_DIR})
	file(GLOB_RECURSE tmp_srcs ${dir}/*.cpp ${dir}/*.h)
	list(APPEND CUR_INCLUDES include)
	list(APPEND CUR_SRCS ${tmp_srcs})
endforeach()

add_library(${PROJECT_NAME} SHARED ${CUR_SRCS})
target_include_directories(${PROJECT_NAME} PUBLIC
	../util/debug_log_info/include
	${CUR_INCLUDES}
)
target_link_libraries(${PROJECT_NAME} PUBLIC rs_perception_common)

set(RS_LIBS "")
list(APPEND RS_LIBS ${PROJECT_NAME})
set(RS_PERCEPTION_LIDAR_PERCEPTION_LIBRARY ${RS_LIBS} PARENT_SCOPE)
