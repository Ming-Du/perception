cmake_minimum_required(VERSION 3.5)

add_compile_options(-W)
add_compile_options(-g)
add_compile_options(-O2)
add_compile_options(-std=c++17)

# ========================
# Project
# ========================
project(rs_perception_bev_builder)

find_package(Boost REQUIRED)

set(CUR_SRCS "")
file(GLOB_RECURSE CUR_SRCS
	${PROJECT_SOURCE_DIR}/*.cpp
	${PROJECT_SOURCE_DIR}/*.h
)

add_library(${PROJECT_NAME} SHARED
	${CUR_SRCS}
)
target_include_directories(${PROJECT_NAME}
	PUBLIC
	${PROJECT_SOURCE_DIR}/include
	${RS_INCLUDE_DIRS}/common
    ${Boost_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
	rs_perception_common
    ${Boost_LIBRARIES}
)

set(RS_LIBS "")
list(APPEND RS_LIBS ${PROJECT_NAME})
set(RS_PERCEPTION_LIDAR_PERCEPTION_LIBRARY ${RS_LIBS} PARENT_SCOPE)