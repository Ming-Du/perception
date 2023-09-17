cmake_minimum_required(VERSION 3.5)

set(CUR_LIB common)

set(CUR_SRCS "")
set(CUR_INCLUDES "include")

set(CUR_SUB_DIR "")
LIST(APPEND CUR_SUB_DIR include)
LIST(APPEND CUR_SUB_DIR src)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
	include
	${YAML_CPP_INCLUDE_DIRS}
)

foreach(dir ${CUR_SUB_DIR})
	file(GLOB_RECURSE tmp_srcs ${dir}/*.cpp ${dir}/*.cc ${dir}/*.h)
	list(APPEND CUR_SRCS ${tmp_srcs})
endforeach()

add_library(${RS_LIB_NAME_PREFIX}_${CUR_LIB} SHARED
	${CUR_SRCS}
)
target_include_directories(${RS_LIB_NAME_PREFIX}_${CUR_LIB}
	PUBLIC
	${CUR_INCLUDES}
	${RS_INCLUDE_DIRS}
)
target_link_libraries(${RS_LIB_NAME_PREFIX}_${CUR_LIB}
	PUBLIC
	${RS_LINKER_LIBS}
	yaml-cpp
	stdc++fs
)

set(RS_LIBS "")
list(APPEND RS_LIBS ${RS_LIB_NAME_PREFIX}_${CUR_LIB})
set(RS_PERCEPTION_LIDAR_PERCEPTION_LIBRARY ${RS_LIBS} PARENT_SCOPE)