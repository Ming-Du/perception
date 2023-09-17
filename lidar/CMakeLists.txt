cmake_minimum_required(VERSION 3.0)
project(rs_perception_node)

set(CMAKE_BUILD_TYPE "Release")
add_definitions(-std=c++17)
add_definitions(-DDEBUG_LEVEL=0)
add_compile_options(-std=c++17)
add_compile_options(-W)
add_compile_options(-g)
add_compile_options(-O2)
add_compile_options(-lgflags)

cmake_policy(SET CMP0022 NEW)

if(POLICY CMP0048)
	cmake_policy(SET CMP0048 NEW)
endif()

if(NOT CMAKE_COMPILER_IS_GNUCXX)
	message(FATAL_ERROR "not gnu cpp compiler!")
endif()

if(NOT UNIX)
	message(FATAL_ERROR "not unix system")
endif()

find_package(catkin REQUIRED COMPONENTS
	autopilot_msgs
	roscpp
	rospy
	std_msgs
	visualization_msgs
	common
)

catkin_package(LIBRARIES gflags)

find_package(OpenMP REQUIRED)

if(OPENMP_FOUND)
	message("OPENMP FOUND")
	set(CMAKE_CXX_STANDARD 17)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")
set(RS_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/perception/lidar")
set(RS_LINKER_LIBS "")
set(RS_LIB_NAME_PREFIX rs_perception)

include_directories(${catkin_INCLUDE_DIRS}
	${RS_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}/../../
)

# =============================
# add dependence
# =============================
include(cmake/dependence.cmake)
include(cmake/cuda.cmake)

# =============================
# add modules
# =============================
add_subdirectory(common)
add_subdirectory(modules/util)
add_subdirectory(modules/denoise)
add_subdirectory(modules/bev_builder)
add_subdirectory(modules/roi_filter)
add_subdirectory(modules/preprocessor)
add_subdirectory(modules/ground_filter)
add_subdirectory(modules/segmentor)
add_subdirectory(modules/tracker)
add_subdirectory(modules/infer)
add_subdirectory(modules/ai_detection)
add_subdirectory(modules/ai_refine)
add_subdirectory(modules/refiner)
add_subdirectory(modules/rviz_display)
add_subdirectory(application)

# =============================
# add node
# =============================
set(ALL_LIBS
	${RS_LIB_NAME_PREFIX}_util
	${RS_LIB_NAME_PREFIX}_infer
	${RS_LIB_NAME_PREFIX}_common
	${RS_LIB_NAME_PREFIX}_rviz_display
	${RS_LIB_NAME_PREFIX}_application
	${RS_LIB_NAME_PREFIX}_ai_detection
	${RS_LIB_NAME_PREFIX}_ai_refine
	${RS_LIB_NAME_PREFIX}_preprocessor
	${RS_LIB_NAME_PREFIX}_bev_builder
	${RS_LIB_NAME_PREFIX}_roi_filter
	${RS_LIB_NAME_PREFIX}_ground_filter
	${RS_LIB_NAME_PREFIX}_segmentor
	${RS_LIB_NAME_PREFIX}_denoise
	${RS_LIB_NAME_PREFIX}_tracker
	${RS_LIB_NAME_PREFIX}_refiner
)

add_executable(rs_perception_node node/rs_perception_node.cpp)
target_link_libraries(rs_perception_node PRIVATE
	${ALL_LIBS}
	${catkin_LIBRARIES}
	gflags
)

install(TARGETS ${PROJECT_NAME} ${ALL_LIBS}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY scripts
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})	
install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY modules/infer/ai_engine/tensorRT/aarch64/v7/
	DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/../)
install(FILES modules/denoise/include/xgboost_lib/libxgboost.so
	DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/)
