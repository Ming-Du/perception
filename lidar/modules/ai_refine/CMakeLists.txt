cmake_minimum_required(VERSION 3.5)

set(CMAKE_BUILD_TYPE Release)
add_definitions(-std=c++11)
add_compile_options(-O2)
add_compile_options(-W)
add_compile_options(-g)
add_compile_options(-O2)
add_compile_options(-std=c++11)
add_definitions(-DDEBUG_LEVEL=0)

# ========================
# Project
# ========================
project(rs_perception_ai_refine)
set(CUR_LIB rs_perception_ai_refine)

# ========================
# cuda libs
# ========================
set(AI_REFINE_LINKER_LIBS "")
set(AI_REFINE_INCLUDE_DIRS "")

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-arch=${NVCC_FLAGS_EXTRA_readable};-std=c++11;--ptxas-options=-v;-g)
list(APPEND AI_REFINE_INCLUDE_DIRS ${RS_PERCEPTION_GPU_INCLUDE_DIRS})
list(APPEND AI_REFINE_LINKER_LIBS ${RS_PERCEPTION_GPU_LINKER_LIBS})

set(CUR_SRCS "")
set(CUR_INCLUDES "")

set(CUR_SUB_DIR "")
LIST(APPEND CUR_SUB_DIR include)
LIST(APPEND CUR_SUB_DIR src)

foreach(dir ${CUR_SUB_DIR})
	file(GLOB_RECURSE tmp_srcs ${dir}/*.cpp ${dir}/*.cu ${dir}/*.h)
	list(APPEND CUR_INCLUDES include)
	list(APPEND CUR_SRCS ${tmp_srcs})
endforeach()

cuda_add_library(${CUR_LIB} SHARED
	${CUR_SRCS}
)
target_include_directories(${CUR_LIB}
	PUBLIC
	${CUR_INCLUDES}
	${AI_REFINE_INCLUDE_DIRS}
)
target_link_libraries(${CUR_LIB}
	rs_perception_common
	rs_perception_infer
	${AI_REFINE_LINKER_LIBS}
)

set(RS_LIBS "")
list(APPEND RS_LIBS ${CUR_LIB})
set(RS_PERCEPTION_LIDAR_PERCEPTION_LIBRARY ${RS_LIBS} PARENT_SCOPE)
