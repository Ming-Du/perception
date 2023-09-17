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
project(rs_perception_infer)
set(CUR_LIB rs_perception_infer)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/ai_engine/cmake)
list(APPEND INFERENCE_INCLUDE_DIRS ${RS_PERCEPTION_GPU_INCLUDE_DIRS})
list(APPEND INFERENCE_LINKER_LIBS ${RS_PERCEPTION_GPU_LINKER_LIBS})
find_package(TensorRT 6.0 REQUIRED)

if(TENSORRT_FOUND)
    list(APPEND INFERENCE_INCLUDE_DIRS ${TENSORRT_INCLUDE_DIR})
    list(APPEND INFERENCE_LINKER_LIBS ${TENSORRT_LIBS})
else()
    message(FATAL_ERROR "Could NOT find TENSORRT")
endif()

set(CUR_SRCS "")
set(CUR_INCLUDES "")

set(CUR_SUB_DIR "")
LIST(APPEND CUR_SUB_DIR include)
LIST(APPEND CUR_SUB_DIR src)

foreach(dir ${CUR_SUB_DIR})
    file(GLOB_RECURSE tmp_srcs ${dir}/*.cpp ${dir}/*.h)
    list(APPEND CUR_INCLUDES include)
    list(APPEND CUR_SRCS ${tmp_srcs})
endforeach()

add_library(${CUR_LIB} SHARED
    ${CUR_SRCS}
)
target_include_directories(${CUR_LIB}
    PUBLIC
    ${CUR_INCLUDES}
    ${INFERENCE_INCLUDE_DIRS}
)
target_link_libraries(${CUR_LIB}
    PUBLIC
    rs_perception_common
    ${INFERENCE_LINKER_LIBS}
)

set(RS_LIBS "")
list(APPEND RS_LIBS ${CUR_LIB})
set(RS_PERCEPTION_INFER_LIBRARY ${RS_LIBS} PARENT_SCOPE)
