cmake_minimum_required(VERSION 3.18)

project(FRCNN)

INCLUDE_DIRECTORIES(./include)
INCLUDE_DIRECTORIES(./trt_include)


# cuda
FIND_PACKAGE(CUDA REQUIRED)
set(CUDA_DIR "/usr/local/cuda")
INCLUDE_DIRECTORIES(/usr/local/cuda/include)


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


# tensorrt
set(TENSORRT_DIR lib/)
set(TENSORRT_BUILD /usr/lib/aarch64-linux-gnu)

#set(GPU_ARCHS 72)

if (NOT DEFINED TENSORRT_DIR)
  set(TENSORRT_DIR "$ENV{HOME}/SDK/TensorRT")
endif()
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
        HINTS ${TENSORRT_DIR} ${CUDA_TOOLKIT_ROOT_DIR}
        PATH_SUFFIXES include)
MESSAGE(STATUS "Found TensorRT headers at ${TENSORRT_INCLUDE_DIR}")

find_library(TENSORRT_LIBRARY_INFER nvinfer
        HINTS ${TENSORRT_DIR} ${TENSORRT_BUILD} ${CUDA_TOOLKIT_ROOT_DIR}
        PATH_SUFFIXES lib lib64 lib/x64)

find_library(TENSORRT_LIBRARY_PARSERS nvparsers
        HINTS ${TENSORRT_DIR} ${TENSORRT_BUILD} ${CUDA_TOOLKIT_ROOT_DIR}
        PATH_SUFFIXES lib lib64 lib/x64)

find_library(TENSORRT_LIBRARY_ONNXPARSER nvonnxparser
        HINTS  ${TENSORRT_DIR} ${TENSORRT_BUILD} ${CUDA_TOOLKIT_ROOT_DIR}
        PATH_SUFFIXES lib lib64 lib/x64 out)

set(TENSORRT_LIBRARY ${TENSORRT_LIBRARY_INFER}
                     ${TENSORRT_LIBRARY_PARSERS}
                     ${TENSORRT_LIBRARY_ONNXPARSER}
                     )
MESSAGE(STATUS "Find TensorRT libs at ${TENSORRT_LIBRARY}")


INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include/common)
LINK_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_definitions(-std=c++14)
add_definitions(-fPIC)
add_definitions(-Wno-unused-parameter)
set(CMAKE_BUILD_TYPE Debug)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O0")


# CUDA targets
if (DEFINED GPU_ARCHS)
  message(STATUS "GPU_ARCHS defined as ${GPU_ARCHS}. Generating CUDA code for SM ${GPU_ARCHS}")
  separate_arguments(GPU_ARCHS)
else()
  list(APPEND GPU_ARCHS
      61
      70
      75
    )

  string(REGEX MATCH "aarch64" IS_ARM "${TRT_PLATFORM_ID}")
  if (IS_ARM)
    # Xavier (SM72) only supported for aarch64.
    list(APPEND GPU_ARCHS 72)
  endif()

  if (CUDA_VERSION VERSION_GREATER_EQUAL 11.0)
    # Ampere GPU (SM80) support is only available in CUDA versions > 11.0
    list(APPEND GPU_ARCHS 80)
  else()
    message(WARNING "Detected CUDA version is < 11.0. SM80 not supported.")
  endif()

  message(STATUS "GPU_ARCHS is not defined. Generating CUDA code for default SMs: ${GPU_ARCHS}")
endif()
# Generate SASS for each architecture
foreach(arch ${GPU_ARCHS})
    set(GENCODES "${GENCODES} -gencode arch=compute_${arch},code=sm_${arch}")
endforeach()
# Generate PTX for the last architecture in the list.
list(GET GPU_ARCHS -1 LATEST_SM)
set(GENCODES "${GENCODES} -gencode arch=compute_${LATEST_SM},code=compute_${LATEST_SM}")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcompiler -Wno-deprecated-declarations")

set(CMAKE_CUDA_ARCHITECTURES ${GPU_ARCHS})

add_subdirectory(trt_plugin)

file(GLOB frcnn_src ./src/*.cc ./src/*.cu)
file(GLOB trt_src ./trt_src/*.cc ./trt_src/*.cu)
include_directories(include ${catkin_INCLUDE_DIRS} )
cuda_add_library(frcnn SHARED ${frcnn_src} ${trt_src})
target_link_libraries(frcnn yaml-cpp  trt_plugin)
