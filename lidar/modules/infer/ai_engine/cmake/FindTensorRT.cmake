include(FindPackageHandleStandardArgs)
unset(TENSORRT_FOUND)
set(TENSORRT_BUILD_DIR ${PROJECT_SOURCE_DIR}/ai_engine/tensorRT)
set(DEFAULT_RS_TRT_VER "V6")
set(RS_TRT_VER ${DEFAULT_RS_TRT_VER})
set(TENSORRT_VERSION "6.0.1")
set(TENSORRT_BUILD_DIR ${TENSORRT_BUILD_DIR}/${CMAKE_SYSTEM_PROCESSOR})
set(TRT_SYSTEM_LIB_DIR "")

set(TENSORRT_INCLUDE_DIR ${TENSORRT_BUILD_DIR}/include)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    if (CUDA_VERSION VERSION_LESS 11.0)
        set(RS_TRT_VER "V6")
    else()
        message(FATAL_ERROR "Unsupport CUDA VERSION ${CUDA_VERSION} for TensorRT-${TENSORRT_VERSION}, Support Version is 10.0, 10.1, 10.2")
    endif()

elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    if(CUDA_VERSION VERSION_EQUAL 10.0)
        set(RS_TRT_VER "V5")
        set(TENSORRT_VERSION "5.1.6")
        set(TENSORRT_INCLUDE_DIR ${TENSORRT_BUILD_DIR}/v5/include)
    elseif(CUDA_VERSION VERSION_EQUAL 10.2)
        set(RS_TRT_VER "V7")
        set(TENSORRT_VERSION "7.1.3")
        set(TENSORRT_INCLUDE_DIR ${TENSORRT_BUILD_DIR}/v7/include)
    else()
        message(FATAL_ERROR "Unsupport CUDA VERSION ${CUDA_VERSION}, Support Version is 10.0, 10.2")
    endif()
    set(TRT_SYSTEM_LIB_DIR /usr/lib/aarch64-linux-gnu)
endif()

add_definitions(-DRSPERCEPTION_TENSORRT_${RS_TRT_VER})

set(TRT_LIB_DIR ${TENSORRT_BUILD_DIR}/lib)

find_library(NVINFER_LIBRARY NAMES nvinfer PATHS ${TRT_LIB_DIR} ${TRT_SYSTEM_LIB_DIR} NO_DEFAULT_PATH)
find_library(NVPARSER_LIBRARY NAMES nvonnxparser PATHS ${TRT_LIB_DIR} ${TRT_SYSTEM_LIB_DIR} NO_DEFAULT_PATH)
if (RS_TRT_VER MATCHES "V5" OR RS_TRT_VER MATCHES "V6")
    find_library(NVPARSER_RUNTIME_LIBRARY NAMES nvonnxparser_runtime PATHS ${TRT_LIB_DIR} ${TENSORRT_BUILD_DIR}/v5/lib NO_DEFAULT_PATH)
elseif(RS_TRT_VER MATCHES "V7")
    set(NVINFER_PLUGIN_LIBRARY ${TENSORRT_BUILD_DIR}/v7/lib/libnvinfer_plugin.so)
    # find_library(NVINFER_PLUGIN_LIBRARY NAMES nvinfer_plugin PATHS ${TENSORRT_BUILD_DIR}/v7/lib NO_DEFAULT_PATH)
endif()

list(APPEND TENSORRT_LIBS ${NVPARSER_LIBRARY}
                          ${NVPARSER_RUNTIME_LIBRARY}
                          ${NVINFER_LIBRARY}
                          ${NVINFER_PLUGIN_LIBRARY})

find_package_handle_standard_args(TENSORRT DEFAULT_MSG TENSORRT_INCLUDE_DIR TENSORRT_LIBS)

# hide locals from GUI
mark_as_advanced(TENSORRT_INCLUDE_DIR)


