cmake_minimum_required(VERSION 3.18)

set(TARGET_NAME trt_plugin)
set(SHARED_TARGET ${TARGET_NAME})
set(STATIC_TARGET ${TARGET_NAME}_static)

#enable_language(CUDA)

set(PLUGIN_LIST FrcnnSegProposalPlugin
                CenterShiftCustomPlugin
                )

set(INFER_PLUGIN_SRC InferPlugin.cc checkMacrosPlugin.cc)
set(INFER_PLUGIN_LIB ${TENSORRT_LIBRARY})

foreach(PLUGIN_NAME ${PLUGIN_LIST})
add_subdirectory(${PLUGIN_NAME})
set(INFER_PLUGIN_LIB ${INFER_PLUGIN_LIB} ${PLUGIN_NAME}_static)
endforeach()

cuda_add_library(${SHARED_TARGET} SHARED ${INFER_PLUGIN_SRC})
target_link_libraries(${SHARED_TARGET} ${INFER_PLUGIN_LIB})
target_link_libraries(${SHARED_TARGET} cublas)
