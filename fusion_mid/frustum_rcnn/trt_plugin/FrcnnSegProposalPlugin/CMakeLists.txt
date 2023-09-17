set(TARGET_NAME FrcnnSegProposalPlugin)
set(SHARED_TARGET ${TARGET_NAME})
set(STATIC_TARGET ${TARGET_NAME}_static)
#find_package(CUDA REQUIRED)
#enable_language(CUDA)

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} --compiler-options -fPIC)

file(GLOB src *.cc *.cu)

cuda_add_library(${STATIC_TARGET} STATIC ${src})
target_link_libraries(${STATIC_TARGET} ${CUDA_LIBRARY})
target_link_libraries(${STATIC_TARGET} ${TENSORRT_LIBRARY})

set_property(TARGET ${STATIC_TARGET} PROPERTY CUDA_ARCHITECTURES ${GPU_ARCHS})
