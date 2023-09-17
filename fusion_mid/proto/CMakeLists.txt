project(perception_fusion_mid_proto)

find_package(Protobuf REQUIRED)
set(proto_dir ${CMAKE_CURRENT_SOURCE_DIR})
file(GLOB proto_files "${proto_dir}/*.proto")

set(proto_gen_dir ${CMAKE_CURRENT_SOURCE_DIR})

# Create lists of files to be generated
set(proto_gen_cpp_files "")
foreach(proto_file ${proto_files})  
  get_filename_component(proto_name ${proto_file} NAME_WE)
  list(APPEND proto_gen_cpp_files 
    ${proto_gen_dir}/${proto_name}.pb.cc
  )
endforeach(proto_file ${proto_files})

# Run protoc and generate language-specific headers.
add_custom_command(
  OUTPUT ${proto_gen_cpp_files}
  COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
    --proto_path=${proto_gen_dir}
    --proto_path=${common_SOURCE_DIR}/proto
    --cpp_out=${proto_gen_dir} ${proto_files}
  DEPENDS ${PROTOBUF_PROTOC_EXECUTABLE} ${proto_files}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
set_source_files_properties(${proto_gen_cpp_files} PROPERTIES GENERATED TRUE)

include_directories(
  ${PROTOBUF_INCLUDE_DIR}
  ${common_SOURCE_DIR}/proto
)
add_library(${PROJECT_NAME} ${proto_gen_cpp_files})
target_link_libraries(${PROJECT_NAME}
  ${PROTOBUF_LIBRARY}
  #common_proto
)
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
  