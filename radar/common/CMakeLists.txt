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

foreach (dir ${CUR_SUB_DIR})
    file(GLOB_RECURSE tmp_srcs ${dir}/*.cpp ${dir}/*.h)
    list(APPEND CUR_SRCS ${tmp_srcs})
endforeach ()

add_library(${RADAR_LIB_NAME_PREFIX}_${CUR_LIB} SHARED
        ${CUR_SRCS}
        )
target_include_directories(${RADAR_LIB_NAME_PREFIX}_${CUR_LIB}
        PUBLIC
        ${CUR_INCLUDES}
        ${RADAR_INCLUDE_DIRS}
        )
target_link_libraries(${RADAR_LIB_NAME_PREFIX}_${CUR_LIB}
        PUBLIC
        ${RADAR_INCLUDE_DIRS}
        yaml-cpp
        )
install(TARGETS ${RADAR_LIB_NAME_PREFIX}_${CUR_LIB}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )
