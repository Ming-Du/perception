set(CUR_LIB rviz_display)

set(CUR_SRCS "")
set(CUR_INCLUDES "include")

set(CUR_SUB_DIR "")
LIST(APPEND CUR_SUB_DIR include)
LIST(APPEND CUR_SUB_DIR src)

foreach(dir ${CUR_SUB_DIR})
	file(GLOB_RECURSE tmp_srcs ${dir}/*.cpp ${dir}/*.h)
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
	rs_perception_common
)