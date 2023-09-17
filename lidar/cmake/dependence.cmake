
#=============================
# find Eigen
#=============================
find_package(Eigen3 REQUIRED)
list(APPEND RS_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})

#=============================
# find pcl
#=============================
find_package(PCL REQUIRED)
list(APPEND RS_INCLUDE_DIRS ${PCL_INCLUDE_DIRS})
list(APPEND RS_LINKER_LIBS ${PCL_LIBRARIES})

#=============================
# find ros
#=============================
find_package(tf QUIET)
find_package(pcl_ros QUIET)
find_package(roscpp 1.12 QUIET)
list(APPEND RS_INCLUDE_DIRS ${roscpp_INCLUDE_DIRS})
list(APPEND RS_INCLUDE_DIRS ${tf_INCLUDE_DIRS})
list(APPEND RS_INCLUDE_DIRS ${pcl_ros_INCLUDE_DIRS})
list(APPEND RS_LINKER_LIBS ${roscpp_LIBRARIES})
list(APPEND RS_LINKER_LIBS ${tf_LIBRARIES})
list(APPEND RS_LINKER_LIBS ${pcl_ros_LIBRARIES})