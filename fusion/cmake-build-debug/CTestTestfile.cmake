# CMake generated Testfile for 
# Source directory: /home/mogo/data/catkin_ws/src/perception/fusion
# Build directory: /home/mogo/data/catkin_ws/src/perception/fusion/cmake-build-debug
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_perception_fusion2_gtest_unit_test "/home/mogo/data/catkin_ws/src/perception/fusion/cmake-build-debug/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/mogo/data/catkin_ws/src/perception/fusion/cmake-build-debug/test_results/perception_fusion2/gtest-unit_test.xml" "--return-code" "/home/mogo/data/catkin_ws/src/perception/fusion/cmake-build-debug/devel/lib/perception_fusion2/unit_test --gtest_output=xml:/home/mogo/data/catkin_ws/src/perception/fusion/cmake-build-debug/test_results/perception_fusion2/gtest-unit_test.xml")
set_tests_properties(_ctest_perception_fusion2_gtest_unit_test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/melodic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/melodic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/melodic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/mogo/data/catkin_ws/src/perception/fusion/CMakeLists.txt;112;catkin_add_gtest;/home/mogo/data/catkin_ws/src/perception/fusion/CMakeLists.txt;0;")
subdirs("gtest")
