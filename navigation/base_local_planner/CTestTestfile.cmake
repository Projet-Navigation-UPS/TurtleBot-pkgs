# CMake generated Testfile for 
# Source directory: /home/projturtlebot/catkin_ws/src/navigation/base_local_planner
# Build directory: /home/projturtlebot/catkin_ws/src/navigation/base_local_planner
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
ADD_TEST(_ctest_base_local_planner_gtest_base_local_planner_utest "/home/projturtlebot/catkin_ws/src/navigation/base_local_planner/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/indigo/share/catkin/cmake/test/run_tests.py" "/home/projturtlebot/catkin_ws/src/navigation/base_local_planner/test_results/base_local_planner/gtest-base_local_planner_utest.xml" "--return-code" "/home/projturtlebot/catkin_ws/src/navigation/base_local_planner/devel/lib/base_local_planner/base_local_planner_utest --gtest_output=xml:/home/projturtlebot/catkin_ws/src/navigation/base_local_planner/test_results/base_local_planner/gtest-base_local_planner_utest.xml")
ADD_TEST(_ctest_base_local_planner_gtest_line_iterator "/home/projturtlebot/catkin_ws/src/navigation/base_local_planner/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/indigo/share/catkin/cmake/test/run_tests.py" "/home/projturtlebot/catkin_ws/src/navigation/base_local_planner/test_results/base_local_planner/gtest-line_iterator.xml" "--return-code" "/home/projturtlebot/catkin_ws/src/navigation/base_local_planner/devel/lib/base_local_planner/line_iterator --gtest_output=xml:/home/projturtlebot/catkin_ws/src/navigation/base_local_planner/test_results/base_local_planner/gtest-line_iterator.xml")
SUBDIRS(gtest)
