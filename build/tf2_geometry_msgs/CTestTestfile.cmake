# CMake generated Testfile for 
# Source directory: /home/pi/catkin_ws/src/geometry2/tf2_geometry_msgs
# Build directory: /home/pi/catkin_ws/build/tf2_geometry_msgs
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_tf2_geometry_msgs_gtest_test_tomsg_frommsg "/home/pi/catkin_ws/build/tf2_geometry_msgs/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/pi/catkin_ws/build/tf2_geometry_msgs/test_results/tf2_geometry_msgs/gtest-test_tomsg_frommsg.xml" "--return-code" "/home/pi/catkin_ws/devel/.private/tf2_geometry_msgs/lib/tf2_geometry_msgs/test_tomsg_frommsg --gtest_output=xml:/home/pi/catkin_ws/build/tf2_geometry_msgs/test_results/tf2_geometry_msgs/gtest-test_tomsg_frommsg.xml")
set_tests_properties(_ctest_tf2_geometry_msgs_gtest_test_tomsg_frommsg PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/pi/catkin_ws/src/geometry2/tf2_geometry_msgs/CMakeLists.txt;34;catkin_add_gtest;/home/pi/catkin_ws/src/geometry2/tf2_geometry_msgs/CMakeLists.txt;0;")
add_test(_ctest_tf2_geometry_msgs_rostest_test_test.launch "/home/pi/catkin_ws/build/tf2_geometry_msgs/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/pi/catkin_ws/build/tf2_geometry_msgs/test_results/tf2_geometry_msgs/rostest-test_test.xml" "--return-code" "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/pi/catkin_ws/src/geometry2/tf2_geometry_msgs --package=tf2_geometry_msgs --results-filename test_test.xml --results-base-dir \"/home/pi/catkin_ws/build/tf2_geometry_msgs/test_results\" /home/pi/catkin_ws/src/geometry2/tf2_geometry_msgs/test/test.launch ")
set_tests_properties(_ctest_tf2_geometry_msgs_rostest_test_test.launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/home/pi/catkin_ws/src/geometry2/tf2_geometry_msgs/CMakeLists.txt;43;add_rostest;/home/pi/catkin_ws/src/geometry2/tf2_geometry_msgs/CMakeLists.txt;0;")
add_test(_ctest_tf2_geometry_msgs_rostest_test_test_python.launch "/home/pi/catkin_ws/build/tf2_geometry_msgs/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/pi/catkin_ws/build/tf2_geometry_msgs/test_results/tf2_geometry_msgs/rostest-test_test_python.xml" "--return-code" "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/pi/catkin_ws/src/geometry2/tf2_geometry_msgs --package=tf2_geometry_msgs --results-filename test_test_python.xml --results-base-dir \"/home/pi/catkin_ws/build/tf2_geometry_msgs/test_results\" /home/pi/catkin_ws/src/geometry2/tf2_geometry_msgs/test/test_python.launch ")
set_tests_properties(_ctest_tf2_geometry_msgs_rostest_test_test_python.launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/home/pi/catkin_ws/src/geometry2/tf2_geometry_msgs/CMakeLists.txt;44;add_rostest;/home/pi/catkin_ws/src/geometry2/tf2_geometry_msgs/CMakeLists.txt;0;")
subdirs("gtest")
