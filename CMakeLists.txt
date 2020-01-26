cmake_minimum_required(VERSION 2.4.6)
project(ros_jetson_stats)

find_package(catkin REQUIRED rospy)

catkin_python_setup()

catkin_package()

include_directories()

catkin_install_python(PROGRAMS src/jetson_stats.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})