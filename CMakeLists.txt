cmake_minimum_required(VERSION 2.4.6)
project(ros_jetson_stats)

find_package(catkin REQUIRED rospy genmsg std_msgs)

catkin_python_setup()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate services in the 'srv' folder
add_service_files(
   FILES
   nvpmodel.srv
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
include_directories()

catkin_install_python(PROGRAMS scripts/jetson_stats.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Install param and launch folders
install(DIRECTORY launch param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
