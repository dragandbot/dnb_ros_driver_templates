cmake_minimum_required(VERSION 2.8.3)
project(dnb_driver_template_python)

find_package(catkin REQUIRED COMPONENTS
  industrial_msgs
  robot_movement_interface
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  dnb_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(PROGRAMS
  scripts/driver_template.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY module_config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
