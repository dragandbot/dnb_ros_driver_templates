cmake_minimum_required(VERSION 2.8.3)
project(dnb_driver_template_cpp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    robot_movement_interface
    roscpp
    rospy
    std_msgs
    std_srvs
    sensor_msgs
    industrial_msgs
    tf
    dnb_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS robot_movement_interface roscpp rospy std_msgs std_srvs tf dnb_msgs
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(driver_template_node
    src/driver_template.cpp
    src/individual_command_template.cpp
)
add_dependencies(driver_template_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(driver_template_node ${catkin_LIBRARIES} )

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(
    TARGETS driver_template_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY module_config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
