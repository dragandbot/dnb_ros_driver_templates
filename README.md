# README #

### What is this repository for? ###

* ROS drivers templates for creating new drivers:
* C++ template ROS package
* Python template ROS package

### How to use the C++ subpackage? ###

1. For a new driver copy the dnb_driver_template_cpp sub package to your workspace/src folder.
2. Modify the package directory, the CMakeLists.txt and package.xml. Rename all dnb_driver_template_cpp occurences to your new package name.
3. Modify the driver_template.cpp and driver_template.h files. Rename all DriverTemplate occurences to your new driver class name.
4. Modify the individual_command_template.cpp and individual_command_template.h files. Rename all IndividualCommandTemplate occurences to your individual movement command name.
5. Modify the driver_template.cpp and driver_template.h files. Rename all IndividualCommandTemplate occurences to your individual movement command class name.
6. Rename the driver_template.cpp, driver_template.h, individual_command_template.cpp and individual_command_template.h files corresponding to your new driver.
7. Adjust the .cpp filenames in the CMakeLists.txt respectively.
8. Run catkin build in the workspace. Until here no real modifications have been done, just renaming to your new driver. So it should compile without problems.
9. Modify the (former) driver_template.cpp and individual_command_template.cpp files to your individual system. Watch for <!> occurences and adjust these respective their descriptions. Possibly you will have to adjust the entire package to satisfy the robot controllers requirements.
10. Adjust launchfile and module_config files


### How to use the Python subpackage? ###

1. For a new driver copy the dnb_driver_template_python subpackage to your workspace/src folder.
2. Modify the package directory, the CMakeLists.txt and package.xml. Rename all dnb_driver_template_python occurences to your new package name.
3. Modify the individual_command_template.py file. Rename all IndividualCommandTemplate occurences to your individual movement command class name.
4. Modify the driver_template.py file. Rename all IndividualCommandTemplate occurences to your individual movement command class name.
5. Rename the driver_template.py and individual_command_template.py files corresponding to your new driver.
6. Run catkin build in the workspace. Until here no real modifications have been done, just renaming to your new driver. So it should compile without problems.
7. Modify the (former) driver_template.py and individual_command_template.py files to your individual system. Watch for <!> occurences and adjust these respective their descriptions. Possibly you will have to adjust the entire package to satisfy the robot controllers requirements.
8. Adjust launchfile and module_config files