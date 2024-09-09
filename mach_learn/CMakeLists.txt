cmake_minimum_required(VERSION 3.0.2)
project(mach_learn)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark Python scripts for installation
## This will install the Python script 'knn_run.py' located in 'src/' to the appropriate ROS package bin directory
catkin_install_python(PROGRAMS
  src/knn_run.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries (if using C++)
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mach_learn.cpp)

# Add folders to be
