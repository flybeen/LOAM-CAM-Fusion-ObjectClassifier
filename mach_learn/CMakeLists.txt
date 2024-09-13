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
## Install both KNN and CNN scripts
catkin_install_python(PROGRAMS
  src/knn_run.py  # 기존 KNN 스크립트
  src/pcl_cnn_run.py  # 새로 추가된 CNN 스크립트
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries (if using C++)
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mach_learn.cpp)

# Add folders to be installed
