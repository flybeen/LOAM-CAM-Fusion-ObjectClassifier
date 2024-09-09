cmake_minimum_required(VERSION 3.0.2)
project(lidar_camera_calibration)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  tf
  tf2_ros
  tf_conversions
)

# Python 설정 추가
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs cv_bridge image_transport tf tf2_ros tf_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Python 스크립트를 설치 대상으로 설정
catkin_install_python(PROGRAMS src/lidar_camera_calibration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
