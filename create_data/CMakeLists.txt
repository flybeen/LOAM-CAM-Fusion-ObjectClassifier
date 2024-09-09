cmake_minimum_required(VERSION 3.0.2)
project(create_data)

## Find catkin macros and libraries
## rospy, std_srvs, gazebo_msgs 등의 의존성을 찾습니다.
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_srvs
  gazebo_msgs
)

###################################
## catkin specific configuration ##
###################################
## 패키지의 종속성을 설정합니다
catkin_package(
  CATKIN_DEPENDS rospy std_srvs gazebo_msgs
)

###########
## Build ##
###########

## src 디렉토리에 있는 run.py 스크립트를 설치하는 설정입니다
catkin_install_python(PROGRAMS
  src/run.py  # 여기에 실제 파일 경로 및 이름을 명시합니다.
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Install ##
#############

## Python 스크립트를 설치합니다
## src 디렉토리의 run.py를 실행 파일로 설치합니다.
catkin_install_python(PROGRAMS
  src/run.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
