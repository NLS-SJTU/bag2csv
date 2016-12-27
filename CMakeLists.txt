cmake_minimum_required(VERSION 2.8.3)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(bag2csv)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  mavros_msgs
)

SET(CMAKE_CXX_FLAGS -std=c++11)

find_package(Qt4 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt4 COMPONENTS QTCORE QTGUI)

ADD_DEFINITIONS (${QT_DEFINITIONS})

catkin_package(
   LIBRARIES drone_sensor
   CATKIN_DEPENDS  message_runtime  drone_sensor
)

include_directories(
  .
  ./include
  ./config
  ./src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

set(SOURCE_FILES
        src/main.cpp
        config/param.yaml
)

add_executable(bag2csv ${SOURCE_FILES})

target_link_libraries(bag2csv
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

