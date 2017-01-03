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

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   recordMsg.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
 )

catkin_package(
   LIBRARIES drone_sensor
   CATKIN_DEPENDS  message_runtime  drone_sensor
)

include_directories(
  .
  ./include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

set(SOURCE_FILES
        src/bag2csv/main.cpp
        src/bag2csv/recordMsg.h
        src/bag2csv/myprocesshandler.cpp
        config/param.yaml
)

add_executable(bag2csv_node ${SOURCE_FILES})

target_link_libraries(bag2csv_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${QT_LIBRARIES}
)

QT4_WRAP_CPP(bag2csv_gui_MOCS src/bag2csv_gui/uiwindow.h)
QT4_WRAP_UI(bag2csv_gui_UIS src/bag2csv_gui/mainwindow.ui)
ADD_EXECUTABLE(bag2csv_gui_node
    src/bag2csv_gui/main.cpp
    src/bag2csv_gui/uiwindow.cpp
    src/bag2csv_gui/recordMsg.h
    ${bag2csv_gui_MOCS}
    ${bag2csv_gui_UIS})
TARGET_LINK_LIBRARIES(bag2csv_gui_node
${catkin_LIBRARIES}
${OPENCV_LIBRARIES}
${QT_LIBRARIES})




