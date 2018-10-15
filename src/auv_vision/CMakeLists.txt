cmake_minimum_required(VERSION 2.8.3)
project(auv_vision)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Helps with finding OpenCV's cmake configs. Uncomment to use locally built OpenCV.
## set(OpenCV_DIR /usr/local/share/OpenCV)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  cv_bridge
  sensor_msgs
  image_transport
)

## Find OpenCV
find_package(OpenCV)
## For locally built OpenCV, comment previous line and uncomment following:
## find_package(OpenCV REQUIRED HINTS ${OpenCV_DIR} NO_DEFAULT_PATH)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

#catkin_package(
#   CATKIN_DEPENDS message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES auv_vision
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
#)

###########
## Build ##
###########

include_directories(
  ## Uncomment to use locally built OpenCV:
  ## ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  "include"
)

add_executable(gate_locator
        src/gate_locator.cpp
        include/AbstractImageConverter.h
        src/AbstractImageConverter.cpp
        include/gate/GateDescriptor.h
        src/gate/GateDescriptor.cpp
        include/gate/GateDetector.h
        include/util/ImgprocPipeline.h
        src/util/ImgprocPipeline.cpp
        src/gate/GateDetector.cpp)
target_link_libraries(gate_locator ${catkin_LIBRARIES})
target_link_libraries(gate_locator ${OpenCV_LIBS})

add_executable(locators_handler
        src/locators_handler.cpp
        include/AbstractImageConverter.h
        src/AbstractImageConverter.cpp
        include/gate/GateDescriptor.h
        src/gate/GateDescriptor.cpp
        include/gate/GateDetector.h
        include/util/ImgprocPipeline.h
        src/util/ImgprocPipeline.cpp
        src/gate/GateDetector.cpp)
target_link_libraries(locators_handler ${catkin_LIBRARIES})
target_link_libraries(locators_handler ${OpenCV_LIBS})
