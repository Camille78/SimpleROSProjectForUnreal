cmake_minimum_required(VERSION 2.8.3)
project(custom_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  geographic_msgs
  message_generation
)

## Generate messages in the 'msg' folder

file (GLOB MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/msg "msg/*.msg")
file (GLOB MSG_FILES_ABS "msg/*.msg")

#file (GLOB SRV_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/srv "srv/*.srv")
#file (GLOB SRV_FILES_ABS "srv/*.srv")

add_message_files(
   FILES ${MSG_FILES}
)

#add_service_files(
#   FILES ${SRV_FILES}
#)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  geographic_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs message_runtime geographic_msgs
)

## Build ##

include_directories(
  ${catkin_INCLUDE_DIRS}
)
