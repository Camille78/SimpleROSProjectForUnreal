cmake_minimum_required(VERSION 2.8.3)
project(unrealROS)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)



catkin_package(CATKIN_DEPENDS )


