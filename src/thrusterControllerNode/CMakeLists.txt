cmake_minimum_required(VERSION 2.8.3)
project(thrusterControllerNode)

add_definitions(-std=c++11)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_AUTOMOC ON)

find_package(Qt5Core)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    custom_msgs
    librosqt
    angles)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
)

include_directories(${catkin_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS})
file(GLOB SOURCES src/*.cpp)
file(GLOB HEADERS includes/*.h)

add_executable(thrusterController ${SOURCES} ${HEADERS})
target_link_libraries(thrusterController ${catkin_LIBRARIES} ${Qt5Core_LIBRARIES})

