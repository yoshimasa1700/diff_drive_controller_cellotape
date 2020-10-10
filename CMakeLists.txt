cmake_minimum_required(VERSION 3.0.0)
project(diff_drive_controller_cellotape)
find_package(catkin REQUIRED)

catkin_metapackage()

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp geometry_msgs)

include_directories(
  include
  /usr/local/include
  ${catkin_INCLUDE_DIRS})

add_executable(
  ${PROJECT_NAME}
  src/diff_drive_controller_cellotape.cpp)

link_directories(
  /usr/local/lib)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  pigpio
  pigpiod_if2
  rt
  )
