cmake_minimum_required(VERSION 2.8.3)
project(iai_qb_cube_driver)

find_package(catkin REQUIRED COMPONENTS
  qb_cube_interface
  roscpp)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iai_qb_cube_driver
#  CATKIN_DEPENDS qb_cube_interface roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS})

add_executable(iai_qb_cube_driver 
  src/iai_qb_cube_driver/Driver.cpp
  src/iai_qb_cube_driver/main.cpp)

target_link_libraries(iai_qb_cube_driver
  ${catkin_LIBRARIES})
