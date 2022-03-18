cmake_minimum_required(VERSION 3.0.2)
project(localization2D)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  nav_msgs
  std_msgs
  tf
  visualization_msgs
)


find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES localization2D
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs tf visualization_msgs nav_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

add_executable(rl_node src/RL2D.cpp)
target_link_libraries(rl_node
    ${catkin_LIBRARIES}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_LIBRARIES}
)

