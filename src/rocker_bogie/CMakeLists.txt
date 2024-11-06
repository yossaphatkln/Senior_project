cmake_minimum_required(VERSION 3.0.2)
project(rocker_bogie)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

# Add this line to find the Gazebo package
find_package(gazebo REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# Link against Gazebo and ROS libraries
add_library(skid_steering_plugin src/skid_steering_plugin.cpp)
target_link_libraries(skid_steering_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
  LIBRARIES skid_steering_plugin
  CATKIN_DEPENDS roscpp geometry_msgs
)
