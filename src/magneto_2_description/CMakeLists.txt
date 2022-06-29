cmake_minimum_required(VERSION 2.8.3)
project(magneto_2_description)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
