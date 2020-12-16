cmake_minimum_required(VERSION 3.0.2)
project(my_pnc)

set(CMAKE_BUILD_TYPE "Release")
set(ROS_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  my_ros_msgs
)
find_package(Eigen3)

catkin_package(
  LIBRARIES my_pnc magneto_pnc_node
  CATKIN_DEPENDS my_ros_msgs #my_utils RobotSystem Filter my_wbc Geometry Planner
)

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(CMAKE_MODULE_PATH 
  ${CMAKE_MODULE_PATH} 
  ${PROJECT_SOURCE_DIR}/cmake)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

file(GLOB_RECURSE sources "*.cpp")
file(GLOB my_pnc_headers "src/*.hpp")
file(GLOB_RECURSE magneto_my_pnc_headers "src/Magneto/*.hpp")
file(GLOB pnc_node_source "src/MagnetoPncNode.cpp")
list(REMOVE_ITEM sources ${pnc_node_source})

# message("${pnc_node_source}")
# message("${sources}")
#message("${headers}")

add_library(my_pnc SHARED ${sources}
                              ${headers})                              
target_link_libraries(my_pnc ${DART_LIBRARIES}
                                  my_robot_system
                                  my_utils
                                  my_filter                                  
                                  my_wbc
                                  my_geometry)                                  
target_include_directories(my_pnc PUBLIC   
                            ${PROJECT_INCLUDE_DIR})


add_executable(magneto_pnc_node ${pnc_node_source})
target_link_libraries(magneto_pnc_node ${DART_LIBRARIES}
                                        ${catkin_LIBRARIES}
                                        my_pnc
                                        my_robot_system
                                        my_utils
                                        my_ros_msgs
                                        )

                            
# install(TARGETS my_pnc DESTINATION "${INSTALL_LIB_DIR}")
# install(FILES ${my_pnc_headers} DESTINATION
#     "${INSTALL_INCLUDE_DIR}/my_pnc")
