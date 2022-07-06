cmake_minimum_required(VERSION 3.0.2)
project(magneto_test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_BUILD_TYPE "Release")
find_package(Eigen3)
find_package(DART 6.9 REQUIRED COMPONENTS utils-urdf gui-osg CONFIG)
find_package(catkin REQUIRED
    roscpp
    rospy
    std_msgs
    sensor_msgs
    pnc_utils
    pnc_core
    magneto_pnc
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES magneto_sim
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
include_directories(
# include
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${DART_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(magneto_sim_lib SHARED
#     ${PROJECT_SOURCE_DIR}/src/magneto_dartsim.cpp
#     ${PROJECT_SOURCE_DIR}/include/magneto_test/magneto_dartsim.hpp
# )
# target_link_libraries(magneto_sim_lib 
#                     ${catkin_LIBRARIES}
#                     ${DART_LIBRARIES}
#                     ${EIGEN_LIBRARIES}
# )

# target_include_directories(magneto_sim_lib PUBLIC
# ${PROJECT_INCLUDE_DIR})

file(GLOB_RECURSE sim_headers "include/magneto_test/simulation/*.hpp")
file(GLOB_RECURSE sim_sources "src/simulation/*.cpp")

file(GLOB_RECURSE hw_headers "include/magneto_test/hwtest/*.hpp")
file(GLOB_RECURSE hw_sources "src/hwtest/*.cpp")

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(magneto_sim ${PROJECT_SOURCE_DIR}/src/main_sim.cpp)
add_executable(magneto_sim ${sim_sources} ${sim_headers})
target_link_libraries(magneto_sim                     
                    ${catkin_LIBRARIES}
                    ${DART_LIBRARIES}
                    ${EIGEN_LIBRARIES}
)

add_executable(magneto_hw ${hw_headers} ${hw_sources})
target_link_libraries(magneto_hw                     
                    ${catkin_LIBRARIES}
                    ${DART_LIBRARIES}
                    ${EIGEN_LIBRARIES}
)


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_magneto_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
