cmake_minimum_required(VERSION 3.0.2)
project (myCdd)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
)

FILE(GLOB_RECURSE headers "*.h" "*.hpp")
file(GLOB_RECURSE sources *.c)

add_library (myCdd SHARED ${headers} ${sources})
target_include_directories(myCdd PUBLIC   
                      ${PROJECT_SOURCE_DIR})



#install(TARGETS myCdd DESTINATION "${INSTALL_LIB_DIR}")
#install(FILES ${headers} DESTINATION
#    "${INSTALL_INCLUDE_DIR}/pnc_utils/ThirdParty/CDD")
