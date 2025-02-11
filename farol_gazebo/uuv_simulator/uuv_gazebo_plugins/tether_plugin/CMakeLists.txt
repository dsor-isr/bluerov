cmake_minimum_required(VERSION 3.1)
project(tether_plugin)

find_package(catkin REQUIRED COMPONENTS gazebo_dev)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

add_definitions(-std=c++17)

set(PLUGINS_LIST "")

catkin_package(
  INCLUDE_DIRS
    include
    ${CMAKE_CURRENT_BINARY_DIR}
  LIBRARIES
  tether_plugin
)

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${CMAKE_CURRENT_BINARY_DIR} # for generated messages
                    ${catkin_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIR})

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${EIGEN3_LIBRARY_DIRS})

###########
## Build ##
###########
add_library(tether_plugin
    src/TetherPlugin.cc
)
target_link_libraries(tether_plugin
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${Boost_LIBRARIES})

list(APPEND PLUGINS_LIST tether_plugin)

#############
## Install ##
#############

install(TARGETS ${PLUGINS_LIST}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN "*~" EXCLUDE
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN ".hh"
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.pb.*"
  PATTERN "*~" EXCLUDE
)