cmake_minimum_required(VERSION 3.1)
project(camera_server)

# Subdirectory for the video application
add_subdirectory(nadjieb_mjpeg_streamer)

# ROS compilation options
find_package(catkin REQUIRED COMPONENTS)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp farol_gimmicks_library)


add_compile_options(-std=c++17 -Wall -O3)

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  farol_gimmicks_library
)

include_directories(
  ${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS} 
  include
)

add_executable(${PROJECT_NAME} src/CameraServerNode.cpp)
add_dependencies(${PROJECT_NAME} ${camera_server_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${catkin_LIBRARIES} ${OpenCV_LIBS} nadjieb_mjpeg_streamer::nadjieb_mjpeg_streamer)