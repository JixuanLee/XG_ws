cmake_minimum_required(VERSION 2.8.3)
project(daheng_ros)
add_definitions(
    -std=c++11
)

find_package(catkin REQUIRED QUIET COMPONENTS
  roscpp
  rospy
  cv_bridge
  image_transport
  camera_info_manager
  sensor_msgs
)

find_package(OpenCV REQUIRED QUIET)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

find_library(DaHeng_LIBRRARIES NAMES gxiapi PATH /usr/lib)

add_executable(daheng src/daheng.cpp)
target_link_libraries(daheng ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${DaHeng_LIBRRARIES})


install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
