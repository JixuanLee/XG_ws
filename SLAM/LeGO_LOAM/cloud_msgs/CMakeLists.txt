cmake_minimum_required(VERSION 2.8.3)
project(cloud_msgs)


find_package(catkin REQUIRED QUIET COMPONENTS
  roscpp
  message_generation
  std_msgs
)

add_message_files(
  FILES
  cloud_info.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS 
  roscpp
  message_runtime 
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)