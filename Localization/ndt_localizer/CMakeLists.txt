cmake_minimum_required(VERSION 2.8.3)
project(ndt_localizer)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    diagnostic_msgs
    pcl_conversions
    pcl_ros
    message_generation
    can_unpack
)
find_package(PCL REQUIRED QUIET)
find_package(yaml-cpp REQUIRED QUIET)
find_library(YAML_CPP_LIBRARY NAMES ${YAML_CPP_LIBRARIES} PATHS ${YAML_CPP_LIBRARY_DIR})


# generate_messages(
#     DEPENDENCIES
#     std_msgs
# )

catkin_package(
    # INCLUDE_DIRS include
    # CATKIN_DEPENDS
    # roscpp
    # tf2
    # tf2_ros
    # tf2_geometry_msgs
    # geometry_msgs
    # sensor_msgs
    # nav_msgs
    # diagnostic_msgs
    # pcl_conversions
    # pcl_ros
    # message_generation
)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS})

###########
## Build ##
###########

add_executable(map_loader src/map_loader.cpp)
target_link_libraries(map_loader ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${YAML_CPP_LIBRARY})

add_executable(ndt_localizer_core src/ndt_localizer_core.cpp)
add_dependencies(ndt_localizer_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(ndt_localizer_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})
