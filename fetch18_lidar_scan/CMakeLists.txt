cmake_minimum_required(VERSION 2.8.3)
project(fetch18_lidar_scan)

add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  geometry_msgs
  tf2_msgs
  pcl_ros	
  pcl_conversions
)
find_package(PCL 1.8 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

add_subdirectory (utils)

###########
## Build ##
###########
set(slam_files
  ${CMAKE_CURRENT_SOURCE_DIR}/src/mapping.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/action_model.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor_model.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/particle_filter.cpp
)
add_library (slam ${slam_files})

target_include_directories (slam PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/datatypes
)

target_link_libraries(slam
  ${catkin_LIBRARIES}
  utils
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/datatypes
)

add_executable(Slam_Fetch
  ${CMAKE_CURRENT_SOURCE_DIR}/src/slam_fetch.cpp
)

target_link_libraries(Slam_Fetch
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  utils
  slam
)

