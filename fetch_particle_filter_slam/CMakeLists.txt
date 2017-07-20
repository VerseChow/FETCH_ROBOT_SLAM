cmake_minimum_required(VERSION 2.8.3)
project(fetch_particle_filter_slam)

add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
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
find_package(Boost)

add_subdirectory (utils)

catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
#   CATKIN_DEPENDS roscpp nodelet
#   DEPENDS eigen opencv
)

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
)

target_link_libraries(slam
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  utils
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/datatypes
)

add_executable(Slam_Fetch
  ${CMAKE_CURRENT_SOURCE_DIR}/src/slam_fetch.cpp
)

target_link_libraries(Slam_Fetch
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  utils
  slam
)

