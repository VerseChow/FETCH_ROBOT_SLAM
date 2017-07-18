add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
)

add_library (utils ${CMAKE_CURRENT_SOURCE_DIR}/utils.cpp)

target_include_directories (utils PUBLIC
				${CMAKE_CURRENT_SOURCE_DIR}
				${CMAKE_CURRENT_SOURCE_DIR}/datatypes)