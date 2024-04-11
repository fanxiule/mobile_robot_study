cmake_minimum_required(VERSION 3.0.2)
project(robot_sensor_model)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  roscpp
  sensor_msgs
)

find_package(Boost REQUIRED)
find_package(PCL REQUIRED)

catkin_package()

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(scan_to_pointcloud_converter src/scan2pointcloud_converter.cpp)
add_dependencies(scan_to_pointcloud_converter ${catkin_EXPORTED_TARGETS})
target_link_libraries(scan_to_pointcloud_converter ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)