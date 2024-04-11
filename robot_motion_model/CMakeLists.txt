cmake_minimum_required(VERSION 3.0.2)
project(robot_motion_model)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  angles
  geometry_msgs
  nav_msgs
  roscpp
  tf2
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ground_truth_path src/ground_truth_path.cpp)
add_dependencies(ground_truth_path ${catkin_EXPORTED_TARGETS})
target_link_libraries(ground_truth_path ${catkin_LIBRARIES})

add_executable(kinematic_model src/kinematic_model.cpp)
add_dependencies(kinematic_model ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinematic_model ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)