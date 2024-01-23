cmake_minimum_required(VERSION 3.0.2)
project(path_generator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES path_generator
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)



include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/path_generator.cpp src/a_star.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
