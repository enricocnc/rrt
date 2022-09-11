cmake_minimum_required(VERSION 3.0.2)
project(rrt)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  visualization_msgs
  tf2
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rrt
#  CATKIN_DEPENDS nav_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/rrt.cpp
  src/rrt_2D.cpp
  src/rrt_diff_drive.cpp
  src/dubins.cpp
)

add_executable(${PROJECT_NAME}_node src/rrt_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
