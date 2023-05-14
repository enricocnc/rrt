cmake_minimum_required(VERSION 3.0.2)
project(rrt)

include(cmake/compiler_flags.cmake)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  visualization_msgs
  tf2
  )
  
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/utils.cpp
  src/types.cpp
  src/dubins.cpp
  src/posq.cpp
  src/cubic_spline.cpp
  src/rrt_core.cpp
  src/rrt_ros_interface.cpp
  src/tree.cpp
  src/grid.cpp
  src/random_pose_generator.cpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 20)
add_compiler_flags(${PROJECT_NAME})

if(ASAN_ENABLED)
  message(STATUS "Enabling ASan...")
  target_compile_options(${PROJECT_NAME} PUBLIC -fsanitize=leak -fsanitize=address -fsanitize=undefined)
  target_link_options(${PROJECT_NAME} PUBLIC -fsanitize=leak -fsanitize=address  -fsanitize=undefined)
endif()

add_executable(${PROJECT_NAME}_node src/rrt_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

set_target_properties(${PROJECT_NAME}_node PROPERTIES CXX_STANDARD 20)
add_compiler_flags(${PROJECT_NAME}_node)

