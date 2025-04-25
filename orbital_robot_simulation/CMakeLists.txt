cmake_minimum_required(VERSION 3.8)
project(orbital_robot_simulation)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  pluginlib
  rclcpp
  rclcpp_lifecycle
  nav_msgs
  geometry_msgs
  trajectory_msgs
  visualization_msgs
  tf2_eigen
  ament_index_cpp
  control_toolbox
)

# Find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

set(INSTALL_SHARE_DIRS
  description
  description/ros2_control
  description/urdf
  description/rviz
  description/gazebo
  bringup
)

# Install
install(DIRECTORY ${INSTALL_INCLUDE_DIR} DESTINATION include/orbital_robot_simulation)
foreach(DIR IN LISTS INSTALL_SHARE_DIRS)
  install(DIRECTORY ${DIR} DESTINATION share/orbital_robot_simulation)
endforeach()

add_executable(joint_state_to_trajectory_node 
  src/joint_state_to_trajectory.cpp)
ament_target_dependencies(joint_state_to_trajectory_node ${THIS_PACKAGE_INCLUDE_DEPENDS} )

# include directories
target_include_directories(joint_state_to_trajectory_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:include>
)

# add_executable(effort_publisher src/effort_publisher.cpp)
# ament_target_dependencies(effort_publisher ${THIS_PACKAGE_INCLUDE_DEPENDS} )

#include directories
# target_include_directories(effort_publisher
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
#     $<INSTALL_INTERFACE:include>
# )

install(TARGETS
  # effort_publisher
  joint_state_to_trajectory_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
