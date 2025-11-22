cmake_minimum_required(VERSION 3.22)
project(roarm_moveit_servo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(roarm_msgs REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen3 REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  geometry_msgs
  control_msgs
  sensor_msgs
  roarm_msgs
  Eigen3
  Boost
)

# Include directories for C++
include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${THIS_PACKAGE_INCLUDE_DIRS}
)

# C++ executable
add_executable(keyboardcontrol src/keyboardcontrol.cpp)
target_include_directories(keyboardcontrol PUBLIC include)
ament_target_dependencies(keyboardcontrol ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

# Install C++ node
install(TARGETS keyboardcontrol
  DESTINATION lib/${PROJECT_NAME}
)

# Python scripts
install(PROGRAMS
  scripts/joystickcontrol.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch/config/rviz folders
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
