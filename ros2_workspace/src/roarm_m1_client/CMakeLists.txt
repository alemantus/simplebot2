cmake_minimum_required(VERSION 3.8)
project(roarm_m1_client)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# --- C++ executables ---
# add_executable(your_cpp_node src/your_cpp_node.cpp)
# ament_target_dependencies(your_cpp_node rclcpp std_msgs)
# install(TARGETS
#   your_cpp_node
#   DESTINATION lib/${PROJECT_NAME})

# --- Python scripts ---
install(PROGRAMS
  scripts/m1_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
