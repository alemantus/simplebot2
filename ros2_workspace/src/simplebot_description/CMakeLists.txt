cmake_minimum_required(VERSION 3.8)
project(simplebot_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install robot description (URDF / Xacro / config)
install(
  DIRECTORY src/description
  DESTINATION share/${PROJECT_NAME}
)

# Install meshes
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

# Install config
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


# Install RViz configs
install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install RViz configs
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python executable
install(
  PROGRAMS
    simplebot_description/mecanum_joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mecanum_joint_state_publisher
)


ament_package()
