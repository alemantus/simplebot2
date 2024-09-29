cmake_minimum_required(VERSION 3.8)
project(camera_package)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)  # Find OpenCV


# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}  # Include OpenCV headers
)

add_executable(camera_publisher src/camera_publisher.cpp)
target_link_libraries(camera_publisher
  cv_bridge::cv_bridge
  image_transport::image_transport
  opencv_imgcodecs
  rclcpp::rclcpp
  ${OpenCV_LIBS}  # Link OpenCV libraries
)

# Install C++ executables
install(TARGETS
  camera_publisher
  DESTINATION lib/${PROJECT_NAME})

# Python setup
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/camera_publisher_python.py
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(cv_bridge image_transport rclcpp sensor_msgs std_msgs)
ament_package()


