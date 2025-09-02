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
find_package(yaml-cpp REQUIRED)  # Find yaml-cpp for YAML parsing
find_package(fastcdr REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}  # Include OpenCV headers
  ${YAML_CPP_INCLUDE_DIRS}  # Include yaml-cpp headers (if needed)
)

# Add executable for C++ node
add_executable(camera_publisher src/camera_publisher.cpp)
target_link_libraries(camera_publisher
  cv_bridge::cv_bridge
  image_transport::image_transport
  opencv_imgcodecs  # OpenCV module for image compression (cv::imencode)
  rclcpp::rclcpp
  yaml-cpp  # Link yaml-cpp for YAML parsing
  ${OpenCV_LIBS}  # Link OpenCV libraries
)

add_executable(camera_compressed_publisher src/camera_compressed_publisher.cpp)
target_link_libraries(camera_compressed_publisher
  cv_bridge::cv_bridge
  image_transport::image_transport
  opencv_imgcodecs  # OpenCV module for image compression (cv::imencode)
  rclcpp::rclcpp
  ${OpenCV_LIBS}  # Link OpenCV libraries
)

# Install C++ executables
install(TARGETS
  camera_publisher
  camera_compressed_publisher
  DESTINATION lib/${PROJECT_NAME})

# Python setup (if using Python in your package)
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts (if any)
install(PROGRAMS
  scripts/camera_publisher_python.py
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch calibration_files
  DESTINATION share/${PROJECT_NAME}
)
# Export necessary dependencies
ament_export_include_directories(include)
ament_export_dependencies(cv_bridge image_transport rclcpp sensor_msgs std_msgs OpenCV yaml-cpp)
ament_package()
