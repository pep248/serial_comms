cmake_minimum_required(VERSION 3.8)
project(serial_communication)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# DEPENDENCIES

# Find packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# PYTHON
# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/gps_node.py
  scripts/imu_node.py
  DESTINATION lib/${PROJECT_NAME}
)


# COPY A PARTICULAR FOLDER TO THE INSTALL DIRECTORY
# Install config dependencies
install(
  DIRECTORY
    config
  DESTINATION
    share/${PROJECT_NAME}
)


# LAUNCH
# Install launchfile
ament_auto_package(INSTALL_TO_SHARE launch)