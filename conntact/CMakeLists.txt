cmake_minimum_required(VERSION 3.5)
project(conntact)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

install(
  DIRECTORY config resource src/conntact
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules
ament_python_install_package(src/${PROJECT_NAME})

# Install Python executables
#install(
#    PROGRAMS
#        test.py
#        nodes/ros2_spiral_search_node
#       nodes/corner_search_node
#        nodes/plotting_node
#    DESTINATION lib/${PROJECT_NAME}/
#)

ament_package()
