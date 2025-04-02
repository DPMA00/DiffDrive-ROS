cmake_minimum_required(VERSION 3.5)
project(my_cpp_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/../serial/include
)

# ardino pid_diffdrive_control node
add_executable(pid_diffdrive_control src/pid_diffdrive_control.cpp)
ament_target_dependencies(pid_diffdrive_control
                          rclcpp
                          nav_msgs
                          my_robot_interfaces)

# arduino_motor_control node
add_executable(arduino_bridge_node src/arduino_bridge_node.cpp)
target_link_libraries(arduino_bridge_node serial)

ament_target_dependencies(arduino_bridge_node
  rclcpp
  my_robot_interfaces
)

# Installation
install(TARGETS 
  arduino_bridge_node
  pid_diffdrive_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
