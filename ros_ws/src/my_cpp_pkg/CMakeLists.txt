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
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

include_directories(
  ${PROJECT_SOURCE_DIR}/../serial/include
  ${mpc_solver_INCLUDE_DIRS}
)



add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../serial ${CMAKE_CURRENT_BINARY_DIR}/serial_build)

add_executable(pid_diffdrive_control src/pid_diffdrive_control.cpp)
ament_target_dependencies(pid_diffdrive_control
                          rclcpp
                          nav_msgs
                          geometry_msgs
                          my_robot_interfaces)



add_executable(mcu_bridge_node src/mcu_bridge_node.cpp)
target_link_libraries(mcu_bridge_node serial)
ament_target_dependencies(mcu_bridge_node
  rclcpp
  my_robot_interfaces
  example_interfaces
  sensor_msgs
)


add_executable(odometry_node src/odometry_node.cpp)
ament_target_dependencies(odometry_node
                          rclcpp
                          nav_msgs
                          geometry_msgs
                          tf2
                          tf2_ros
                          my_robot_interfaces
                          )

install(TARGETS 
  mcu_bridge_node
  pid_diffdrive_control
  odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
