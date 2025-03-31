cmake_minimum_required(VERSION 3.5)
project(my_cpp_pkg)

# Required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(my_robot_interfaces REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/../serial/include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# arduino_bridge node
#add_executable(arduino_bridge src/arduino_bridge.cpp)
#ament_target_dependencies(arduino_bridge
#  rclcpp
#  std_msgs
#)



# arduino_motor_control node
add_executable(arduino_motor_control src/arduino_motor_control.cpp)
target_link_libraries(arduino_motor_control serial)

ament_target_dependencies(arduino_motor_control
  rclcpp
  my_robot_interfaces
)

# Installation
install(TARGETS 
  #arduino_bridge
  arduino_motor_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
