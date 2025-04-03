cmake_minimum_required(VERSION 3.8)
project(actions_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)


add_executable(target_handling_actionserver src/target_handling_actionserver.cpp)
ament_target_dependencies(target_handling_actionserver
                          rclcpp
                          rclcpp_action
                          my_robot_interfaces
                          nav_msgs
                          geometry_msgs)


install(TARGETS
      target_handling_actionserver
        DESTINATION lib/${PROJECT_NAME}/
        )

ament_package()
