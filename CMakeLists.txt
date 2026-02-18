cmake_minimum_required(VERSION 3.8)
project(avdr_gz_worlds)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS loader register)

# Elevator controller plugin (C++)
add_library(elevator_controller SHARED
  plugins/elevator_controller.cpp
)

ament_target_dependencies(elevator_controller
  rclcpp
  std_msgs
  geometry_msgs
  example_interfaces
)

target_include_directories(elevator_controller PRIVATE
  /usr/include/ignition/gazebo6
)

target_link_libraries(elevator_controller
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::ignition-plugin1
)

# Install C++ plugin
install(TARGETS elevator_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts (when they exist)
install(PROGRAMS
  scripts/elevator_scheduler.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()