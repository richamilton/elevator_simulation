cmake_minimum_required(VERSION 3.8)
project(elevator_simulation)

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
find_package(gazebo_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo REQUIRED)
find_package(example_interfaces REQUIRED)

# Include directories
include_directories(${GAZEBO_INCLUDE_DIRS})

# Elevator controller plugin (C++)
add_library(elevator_controller SHARED
  plugins/elevator_controller.cpp
)

ament_target_dependencies(elevator_controller
  rclcpp
  std_msgs
  geometry_msgs
  gazebo_ros
  example_interfaces
)

target_link_libraries(elevator_controller ${GAZEBO_LIBRARIES})

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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()