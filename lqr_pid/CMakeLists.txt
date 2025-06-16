cmake_minimum_required(VERSION 3.8)
project(lqr_pid)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
foreach(PACKAGE
  ament_cmake
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  ackermann_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  eigen3_cmake_module
  Eigen3
  tf2_ros
  tf2)

  find_package(${PACKAGE} REQUIRED)

endforeach()

# Include Cpp "include" directory
include_directories(include)
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

# Create Cpp executable
add_executable(lqr_pid src/lqr_pid_node.cpp)

ament_target_dependencies(lqr_pid
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs visualization_msgs tf2_ros tf2 eigen3_cmake_module Eigen3
)

add_executable(waypoint_visualizer src/waypoint_visualizer.cpp)

ament_target_dependencies(waypoint_visualizer
  rclcpp geometry_msgs std_msgs visualization_msgs eigen3_cmake_module Eigen3
)

# Install Cpp executables
install(TARGETS
  lqr_pid
  waypoint_visualizer
  DESTINATION lib/${PROJECT_NAME})

# Install launch files and config
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

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

ament_package()
