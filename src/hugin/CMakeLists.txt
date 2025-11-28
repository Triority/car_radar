cmake_minimum_required(VERSION 3.5)
project(hugin)

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -pedantic
    -Wshadow
    -Wextra
    -Wpointer-arith
    -Wcast-qual
    -Wcast-align
    -Wconversion
    -Wstrict-aliasing=2
  )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(arbe_raf REQUIRED)
find_package(raf_interfaces REQUIRED)
find_package(Boost 1.71 REQUIRED)

# UDP node
add_executable(udp_node_exe
  src/udp_node_exe.cpp
  src/hugin_string_helpers.cpp
  src/hugin_udp_receiver_node.hpp
)
ament_target_dependencies(udp_node_exe rclcpp std_msgs raf_interfaces Boost)

# TCP Control node
add_executable(control_node_exe
  src/control_node_exe.cpp
  src/hugin_raf_control_node.hpp
  src/hugin_string_helpers.cpp
)
ament_target_dependencies(control_node_exe rclcpp std_msgs std_srvs arbe_raf raf_interfaces Boost)

# Pointcloud parser
add_executable(pointcloud_parser_node_exe
  src/pointcloud_parser_node_exe.cpp
  src/hugin_raf_pointcloud_parser_node.hpp
  src/hugin_string_helpers.cpp
)
ament_target_dependencies(pointcloud_parser_node_exe rclcpp std_msgs sensor_msgs arbe_raf raf_interfaces)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Install executables
install(TARGETS
  udp_node_exe
  control_node_exe
  pointcloud_parser_node_exe
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  include_directories(test)

  find_package(ament_cmake_gtest REQUIRED)

  # Add tests
  ament_add_gtest(string_helpers_gtest test/string_helpers.cpp src/hugin_string_helpers.cpp)
  target_include_directories(string_helpers_gtest PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/src)

  # Install tests
  install(TARGETS
    string_helpers_gtest
    DESTINATION test/${PROJECT_NAME})
endif()

ament_package()
