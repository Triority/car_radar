# Copyright (c) 2023 Sensrad

cmake_minimum_required(VERSION 3.8)
project(runes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(oden_interfaces REQUIRED)
find_package(message_filters REQUIRED)

find_package(Eigen3 3.3 REQUIRED)
find_package(liboden REQUIRED)

# Define the executable
add_executable(perception_visualizer src/main.cpp src/PerceptionVisualizer.cpp src/PolygonGenerator.cpp)

target_include_directories(perception_visualizer PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(perception_visualizer Eigen3::Eigen)

# Add common dependencies
ament_target_dependencies(perception_visualizer liboden rclcpp std_msgs shape_msgs geometry_msgs visualization_msgs oden_interfaces message_filters Eigen3)

# Install targets
install(TARGETS perception_visualizer
  DESTINATION lib/${PROJECT_NAME})

ament_package()
