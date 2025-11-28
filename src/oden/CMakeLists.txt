# Copyright (c) 2023 Sensrad

cmake_minimum_required(VERSION 3.8)
project(oden)

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
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(Eigen3 3.3 REQUIRED)
find_package(oden_interfaces REQUIRED)

# Try to find liboden as a package, not required
find_package(liboden QUIET)

# Define the executable
add_executable(oden src/main.cpp src/RadarPerception.cpp)

# Add common dependencies
ament_target_dependencies(oden
  rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs Eigen3 oden_interfaces)

# If liboden is found.
if(liboden_FOUND)
  message(STATUS "Using liboden package: ${liboden_INCLUDE_DIRS}")

  # Add liboden as a dependency
  ament_target_dependencies(oden liboden)
else()
  message(STATUS "liboden package not found, using pkg-config")

  # Find liboden via pkgconfig
  find_package(PkgConfig)
  pkg_search_module(liboden REQUIRED IMPORTED_TARGET liboden)

  target_link_libraries(oden
    PkgConfig::liboden)

  target_include_directories(oden PRIVATE
    PkgConfig::liboden
  )
endif()

target_link_libraries(oden Eigen3::Eigen)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Install targets
install(TARGETS oden
  DESTINATION lib/${PROJECT_NAME})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
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
