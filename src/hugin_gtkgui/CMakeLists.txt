# Copyright (c) 2023 Sensrad

cmake_minimum_required(VERSION 3.8)
project(hugin_gtkgui)

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

    # gresource will create long strings, this is not a problem.
    -Wno-overlength-strings
  )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(raf_interfaces REQUIRED)

# Find gtkmm via pkgconfig
find_package(PkgConfig)
pkg_search_module(gtkmm REQUIRED IMPORTED_TARGET gtkmm-3.0>=0.29.2)

# Compile glib resources
find_program(GLIB_COMPILE_RESOURCES NAMES glib-compile-resources REQUIRED)

set(GRESOURCE_C window.gresources.c)
set(GRESOURCE_XML resources/window.gresource.xml)

# # When you update the window.glad file you need to regenerate
# # window.gresources.c

# Wether to generate the gresource file or use the checked in version.
set(GENERATE_GRESOURCE_C ON)

if(GENERATE_GRESOURCE_C)
  message(STATUS "Generating gresource file")

  add_custom_command(
    OUTPUT ${GRESOURCE_C}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/resources
    COMMAND ${GLIB_COMPILE_RESOURCES}
    ARGS
    --target=${CMAKE_CURRENT_BINARY_DIR}/${GRESOURCE_C}
    --generate-source
    ${CMAKE_CURRENT_SOURCE_DIR}/${GRESOURCE_XML}
    VERBATIM
    MAIN_DEPENDENCY ${GRESOURCE_XML}
    DEPENDS
    resources/window.glade
  )

  # Dummy target to make sure the gresource is built before the executable
  add_custom_target(
    dummy-resource
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${GRESOURCE_C}
  )

  set_source_files_properties(
    ${CMAKE_CURRENT_BINARY_DIR}/${GRESOURCE_C}
    PROPERTIES GENERATED TRUE
  )

  # Create executable based on generated gresource file.
  add_executable(rafgui src/rafgui.cpp ${CMAKE_CURRENT_BINARY_DIR}/${GRESOURCE_C})
else()
  message(STATUS "Using checked in gresource file")

  # Create executable based on checked in gresource file.
  add_executable(rafgui src/rafgui.cpp src/${GRESOURCE_C})
endif()

# Ament dependencies
ament_target_dependencies(rafgui rclcpp std_msgs sensor_msgs raf_interfaces)

# Link with gktmm
target_link_libraries(rafgui
  PkgConfig::gtkmm
)

# Gtkmm includes
target_include_directories(rafgui PRIVATE
  PkgConfig::gtkmm
)

target_include_directories(rafgui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(rafgui PUBLIC cxx_std_17 c_std_17) # Require C99 and C++17

# Install gui executable
install(TARGETS rafgui
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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
