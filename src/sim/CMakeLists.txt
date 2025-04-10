cmake_minimum_required(VERSION 3.8)
project(sim)

# Find required packages
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-sim8 REQUIRED)

# Install directories
install(
  DIRECTORY
    launch
    models
    worlds
    config
  DESTINATION
    share/${PROJECT_NAME}
)

# Crazy Ball Plugin
add_library(sim-crazy-ball-plugin SHARED
  plugins/crazy_ball_plugin.cpp
)

target_include_directories(sim-crazy-ball-plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(sim-crazy-ball-plugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
)

# Start Signal Plugin
add_library(start-signal-plugin SHARED
  plugins/start_signal_plugin.cpp
)

target_include_directories(start-signal-plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(start-signal-plugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
)

# Install plugins
install(
  TARGETS
    sim-crazy-ball-plugin
    start-signal-plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Python modules if any
# ament_python_install_package(${PROJECT_NAME})

# Add custom target to copy models to Gazebo model path during build
add_custom_target(copy_models ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory $ENV{HOME}/.gz/models/${PROJECT_NAME}
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/models/ackermann_car
    $ENV{HOME}/.gz/models/ackermann_car
)

# Mark cpp executables for installation
# install(TARGETS
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install Python executables
install(
  PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_dependencies(
  rclcpp
  rclpy
  ros_gz_sim
  ros_gz_bridge
  geometry_msgs
  nav_msgs
  tf2_ros
  ros_gz_interfaces
  gz-sim8
  gz-plugin2
)

# Package specific configuration
ament_package()
