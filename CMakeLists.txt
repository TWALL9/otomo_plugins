cmake_minimum_required(VERSION 3.5)
project(otomo_plugins)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(otomo_msgs REQUIRED)
find_package(async_serial REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_library(
  ${PROJECT_NAME}_kiss_serial
  SHARED
  src/kiss_serial/kiss_decode.cpp
  src/kiss_serial/kiss_encode.cpp
)

target_include_directories(
  ${PROJECT_NAME}_kiss_serial
  PRIVATE
  include
)

add_library(${PROJECT_NAME}_diffdrive
  SHARED
  src/otomo_diffdrive/otomo_diffdrive.cpp
)

target_include_directories(
  ${PROJECT_NAME}_diffdrive
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}_diffdrive
  hardware_interface
  controller_manager
  async_serial
  otomo_msgs
  rclcpp
  pluginlib
)

target_link_libraries(${PROJECT_NAME}_diffdrive ${PROJECT_NAME}_kiss_serial)

pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)

install(
  TARGETS ${PROJECT_NAME}_kiss_serial ${PROJECT_NAME}_diffdrive
  DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
