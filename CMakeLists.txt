cmake_minimum_required(VERSION 3.5)
project(otomo_plugins)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

set(CMAKE_VERBOSE_MAKEFILE ON)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PLUGINS_DEPENDS
  hardware_interface
  controller_manager
  async_serial
  otomo_msgs
  rclcpp
  pluginlib
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
foreach(DEP IN ITEMS ${PLUGINS_DEPENDS})
  find_package(${DEP} REQUIRED)
endforeach()

add_library(${PROJECT_NAME}
  SHARED
  src/otomo_diffdrive/otomo_diffdrive.cpp
  src/otomo_controller/otomo_controller.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  ${PROJECT_NAME} PUBLIC
  ${PLUGINS_DEPENDS}
)

add_executable(diffdrive_test_node src/diffdrive_test/diffdrive_test_node.cpp)
ament_target_dependencies(diffdrive_test_node rclcpp otomo_msgs async_serial)

pluginlib_export_plugin_description_file(hardware_interface interface_plugins.xml)
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(
  TARGETS diffdrive_test_node
  DESTINATION lib/${PROJECT_NAME}
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

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
)

ament_package()
