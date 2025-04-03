cmake_minimum_required(VERSION 3.8)
project(ffmpeg_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(ffmpeg_cpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_library(
  ffmpeg_hardware SHARED
  src/ffmpeg_hardware.cpp)
target_include_directories(
  ffmpeg_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(
  ffmpeg_hardware
  ${YAML_CPP_LIBRARIES})
ament_target_dependencies(
  ffmpeg_hardware
  controller_interface
  ffmpeg_cpp
  hardware_interface
  pluginlib
  rclcpp
  yaml_cpp_vendor)

install(TARGETS ffmpeg_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
pluginlib_export_plugin_description_file(hardware_interface plugins.xml)

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
