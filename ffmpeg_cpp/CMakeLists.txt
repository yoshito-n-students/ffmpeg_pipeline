cmake_minimum_required(VERSION 3.8)
project(ffmpeg_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ffmpeg_pipeline_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# ffmpeg_cpp as a shared library
add_library(
  ffmpeg_cpp SHARED
  src/ffmpeg_cpp/audio_converter.cpp
  src/ffmpeg_cpp/codec_parameters.cpp
  src/ffmpeg_cpp/datatypes.cpp
  src/ffmpeg_cpp/decoder.cpp
  src/ffmpeg_cpp/encoder.cpp
  src/ffmpeg_cpp/input.cpp
  src/ffmpeg_cpp/output.cpp
  src/ffmpeg_cpp/parser.cpp
  # src/ffmpeg_cpp/resampler.cpp
  src/ffmpeg_cpp/utility.cpp
  src/ffmpeg_cpp/video_converter.cpp)
target_include_directories(
  ffmpeg_cpp PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(ffmpeg_cpp avcodec avdevice avformat avutil swscale ${YAML_CPP_LIBRARIES})
ament_target_dependencies(ffmpeg_cpp ffmpeg_pipeline_msgs rclcpp sensor_msgs yaml_cpp_vendor)

#
add_executable(
  decode_bag
  src/decode_bag.cpp)
target_include_directories(
  decode_bag PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(decode_bag ffmpeg_cpp)
ament_target_dependencies(decode_bag rclcpp rosbag2_storage rosbag2_transport sensor_msgs)

add_executable(
  decode_topic
  src/decode_topic.cpp)
target_include_directories(
  decode_topic PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(decode_topic ffmpeg_cpp)
ament_target_dependencies(decode_topic rclcpp sensor_msgs)

add_executable(
  play_wav
  src/play_wav.cpp)
target_include_directories(
  play_wav PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(play_wav ffmpeg_cpp)
# ament_target_dependencies(stream_camera rclcpp sensor_msgs)

add_executable(
  stream_camera
  src/stream_camera.cpp)
target_include_directories(
  stream_camera PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(stream_camera ffmpeg_cpp)
ament_target_dependencies(stream_camera rclcpp sensor_msgs)

# install the library
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS ffmpeg_cpp
  EXPORT export_ffmpeg_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(
  TARGETS decode_bag decode_topic play_wav stream_camera
  DESTINATION lib/${PROJECT_NAME})

# export the library for downstream projects
ament_export_include_directories(include)
ament_export_targets(export_ffmpeg_cpp HAS_LIBRARY_TARGET)
ament_export_dependencies(ffmpeg_pipeline_msgs rclcpp yaml_cpp_vendor)

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
