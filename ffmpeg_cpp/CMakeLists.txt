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
  src/audio_converter.cpp
  src/audio_fifo.cpp
  src/codec_parameters.cpp
  src/decoder.cpp
  src/dictionary.cpp
  src/encoder.cpp
  src/frame.cpp
  src/input.cpp
  src/output.cpp
  src/packet.cpp
  src/parser.cpp
  src/utility.cpp
  src/video_converter.cpp)
target_include_directories(
  ffmpeg_cpp PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(ffmpeg_cpp avcodec avdevice avformat avutil swscale ${YAML_CPP_LIBRARIES})
ament_target_dependencies(ffmpeg_cpp ffmpeg_pipeline_msgs rclcpp sensor_msgs yaml_cpp_vendor)

#
add_executable(
  decode_bag
  src/demos/decode_bag.cpp)
target_include_directories(
  decode_bag PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(decode_bag ffmpeg_cpp)
ament_target_dependencies(decode_bag rclcpp rosbag2_storage rosbag2_transport sensor_msgs)

add_executable(
  play_wav
  src/demos/play_wav.cpp)
target_include_directories(
  play_wav PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(play_wav ffmpeg_cpp)
# ament_target_dependencies(stream_camera rclcpp sensor_msgs)

add_executable(
  stream_camera
  src/demos/stream_camera.cpp)
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
  TARGETS decode_bag play_wav stream_camera
  DESTINATION lib/${PROJECT_NAME})

# export the library for downstream projects
ament_export_include_directories(include)
ament_export_targets(export_ffmpeg_cpp HAS_LIBRARY_TARGET)
ament_export_dependencies(ffmpeg_pipeline_msgs rclcpp yaml_cpp_vendor)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # the following line specifies the clang-format file to use in clang_format test
  set(ament_cmake_clang_format_CONFIG_FILE ../.clang-format)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_dictionary test/test_dictionary.cpp)
  if(TARGET test_dictionary)
    target_link_libraries(test_dictionary ffmpeg_cpp ${YAML_CPP_LIBRARIES})
    ament_target_dependencies(test_dictionary yaml_cpp_vendor)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      # The tests perform intentional self-moves to prove wrapper robustness, so suppress the
      # corresponding warning just for this target.
      target_compile_options(test_dictionary PRIVATE -Wno-self-move)
    endif()
  endif()

  ament_add_gtest(test_codec_parameters test/test_codec_parameters.cpp)
  if(TARGET test_codec_parameters)
    target_link_libraries(test_codec_parameters ffmpeg_cpp)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      # These tests also rely on intentional self-move assignments; silence the warning locally.
      target_compile_options(test_codec_parameters PRIVATE -Wno-self-move)
    endif()
  endif()

  ament_add_gtest(test_frame test/test_frame.cpp)
  if(TARGET test_frame)
    target_link_libraries(test_frame ffmpeg_cpp)
    target_include_directories(test_frame PRIVATE src)
    ament_target_dependencies(test_frame ffmpeg_pipeline_msgs rclcpp sensor_msgs)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      target_compile_options(test_frame PRIVATE -Wno-self-move)
    endif()
  endif()

  ament_add_gtest(test_packet test/test_packet.cpp)
  if(TARGET test_packet)
    target_link_libraries(test_packet ffmpeg_cpp)
    target_include_directories(test_packet PRIVATE src)
    ament_target_dependencies(test_packet ffmpeg_pipeline_msgs rclcpp sensor_msgs)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      target_compile_options(test_packet PRIVATE -Wno-self-move)
    endif()
  endif()

  ament_add_gtest(test_utility test/test_utility.cpp)
  if(TARGET test_utility)
    target_link_libraries(test_utility ffmpeg_cpp)
    ament_target_dependencies(test_utility sensor_msgs)
  endif()

  ament_add_gtest(test_input test/test_input.cpp)
  if(TARGET test_input)
    target_link_libraries(test_input ffmpeg_cpp)
    target_include_directories(test_input PRIVATE src)
    ament_target_dependencies(test_input rclcpp)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      target_compile_options(test_input PRIVATE -Wno-self-move)
    endif()
  endif()

  ament_add_gtest(test_output test/test_output.cpp)
  if(TARGET test_output)
    target_link_libraries(test_output ffmpeg_cpp)
    target_include_directories(test_output PRIVATE src)
  endif()
endif()

ament_package()
