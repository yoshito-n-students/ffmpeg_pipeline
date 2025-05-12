# ffmpeg_pipeline
A flexible pipeline for video/audio acquisition, encoding, decoding, and output using ffmpeg on ROS 2

## Packages Overview
This repository contains the following packages:

* [`ffmpeg_controllers`](ffmpeg_controller): Controllers for encoding, decoding, and filtering audio/video streams.
* [`ffmpeg_cpp`](ffmpeg_cpp): C++ wrapper for the FFmpeg library.
* [`ffmpeg_hardware`](ffmpeg_hardware): FFmpeg input/output nodes utilizing hardware acceleration.
* [`ffmpeg_image_transport`](ffmpeg_image_transport): ROS2 image transport plugin for compressed images.
* [`ffmpeg_pipeline`](ffmpeg_pipeline): Core package for constructing FFmpeg-based pipelines.
* [`ffmpeg_pipeline_examples`](ffmpeg_pipeline_examples): Provides practical examples of various pipelines.
* [`ffmpeg_pipeline_msgs`](ffmpeg_pipeline_msgs): ROS2 message definitions for transferring audio and video data.

## Main Features
* Real-time audio and video processing.
* Flexible pipeline construction.
* Hardware acceleration support.
* ROS2-native messaging for data communication.
* Utilizes the `ros2_control` framework for controller management.

## Verified Environments
* ROS2 Jazzy on Ubuntu 24.04

## Installation
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <this repository URL>
rosdep install --from-paths . --ignore-src --rosdistro jazzy -y
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Important Packages

### [ffmpeg_controllers](ffmpeg_controller)
This package implements various audio/video controllers as `ros2_control` plugins:

* **EncoderFilter**
  * **Description**: Encodes raw frames into compressed formats.
  * **Inputs**: Raw video/audio frames.
  * **Outputs**: Compressed packets.
  * **Parameters**: Codec type, bitrate, resolution, frame rate.

### [ffmpeg_hardware](ffmpeg_hardware)
This package provides hardware-accelerated input/output nodes as `ros2_control` plugins:

* **FFmpegInput**
  * **Description**: Captures audio/video input using FFmpeg with hardware acceleration.
  * **Outputs**: Raw or compressed video/audio streams.
  * **Parameters**: Device type, hardware acceleration options.

## Running Examples
Below are examples of launching sample pipelines.

### Streaming H.264 Camera Video
```bash
ros2 launch ffmpeg_pipeline_examples h264_camera_pipeline.launch.py
```

### Playing a WAV File
```bash
ros2 launch ffmpeg_pipeline_examples wav_file_pipeline.launch.py
```

### Audio Capture with PulseAudio
```bash
ros2 launch ffmpeg_pipeline_examples pulse_audio_capture_pipeline.launch.py
```