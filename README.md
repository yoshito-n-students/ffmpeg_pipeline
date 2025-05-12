# ffmpeg_pipeline
A flexible pipeline for video/audio acquisition, encoding, decoding, and output using ffmpeg on ROS 2

## Packages Overview
This repository contains the following packages:

* [`ffmpeg_controllers`](ffmpeg_controller): Chainable `ros2_control`-compatible controllers for encoding, decoding, and filtering audio/video streams.
* [`ffmpeg_cpp`](ffmpeg_cpp): C++ wrapper for the FFmpeg library.
* [`ffmpeg_hardware`](ffmpeg_hardware): `ros2_control`-compatible hardware drivers backed by FFmpeg input/output.
* [`ffmpeg_image_transport`](ffmpeg_image_transport): image_transport plugin powered by FFmpeg decoder for compressed images.
* [`ffmpeg_pipeline`](ffmpeg_pipeline): Core package for constructing FFmpeg-based pipelines.
* [`ffmpeg_pipeline_examples`](ffmpeg_pipeline_examples): Provides practical examples of various pipelines.
* [`ffmpeg_pipeline_msgs`](ffmpeg_pipeline_msgs): Message definitions for transferring audio and video data.

## Verified Environments
* ROS 2 Jazzy on Ubuntu 24.04 LTS

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

## Hardware Plugins
See [ffmpeg_hardware](ffmpeg_hardware) for details.

### Input hardwares
* **FFmpegInput**: reads audio/video packets from various devices supported by libavformat and libavdevices

### Output hardwares
* **FFmpegOutput**: writes audio/video packets and frames to various devices supported by libavformat and libavdevices

* **DumpInfoOutput**: dump information of packets and frames from controllers for debug and visualization purposes

## Controller Plugins
See [ffmpeg_controllers](ffmpeg_controller) for details.

### Filter controllers
Filters read objects from other controllers and a hardware, and export processed results via self-owned state interface

* **EncoderFilter**: compresses frames to various formats based on libavcodec

* **DecoderFilter**: decompress packets in various formats based on libavcodec

* **VideoConterterFilter**: changes resolutions and pixel formats based on libswscale

* **AudioConverterFilter**: changes sample rates, channel layouts and sample formats based on libswresample

* **AudioFifoFilter**: changes number of samples in single audio frame based on libavutil

### Broadcaster controllers
Broadcasters read objects from other controllers and a hardware, and publish them as ROS 2 messages

* **PacketBroadcaster**: publishes compressed audio/video packets

* **FrameBroadcaster**: publishes uncompressed audio/video frames

* **ImageBroadcaster**: publishes video frames as sensor_msgs::msg::Image

* **CompressedImageBroadcaster**: publishes video packets as sensor_msgs::msg::CompressedImage

### Receiver controllers
Receivers subscribe ROS 2 messages and export them via self-owned state interfaces

* **PacketReceiver**: subscribes compressed packets and imports them into the pipeline

* **FrameReceiver**: subscribes uncompressed frames and imports them into the pipeline

## image_transport Plugins
See [ffmpeg_image_transport](ffmpeg_image_transport) for details.

* **ffmpeg_sub**: subscribes sensor_msgs::msg::CompressedImage on image/ffmpeg topic and decompresses messages using libavcodec

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