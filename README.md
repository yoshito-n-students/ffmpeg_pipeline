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

## Important Packages

### [ffmpeg_hardware](ffmpeg_hardware)
This package provides hardware-accelerated input/output nodes as `ros2_control` plugins:

#### Input hardwares - read audio/video packets from hardware
* **FFmpegInput**
  * **Description**: Captures audio/video input using FFmpeg with hardware acceleration.
  * **Outputs**: Raw or compressed video/audio streams.
  * **Parameters**: Device type, hardware acceleration options.

#### Output hardwares - write audio/video packets/frames to hardware
* **FFmpegOutput**

* **DumpInfoOutput**

### [ffmpeg_controllers](ffmpeg_controller)
This package implements various audio/video controllers as `ros2_control` plugins:

#### Filter controllers
Filters read objects from state interfaces owned by other controllers and a hardware and export results via self-owned state interface

* **EncoderFilter**
  * **Description**: Encodes raw frames into compressed formats.
  * **Inputs**: `ffmpeg_cpp::Frame`
  * **Outputs**: `ffmpeg_cpp::Packet`, `ffmpeg_cpp::CodecParameters`
  * **Parameters**:
    * **input_name** (`string`, required): name of other controller/hardware that exports input frames
    * **encoder_name** (`string`, default: `""`): name of underlying encoder valid in ffmpeg. If empty, the default encoder for `codec_parameters/codec` will be used.
    * **codec_parameters** (`string`, required): yaml string that can be recognized as `ffmpeg_cpp::CodecParameters`. It will be used to configure the encoder.
    * **encoder_options** (`string`, default: `"{}"`): yaml string that can be recognized as `ffmpeg_cpp::Dictionary`. It will be passed to the encoder as private options.

#### Broadcaster controllers
Broadcasters read objects from state interfaces owned by other controllers and a hardware and publish them as ROS 2 messages

* **PacketBroadcaster**

* **FrameBroadcaster**

* **ImageBroadcaster**

* **CompressedImageBroadcaster**

#### Receiver controllers
Receivers subscribe ROS 2 messages and export them via self-owned state interfaces

* **PacketReceiver**

* **FrameReceiver**

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