# ffmpeg_hardware

`ros2_control`-compatible hardware drivers backed by FFmpeg input/output

## Input Hardware Plugins

Input hardware receives data from external devices and exports it via its own state interfaces.

* **FFmpegInput**: Reads audio/video packets from various devices supported by `libavformat` and `libavdevice` (e.g., V4L2 or network cameras, PulseAudio inputs, local files, screen capture, etc.).
  * **Output states**: `AVPacket`
  * **Parameters**:
    * **url** (string, default:`"/dev/video0"`): URL of the external input device.
    * **input_format** (string, default: `"v4l2"`): Name of the input device format (category) supported by FFmpeg. If left empty, FFmpeg will try to infer it from the `url` value.
    * **media_type** (string, default: `"video"`): Media type of the input stream. Typically `"video"` or `"audio"`.
    * **options** (string, default: `"{timestamps: abs}"`): A YAML string that can be parsed into an `AVDictionary`. It defines device-specific options and will be used to configure the input device.

## Output Hardware Plugins

Output hardware retrieves data from its own command interfaces and sends it to external devices.

* **FFmpegOutput**: Writes audio/video packets and frames to various devices supported by `libavformat` and `libavdevice` (e.g., PulseAudio outputs, local files, etc.).
  * **Input commands**: `AVFrame`, `AVPacket`
  * **Parameters**:
    * **url** (string, default:`"default"`): URL of the external output device.
    * **format** (string, default: `"pulse"`): Name of the output device format (category) supported by FFmpeg. If left empty, FFmpeg will try to infer it from the `url` value.
    * **codec_parameters** (string, default: `"{}"`): A YAML string that can be parsed into an `AVCodecParameters` structure. It defines general codec parameters and will be used to configure the output device.
    * **options** (string, default: `"{}"`): A YAML string that can be parsed into an `AVDictionary`. It defines device-specific options and will be used to configure the output device.
    
* **DumpInfoOutput**: Dumps packet and frame information from controllers for debugging and visualization.
  * **Input commands**: `AVFrame`, `AVPacket`