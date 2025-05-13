# ffmpeg_controllers

Chainable `ros2_control`-compatible controllers for encoding, decoding, and filtering audio/video streams

## Filter Plugins

Filters retrieve data from state interfaces owned by hardware or other controllers and export processed results via their own state interfaces.

* **EncoderFilter**: Compresses frames into various formats using `libavcodec` (e.g., H.264, MJPEG, Opus, and over 100 other codecs).
  * **Input states**: `AVFrame`
  * **Output states**: `AVPacket`, `AVCodecParameters`
  * **Parameters**:
    * **input_name** (string, required): Name of the hardware or controller that owns the input states.
    * **encoder_name** (string, default: `""`): Name of the underlying encoder supported by FFmpeg. If empty, the default encoder for the `codec_parameters/codec` will be used.
    * **codec_parameters** (string, required): A YAML string that can be parsed into an `AVCodecParameters` structure. It defines general codec parameters and will be used to configure the encoder.
    * **encoder_options** (string, default: `"{}"`): A YAML string that can be parsed into an `AVDictionary`. It defines encoder-specific options and will be used to configure the encoder.

* **ParserFilter**: Parses packets using `libavcodec` to extract codec parameters for downstream controllers.
  * **Input states**: `AVPacket`
  * **Output states**: `AVPacket`, `AVCodecParameters`
  * **Parameters**:
    * **input_name** (string, required): Name of the hardware or controller that owns the input states.
    * **codec_name** (string, required): Name of the codec supported by FFmpeg and used to configure the underlying parser.

* **DecoderFilter**: Decompresses packets in various formats using `libavcodec`.
  * **Input states**: `AVPacket`, `AVCodecParameters`
  * **Output states**: `AVFrame`
  * **Parameters**:
    * **input_name** (string, required): Name of the hardware or controller that owns the input states.
    * **decoder_name** (string, default: `""`): Name of the underlying decoder supported by FFmpeg. If empty, the default decoder for the value of `AVCodecParameters::codec_id` will be used.
    * **request_format_name** (string, default: `""`): Name of the preferred output format of `AVFrame`. If left empty or unsupported by the decoder, the decoder's default format will be used.
    * **decoder_options** (string, default: `"{}"`): A YAML string that can be parsed into an `AVDictionary`. It defines decoder-specific options and will be used to configure the decoder.

* **VideoConverterFilter**: Converts resolutions and pixel formats using `libswscale`.
  * **Input states**: `AVFrame`
  * **Output states**: `AVFrame`
  * **Parameters**:
    * **input_name** (string, required): Name of the hardware or controller that owns the input states.
    * **dst_width** (int, default: `0`): Desired width of the output `AVFrame`. If `0` or less, the converter will keep the input width.
    * **dst_height** (int, default: `0`): Desired height of the output `AVFrame`. If `0` or less, the converter will keep the input height.
    * **dst_format** (string, default: `""`): Desired pixel format of output `AVFrame`. If empty, the converter will keep the input format.


* **AudioConverterFilter**: Converts sample rates, channel layouts, and formats using `libswresample`.
  * **Input states**: `AVFrame`
  * **Output states**: `AVFrame`
  * **Parameters**:
    * **input_name** (string, required): Name of the hardware or controller that owns the input states.
    * **out_channel_layout** (string, default: `""`): Desired channel layout of the output `AVFrame` (e.g., `mono`, `stereo`, etc.). This will be parsed by `av_channel_layout_from_string()`. If left empty, the converter will preserve the input channel layout.
    * **out_format** (string, default: `""`): Desired sample format of the output `AVFrame`. If left empty, the converter will preserve the input format.
    * **out_sample_rate** (int, default: `0`): Desired sample rate of the output `AVFrame`. If `0` or less, the converter will preserve the input sample rate.

* **AudioFifoFilter**: Adjusts the number of samples per audio frame using `libavutil`.
  * **Input states**: `AVFrame`
  * **Output states**: `AVFrame`
  * **Parameters**:
    * **input_name** (string, required): Name of the hardware or controller that owns the input states.
    * **channel_layout** (string, default: `""`): Channel layout for the underlying sample FIFO and the output `AVFrame` (e.g., `mono`, `stereo`, etc.). This will be parsed by `av_channel_layout_from_string()`. If left empty, the input channel layout will be used.
    * **nb_samples** (int, required): Desired number of samples in the output `AVFrame`. A larger number than that of the input is recommended; otherwise, some samples may be missing in the output `AVFrame`.

## Broadcaster Plugins

Broadcasters retrieve data from hardware or other controllers and publish it as ROS 2 messages.

* **PacketBroadcaster**: Publishes compressed audio/video packets.

* **FrameBroadcaster**: Publishes uncompressed audio/video frames.

* **ImageBroadcaster**: Publishes video frames as `sensor_msgs::msg::Image`.

* **CompressedImageBroadcaster**: Publishes video packets as `sensor_msgs::msg::CompressedImage`.

## Receiver Plugins

Receivers subscribe to ROS 2 messages and export them via their own state interfaces.

* **PacketReceiver**: Subscribes to compressed packets and injects them into the pipeline.

* **FrameReceiver**: Subscribes to uncompressed frames and injects them into the pipeline.

## Writer Plugins

Writers copy data from a state interface owned by other controller to a command interface owned by a hardware.

* **PacketWriter**: Copies packets from the other controller to the hardware.

* **FrameWriter**: Copies frames from the other controller to the hardware.