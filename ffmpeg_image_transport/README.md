# ffmpeg_image_transport

`image_transport` plugin powered by FFmpeg decoder for compressed images

## `image_transport` Plugins

All parameters are prefixed with `<prefix>` to avoid conflicts with other plugins loaded in the same node. Run `ros2 param list` to find the actual prefix used.

* **ffmpeg_sub**: Decompresses compressed image messages using `libavcodec`.
  * **Subscribed topics**: `~/image/ffmpeg` (`sensor_msgs::msg::CompressedImage`)
  * **Parameters**:
    * **\<prefix>.decoder_name** (string, default: `""`): Name of the underlying decoder supported by FFmpeg. If empty, the default decoder for the value of `CompressedImage::format` will be used.
    * **\<prefix>.hw_type_name** (string, default: `"auto"`): Type of hardware acceleration to use. Can be a specific device name (e.g., `"cuda"`, `"vaapi"`), `"auto"` to automatically select the first available device, or `""` to disable hardware acceleration.

* **ffmpeg_pub**: Compresses image messages using `libavcodec`.
  * **Published topics**: `~/image/ffmpeg` (`sensor_msgs::msg::CompressedImage`)
  * **Parameters**:
    * **\<prefix>.encoder_name** (string, default: `""`): Name of the underlying encoder supported by FFmpeg. If empty, the default encoder for the `codec_parameters/codec_id` will be used.
    * **\<prefix>.codec_parameters** (string, default: `""`): A YAML string that can be parsed into an `AVCodecParameters` structure. It defines general codec parameters and will be used to configure the encoder.
    * **\<prefix>.hw_type_name** (string, default: `"auto"`): Type of hardware acceleration to use. Can be a specific device name (e.g., `"cuda"`, `"vaapi"`), `"auto"` to automatically select the first available device, or `""` to disable hardware acceleration.
    * **\<prefix>.encoder_options** (string, default: `"{}"`): A YAML string that can be parsed into an `AVDictionary`. It defines encoder-specific options and will be used to configure the encoder.