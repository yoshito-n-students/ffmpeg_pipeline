# ffmpeg_image_transport

`image_transport` plugin powered by FFmpeg decoder for compressed images

## `image_transport` Plugins

* **ffmpeg_sub**: Decompresses compressed image messages using `libavcodec`.
  * **Subscribed topics**: `~/image/ffmpeg` (`sensor_msgs::msg::CompressedImage`)
  * **Parameters**:
    * **decoder_name** (string, default: `""`): Name of the underlying decoder supported by FFmpeg. If empty, the default decoder for the value of `CompressedImage::format` will be used.
    * **hw_type_name** (string, default: `"auto"`): Type of hardware acceleration to use. Can be a specific device name (e.g., `"cuda"`, `"vaapi"`), `"auto"` to automatically select the first available device, or `""` to disable hardware acceleration.