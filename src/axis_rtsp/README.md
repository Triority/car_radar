Copyright (c) Sensrad 2023-2024



# axis_rtsp

Package for publishing video frames from an Axis IP camera. Most new axis cameras use the same API.

Axis cameras stream video via TCP or UDP using RTSP/RTP. This node connects to the camera using RTSP and converts the video stream to RGB or JPEG frames which it republishes via ROS2.

The node publishes two topics (camera_1 is an example name space).

For jpeg compressed frames:

* `/camera_1/compressed`
* `/camera_1/camera_info`

For rgb frames:

* `/camera_1/image`
* `/camera_1/camera_info`

The `/image` and `/compressed` topics consists of JPEG encoded frames, whereas the `camera_info` message describes the intrinsic calibration parameters.



## Building

### Install dependencies

This node depends on ROS2, libfmt, and and gstreamer framework. Install the required dependencies by running:

```bash
sudo apt install libfmt-dev gstreamer1.0-alsa gstreamer1.0-plugins-good
gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly libgstreamer1.0-dev
libgstreamer-plugins-base1.0-dev gstreamer1.0-libav
```



## Build

To build with ROS2 colcon:

```bash
colcon build --packages-select axis_rtsp
```



## Possible error

On Ubuntu 22.04 there is a problem with the `gstreamer1.0-vaapi` package (this package is related to hardware accelerated
encoding/decoding). If the node crashes, remove this package:

```bash
sudo apt remove gstreamer1.0-vaapi
```



## Running

The camera IP address and credentials need to be configured in the launch file.

```bash
ros2 launch axis_rtsp standard_launch.py
```
