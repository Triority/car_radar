Copyright (c) Sensrad 2023-2024

## hugin_gtkgui

This node (hugin_gtkgui) provides a GUI application for controlling the Hugin radar. The hugin_gtkgui application is written in C++17 using gtkmm (gtk-3).

The hugin_gtkgui application supports basic operations such as:
start/stop Tx, make a time synchronization between ROS2 and radar time, select mode, and controlling some radar parameters.

## Dependencies

Install the gtkmm libraries from
```bash
sudo apt install libgtkmm-3.0-doc libgstreamermm-1.0-doc devhelp
```

## Build

Then build the node:
```bash
colcon build --packages-select hugin_gtkgui
```

Source the package:
```bash
source ~/ros2_ws/install/setup.bash
```

The hugin_gtkgui node is (standard usage) launched by
```bash
ros2 launch hugin_gtkgui gui_launch.py
```
