Copyright (c) Sensrad 2023-2024


# runes

This is a ROS2 package for receiving perception data from `oden` and publishing them in a standardized format which can be visualized in e.g. rviz2. 


## Requirements

Built and tested with ROS2 humble on Ubuntu 22.04.

Make sure you build and source the `raf_interfaces` and `arbe_raf` packages before you build the `runes` package.


## Build Instructions

* Add folders `runes/`, `raf_interfaces/`, and `arbe_raf/` to `src/` folder in ROS2 workspace.
* Change to ROS2 workspace.
* Build `hugin` package:
```bash
colcon build --packages-select runes
```

* Source ROS2 workspace:
```bash
source install/setup.bash
```

Note that the ROS2 workspace shall be sourced in every active terminal window which interfaces with the ROS2 system (e.g., `ros2 bag record`).


## Usage Instructions

### Start `perception_visualizer` node:

* Change to ROS2 workspace.

* Source ROS2 workspace:
```bash
source install/setup.bash
```

* Launch via launch-file:
```bash
ros2 run runes perception_visualizer
```

This will start the `perception_visualizer` node manually, however, this node will also be brought up when launching the standard file via:
```bash
ros2 launch launch/standard.py
```

This node receives `oden_1/ground_plane_data` ,`oden_1/free_space_data` and `oden_1/object_tracks` messages, and publishes `oden_1/ground_plane_polygon`, `oden_1/free_space_polygon` and `/oden_1/aabb_object_tracks`
