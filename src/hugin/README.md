Copyright (c) Sensrad 2023-2024


# hugin

This is a collection of ROS2 nodes for controlling and receiving data from the Hugin radar. It publishes pointcloud2 data.

The `hugin` package implements three nodes:

* UDP node for receiving point cloud packets and logging packets.

* Control node for controlling radar via TCP.

* Point cloud parser node for parsing point clouds and publishing pointcloud2 messages.

Package `raf_interfaces` implements the service protocol (it is ROS2 best practice to implement this in a separate package).


## Requirements

Built and tested with ROS2 foxy on Ubuntu 20.04, and ROS2 humble on Ubuntu 22.04. You need `libboost-dev`, any recent version will work. Install it with:
```bash
sudo apt install libboost1.71-dev
```

Make sure you build and source the `raf_interfaces` and `arbe_raf` packages before you build the `hugin` package.


## Build Instructions

* Add folders `hugin/`, `raf_interfaces/`, and `arbe_raf/` to `src/` folder in ROS2 workspace.

* Change to ROS2 workspace.

* Build `raf_interfaces` and `arbe_raf` packages with colcon:
```bash
colcon build --packages-select raf_interfaces arbe_raf
```
Note that there might occur some build warnings which may be ignored.

* Source workspace:
```bash
source install/setup.bash
```

* Build `hugin` package:
```bash
colcon build --packages-select hugin
```

* Source again:
```bash
source install/setup.bash
```

Note that the ROS2 workspace shall be sourced in every active terminal
window which interfaces with the ROS2 system (e.g., `ros2 bag record`).

Further note that settings such as the radar IP address can be modified in the file `/config/default.yaml`.


## Usage Instructions

### Start `hugin` Package:

* Change to ROS2 workspace.

* Source ROS2 workspace:
```bash
source install/setup.bash
```

* Launch via launch-file:
```bash
ros2 launch hugin standard_launch.py
```

This will launch under the name space `hugin_1` and publish point clouds under `radar_data`, i.e., this package will publish a ROS2 topic named `/hugin_1/radar_data`.


### Control Hugin Radar:

* Ensure the `hugin` package is started.

* In a separate terminal, change to ROS2 workspace and source it.

* See below for how to start and stop the radar, set sequence type, and write time to the radar.

These are the valid integers to use as sequence type:
```
IdleSeq            = 0
CoarseShortSeq     = 1
CoarseMidSeq       = 2
CoarseLongSeq      = 3
FineShortSeq       = 4
FineMidSeq         = 5
FineLongSeq        = 6
CoarseUltraLongSeq = 10
FineUltraLongSeq   = 11
```

Change sequence type (mode) of the radar with:
```bash
ros2 service call /hugin_1/set_active_seq raf_interfaces/srv/RdrCtrlSetActiveSeq "{sequence_type: 4}"
```


Start radar transmission:
```bash
ros2 service call /hugin_1/start raf_interfaces/srv/RdrCtrlStartTx
```

Stop radar transmission:
```bash
ros2 service call /hugin_1/stop raf_interfaces/srv/RdrCtrlStopTx
```

Sync radar to system (ROS2) time:
```bash
ros2 service call /hugin_1/set_time raf_interfaces/srv/SysCfgSetTime "{'use_ros2_time': true}"
```

Set thresholds (example values):
```bash
ros2 service call /hugin_1/set_thresholds raf_interfaces/srv/RdrCtrlSetThresholds "{static_threshold: 5.0, dynamic_azimuth: 20.0, dynamic_elevation: 5.0}"
```
