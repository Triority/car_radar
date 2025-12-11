# Copyright (c) Sensrad 2023
#!/usr/bin/env bash
set -eu
if [ $# -ne 1 ]; then
        echo "illegal number of parameters"
fi
ros2 bag record /hugin_raf_1/radar_data /hugin_raf_1/radar_header /hugin_raf_1/control_state /hugin_cam_1/compressed /hugin_cam_1/camera_info /tf_static /imu /gnss/nav_sat_fix /rosout -s mcap -o $1