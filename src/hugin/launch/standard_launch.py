# Copyright (c) Sensrad 2023-2024

"""Standard launch file for hugin stack"""

from enum import IntEnum

from launch_ros.actions import Node
from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
)

from launch.substitutions import LaunchConfiguration


class ActiveSeq(IntEnum):
    """Enum for active sequence"""

    IDLE_SEQ = 0
    COARSE_SHORT_SEQ = 1
    COARSE_MID_SEQ = 2
    COARSE_LONG_SEQ = 3
    FINE_SHORT_SEQ = 4
    FINE_MID_SEQ = 5
    FINE_LONG_SEQ = 6
    CALIBRATION_SEQ = 7
    LAB_SEQ = 8
    DELAY_CALIBRATION_SEQ = 9
    COARSE_ULTRA_LONG_SEQ = 10
    FINE_ULTRA_LONG_SEQ = 11
    USER_CONFIGURE_SEQ1 = 12
    USER_CONFIGURE_SEQ2 = 13
    CALIBRATION_FAST_SEQ = 14


RADAR_ID = 1


def generate_launch_description():
    """Standard launch file for the ROS2 Hugin stack"""

    return LaunchDescription(
        [
            DeclareLaunchArgument("log-level", default_value="info"),
            DeclareLaunchArgument("ip", default_value="10.20.30.40"),
            DeclareLaunchArgument("bind", default_value="0.0.0.0"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    f"radar_{RADAR_ID}",
                ],
            ),
            Node(
                package="hugin",
                namespace=f"hugin_{RADAR_ID}",
                respawn=True,
                respawn_delay=1,
                executable="udp_node_exe",
                name="udp_log_node",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log-level"),
                ],
                remappings=[
                    ("udp_data", "udp_log_data"),  # remap topic
                ],
                parameters=[
                    {
                        "bind": LaunchConfiguration("bind"),
                        "port": 6010,  # log port
                        "log_data": False,
                        "publish_data": False,
                    }
                ],
            ),
            Node(
                package="hugin",
                namespace=f"hugin_{RADAR_ID}",
                respawn=True,
                respawn_delay=1,
                executable="udp_node_exe",
                name="udp_node",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log-level"),
                ],
                parameters=[
                    {
                        "bind": "0.0.0.0",  # the local ip address to listen to
                        "port": 6003,  # pointcloud streaming port
                        "log_data": False,
                        "publish_data": True,
                    }
                ],
            ),
            Node(
                package="hugin",
                namespace=f"hugin_{RADAR_ID}",
                respawn=True,
                respawn_delay=1,
                executable="pointcloud_parser_node_exe",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log-level"),
                ],
                name="pointcloud_parser_node",
                parameters=[
                    {
                        "frame_id": f"radar_{RADAR_ID}",
                        "publish_pointcloud": True,
                        "publish_header": True,
                    }
                ],
            ),
            Node(
                package="hugin",
                namespace=f"hugin_{RADAR_ID}",
                respawn=True,
                respawn_delay=2,
                executable="control_node_exe",
                name="control_node",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log-level"),
                ],
                parameters=[
                    {
                        "fw_version": "1.8.1",  # radar firmware version: {1.7.1, 1.8.1, or 1.8.5}
                        "ip": LaunchConfiguration("ip"),
                        "port": 6001,  # control port
                        "active_seq": ActiveSeq.FINE_MID_SEQ.value,  # Initial active sequence
                        "start_tx": False,  # If true start tx on startup
                    },
                ],
            ),
        ]
    )
