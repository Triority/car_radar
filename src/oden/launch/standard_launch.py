# Copyright (c) Sensrad 2023
#
"""Standard launch file Sensrad radar perception."""

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

RADAR_ID = 1


def generate_launch_description():
    """Standard launch file for Hugin preprocessing of point clouds"""

    return LaunchDescription(
        [
            DeclareLaunchArgument("log-level", default_value="info"),
            Node(
                package="oden",
                executable="oden",
                namespace=f"oden_{RADAR_ID}",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log-level"),
                ],
                parameters=[
                    {"tracker_topic": "object_list"},
                    {"radar_topic": "hugin_raf_1/radar_data"},
                    {"frame_id": "radar_1"},
                    {"dynamic": True},  # Set to true if radar is in motion
                    {"local_max": True},  # Set to true if local_max algo is used
                    {
                        "suppress_disturbances": True
                    },  # Set to true if vertical arches filter is used
                    {"vertical_height": 1.0},  # Set to radar vertical height in meters
                    {
                        "radar_elevation_angle": 0.0
                    },  # Set to radar elevation angle in degrees
                ],
            ),
        ]
    )
