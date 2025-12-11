# Copyright (c) Sensrad 2023
#
"""Standard launch file for hugin_raf stack"""

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch GUI with in namespace with parameters"""

    return LaunchDescription(
        [
            DeclareLaunchArgument("log-level", default_value="info"),
            DeclareLaunchArgument("namespace", default_value="hugin_1"),
            Node(
                package="hugin_gtkgui",
                namespace=LaunchConfiguration("namespace"),
                executable="rafgui",
                name="rafgui",
                arguments=[
                    "--gtk-debug=misc",
                ],
                parameters=[
                    {
                        # emtpy
                    }
                ],
            ),
        ]
    )
