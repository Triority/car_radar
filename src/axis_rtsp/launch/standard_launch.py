# Copyright (c) Sensrad 2023

"""Basic support for Axis cameras/F44"""

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# camera intrinsic (example values)
F_X = 1116.1158699160048
C_X = 629.9130761235966
F_Y = 1120.3128233251753
C_Y = 322.47296777743145

# plumb_bob distortion parameters (example values)
D_1 = -0.3790859731972463
D_2 = -0.01834286498891667
D_3 = -0.00027601872356912367
D_4 = -0.0012907466416806797
D_5 = 0.8043773350759326


def generate_launch_description():
    """Example launch file"""

    return LaunchDescription(
        [
            DeclareLaunchArgument("log-level", default_value="info"),
            # This is a default IP for Axis cameras.
            DeclareLaunchArgument("ip", default_value="192.168.0.90"),
            DeclareLaunchArgument("user", default_value="sensrad"),
            DeclareLaunchArgument("password", default_value="sensrad"),
            DeclareLaunchArgument("format", default_value="jpeg"), # or rgb
            DeclareLaunchArgument("frame_id", default_value="camera_1"),
            DeclareLaunchArgument("width", default_value="1920"),
            DeclareLaunchArgument("height", default_value="1080"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "map",
                    LaunchConfiguration("frame_id"),
                ],
            ),
            Node(
                package="axis_rtsp",
                # namespace="camera_1",
                executable="rtsp_image",
                name="rtsp_image",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log-level"),
                ],
                parameters=[
                    {
                        "ip": LaunchConfiguration("ip"),
                        "user": LaunchConfiguration("user"),
                        "password": LaunchConfiguration("password"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "width":  LaunchConfiguration("width"),
                        "height":  LaunchConfiguration("height"),
                        # Camera info (example values)
                        "distortion_model": "plumb_bob",
                        "distortion": [
                            D_1,
                            D_2,
                            D_3,
                            D_4,
                            D_5,
                        ],
                        "intrinsic": [
                            F_X,
                            0.0,
                            C_X,
                            0.0,
                            F_Y,
                            C_Y,
                            0.0,
                            0.0,
                            1.0,
                        ],
                        "rectification": [1.0, 0.0, 0.0,
                                          0.0, 1.0, 0.0,
                                          0.0, 0.0, 1.0],
                        "projection": [
                            F_X,
                            0.0,
                            C_X,
                            0.0,
                            0.0,
                            F_Y,
                            C_Y,
                            0.0,
                            0.0,
                            0.0,
                            1.0,
                            0.0,
                        ],
                    }
                ],
            ),
        ]
    )
