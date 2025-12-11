# Copyright (c) Sensrad 2023-2024
"""Combined launching of:
    classification
    hugin
    hugin_gtkgui
    usbcam
    radar_perception
    object_tracking
    visualization
    rviz2/foxglove
    perception_visualizer"""

# pylint: disable=no-name-in-module
# pylint: disable=wrong-import-order
# pylint: disable=wrong-import-position

# Add the path to the launch folder to the system path to avoid having external
# customers to add it to their PYTHONPATH
import os
import sys

sys.path.append(os.path.join(os.getcwd(), "launch"))
from utils.add_camera_node import add_camera_node
from utils.add_perception_node import add_perception_node
from utils.add_radar_node import add_radar_node
from utils.add_perception_visualizer_node import add_perception_visualizer_node
from utils.check_node_presence import check

from pathlib import Path
from shutil import which
import yaml
from yaml.loader import SafeLoader
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer


# ######################################################################################
# Main settings for user
# ######################################################################################

DEFAULT_CONFIG_NAME = "default.yaml"


# Base path for finding configuration files
FILE_PATH = Path.cwd()


def parse_config(args):
    """Function for parsing the config file argument."""
    # Parse launch arguments
    config_file = DEFAULT_CONFIG_NAME
    for arg in args:
        if "config:=" in arg:
            config_file = arg.strip("config:=")
            break
    return config_file


def get_visualizer_nodes(config):
    """Get the visualizer nodes to launch.
    Args: visualizer (str): Name of the visualizer to launch."""

    nodes = []

    # Visualizer of choice
    if config["visualizer"] == "rviz2":
        rviz_file_name = os.path.join(FILE_PATH, "config", config["visualizer_config"])
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_file_name],
            )
        )
    elif config["visualizer"] == "foxglove-studio":
        nodes.append(ExecuteProcess(cmd=["foxglove-studio"]))
        nodes.append(
            ExecuteProcess(
                cmd=["ros2 launch foxglove_bridge foxglove_bridge_launch.xml"],
                shell=True,
            )
        )
    elif config["visualizer"] == "foxglove-websocket":
        nodes.append(
            ExecuteProcess(
                cmd=["ros2 launch foxglove_bridge foxglove_bridge_launch.xml"],
                shell=True,
            )
        )

    return nodes


def generate_launch_description():
    """Define what to launch."""

    config_file = parse_config(sys.argv)
    print(f"CONFIG_NAME: {config_file}")

    # Get configurations per radar
    with open(
        os.path.join(FILE_PATH, "config", config_file), "rt", encoding="utf8"
    ) as i_f:
        config = yaml.load(i_f, Loader=SafeLoader)

    base_link = config["base_link"]

    # To support multi radar/camera, we loop over each entry in the config file
    nodes_to_launch = []
    nodes_to_launch.append(DeclareLaunchArgument("log-level", default_value="warn"))
    nodes_to_launch.append(DeclareLaunchArgument("bind", default_value="0.0.0.0"))

    visualizer = config["visualizer"]
    if visualizer != "foxglove-websocket" and which(visualizer) is None:
        print(f"Visualizer {visualizer} not found, defaulting to rviz2.")
        visualizer = "rviz2"

    for c_i in config:
        if c_i in ["visualizer", "base_link", "visualizer_config", "rectify"]:
            continue

        config_processed = False

        # Print and retrieve launch configuration
        print(f"Sensor: {c_i}")
        for p_i in config[c_i]:
            print(f"    {p_i}: {config[c_i][p_i]}")

        if config[c_i]["sensor"] == "radar" and config[c_i]["live"]:
            add_radar_node(nodes_to_launch, config[c_i], base_link)
            config_processed = True

        if config[c_i]["sensor"] == "radar" and config[c_i]["perception"]:
            add_perception_node(nodes_to_launch, config[c_i])
            add_perception_visualizer_node(nodes_to_launch, config[c_i])
            config_processed = True

        if config[c_i]["sensor"] == "camera" and visualizer == "rviz2":
            # If rviz is used for visualization decode compressed image into raw.
            config_processed = True
            camera_id = config[c_i]["id"]
            camera_topic_name = config[c_i]["camera_topic_name"]
            camera_topic_base_name = f"{camera_topic_name}_{camera_id}"

            nodes_to_launch.append(
                Node(
                    package="image_transport",
                    name="image_transporter",
                    executable="republish",
                    arguments=[
                        "compressed",
                        "raw",
                        "--ros-args",
                        "--remap",
                        f"in/compressed:={camera_topic_base_name}/compressed",
                        "--remap",
                        f"out:={camera_topic_base_name}/uncompressed",
                    ],
                )
            )

            # Compose and add rectification node
            if config["rectify"]:
                rectify_node_container = ComposableNodeContainer(
                    name="rectify_node_container",
                    namespace=camera_topic_base_name,
                    package="rclcpp_components",
                    executable="component_container",
                    composable_node_descriptions=[
                        ComposableNode(
                            package="image_proc",
                            plugin="image_proc::RectifyNode",
                            name="rectify_node",
                            # Remap subscribers and publishers
                            remappings=[
                                ("image", f"{camera_topic_base_name}/uncompressed"),
                                (
                                    "camera_info",
                                    f"{camera_topic_base_name}/camera_info",
                                ),
                                (
                                    "image_rect",
                                    f"{camera_topic_base_name}/uncompressed_rect",
                                ),
                                (
                                    "image_rect/compressed",
                                    f"{camera_topic_base_name}/_unused/compressed_rect",
                                ),
                                (
                                    "image_rect/compressedDepth",
                                    f"{camera_topic_base_name}/_unused/compressedDepth",
                                ),
                                (
                                    "image_rect/theora",
                                    f"{camera_topic_base_name}/_unused/theora",
                                ),
                            ],
                        ),
                    ],
                    output="screen",
                )
                nodes_to_launch.append(rectify_node_container)

        if config[c_i]["sensor"] == "camera" and config[c_i]["live"]:
            config_processed = True
            add_camera_node(nodes_to_launch, FILE_PATH, config[c_i], base_link)

        if config[c_i]["sensor"] == "imu" and config[c_i]["live"] and check("xbus"):
            config_processed = True
            package_prefix = get_package_share_directory("xbus")
            nodes_to_launch.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [package_prefix, "/launch/standard_launch.py"]
                    )
                )
            )

        if not config_processed:
            print(f"Sensor {config[c_i]['sensor']} not processed.")

    # ############ End of loop over sensors ############

    # Get visualizer nodes and add them to the launch description:
    nodes_to_launch = get_visualizer_nodes(config) + nodes_to_launch

    return LaunchDescription(nodes_to_launch)
