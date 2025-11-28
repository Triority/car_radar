# Copyright (c) Sensrad 2023-2024

# pylint: disable=no-name-in-module

"""Module for launching runes node"""

from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration

from utils.check_node_presence import check


def add_perception_visualizer_node(nodes_to_launch, config):
    """Add polygon node(s) to launch description."""
    radar_id = config["id"]
    radar_topic_name = config["radar_topic_name"]

    namespace = f"oden_{radar_id}"

    radar_namespace = f"{radar_topic_name}_{radar_id}"

    # Check if runes node is present
    if check("runes"):
        nodes_to_launch.append(
            Node(
                package="runes",
                executable="perception_visualizer",
                namespace=namespace,
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log-level"),
                ],
                parameters=[
                    {"radar_topic": f"{radar_namespace}/radar_data"},
                    {"frame_id": f"radar_{radar_id}"},
                ],
            ),
        )
