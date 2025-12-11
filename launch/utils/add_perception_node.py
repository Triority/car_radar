"""Module for launching perception"""
# Copyright (c) 2023-2024 Sensrad

# pylint: disable=no-name-in-module

from launch_ros.actions import Node

from launch.substitutions.launch_configuration import (
    LaunchConfiguration,
)

from utils.add_transform_node import add_transform_node
from utils.check_node_presence import check


def add_perception_node(nodes_to_launch, config):
    """Add perception node(s) to launch description."""
    dynamic = config["dynamic"]
    vertical_height = config["vertical_height"]
    radar_elevation_angle = config["elevation_angle"]
    radar_id = config["id"]
    radar_topic_name = config["radar_topic_name"]
    if "object_classifier" in config:
        object_classifier = config["object_classifier"]
    else:
        object_classifier = None

    namespace = f"oden_{radar_id}"

    radar_namespace = f"{radar_topic_name}_{radar_id}"

    # Arbe RSL filter takes care of suppressing disturbances in FW 1.8.5 and above.
    if config["fw"] == "1.8.5":
        suppress_disturbances = False
    else:
        suppress_disturbances = True

    # Perception C++.
    if check("oden"):
        nodes_to_launch.append(
            Node(
                package="oden",
                executable="oden",
                namespace=namespace,
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log-level"),
                ],
                parameters=[
                    {"tracker_topic": "object_list"},
                    {"radar_topic": f"{radar_namespace}/radar_data"},
                    {"frame_id": f"radar_{radar_id}"},
                    {"dynamic": dynamic},  # Set to true if radar is in motion
                    {"local_max": True},
                    {"suppress_disturbances": suppress_disturbances},
                    {
                        "vertical_height": vertical_height
                    },  # Set to radar vertical height in meters
                    {
                        "radar_elevation_angle": radar_elevation_angle
                    },  # Set to radar elevation angle in degrees
                ],
            ),
        )
    add_transform_node(nodes_to_launch, f"radar_{radar_id}", f"tracker_{radar_id}")

    if object_classifier is not None and check("classification"):
        print(f"Object classifier: {object_classifier}")
        nodes_to_launch.append(
            Node(
                package="classification",
                name="classification",
                namespace=namespace,
                executable="classification_node",
                output="screen",
                parameters=[
                    {"object_classifier": f"{object_classifier}"},
                ],
            ),
        )
