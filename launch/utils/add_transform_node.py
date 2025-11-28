"""Module for launching tf transform node."""
# Copyright (c) Sensrad 2023

from launch_ros.actions import Node

TRANSLATION_ORDER = ["--x", "--y", "--z"]
ROTATION_ORDER = ["--roll", "--pitch", "--yaw"]

# pylint: disable=dangerous-default-value


def add_transform_node(
    nodes_to_launch, parent, child, translation=[0, 0, 0], rotation=[0, 0, 0]
):
    """Add transform node to launch description."""
    # TF transform camera-radar
    transformation = []

    for val, t_o in zip(translation, TRANSLATION_ORDER):
        transformation.append(t_o)
        transformation.append(str(val))
    for val, r_o in zip(rotation, ROTATION_ORDER):
        transformation.append(r_o)
        transformation.append(str(val))
    transformation.extend(
        [
            "--frame-id",
            parent,
            "--child-frame-id",
            child,
        ]
    )

    nodes_to_launch.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=transformation,
        )
    )
