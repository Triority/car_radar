# Copyright (c) Sensrad 2023-2024

# There is a naming conflict on jenkins with 'launch'
# pylint: disable=no-name-in-module

"""Module for launching a radar node."""

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from utils.add_transform_node import add_transform_node
from utils.check_node_presence import check


def add_radar_node(
    nodes_to_launch,
    config,
    base_link,
):
    """Add radar node to launch description."""
    radar_id = config["id"]
    translation = config["translation"]
    rotation = config["rotation"]
    radar_ip = config["radar_ip"]
    radar_fw = config["fw"]
    udp_data_port = config["udp_data_port"]
    udp_log_port = config["udp_log_port"]
    tcp_ctrl_port = config["tcp_ctrl_port"]
    radar_topic_name = config["radar_topic_name"]

    namespace = f"{radar_topic_name}_{radar_id}"

    if check("hugin"):
        nodes_to_launch.append(
            Node(
                package="hugin",
                namespace=namespace,
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
                        "port": udp_log_port,  # log port
                        "log_data": False,
                        "publish_data": False,
                    }
                ],
            )
        )
        nodes_to_launch.append(
            Node(
                package="hugin",
                namespace=namespace,
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
                        "port": udp_data_port,  # pointcloud streaming port
                        "log_data": False,
                        "publish_data": True,
                    }
                ],
            )
        )
        nodes_to_launch.append(
            Node(
                package="hugin",
                namespace=namespace,
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
                        "frame_id": f"radar_{radar_id}",
                        "publish_pointcloud": True,
                        "publish_header": True,
                    }
                ],
            ),
        )

        nodes_to_launch.append(
            Node(
                package="hugin",
                namespace=namespace,
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
                        "fw_version": radar_fw,
                        "ip": radar_ip,
                        "port": tcp_ctrl_port,  # control port
                        "active_seq": 5,  # initial active sequence
                        "start_tx": False,  # if true start tx on startup
                    },
                ],
            ),
        )
        # TF transform map-radar
        add_transform_node(
            nodes_to_launch,
            base_link,
            f"radar_{radar_id}",
            translation,
            rotation,
        )

    # Launch one GUI per radar.
    if check("hugin_gtkgui"):
        nodes_to_launch.append(
            Node(
                package="hugin_gtkgui",
                namespace=namespace,
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
        )
