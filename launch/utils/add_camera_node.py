"""Module for launching camera node."""
# Copyright (c) Sensrad 2023-2024

import os
import sys
import re
import yaml
from yaml.loader import SafeLoader
from launch_ros.actions import Node

from utils.add_transform_node import add_transform_node
from utils.check_node_presence import check


def add_camera_node(nodes_to_launch, file_path, config, base_link):
    """Add camera node to launch description."""

    # Extract data that defines names and topics
    camera_id = config["id"]
    translation = config["translation"]
    rotation = config["rotation"]
    camera_topic_name = config["camera_topic_name"]
    camera_topic_base_name = f"{camera_topic_name}_{camera_id}"

    # Select and extract camera intrinsics
    camera_intrinic_file = os.path.join(file_path, "config", "camera_intrinsic.yaml")
    with open(camera_intrinic_file, "rt", encoding="utf8") as i_f:
        cam_intrinsic = yaml.load(i_f, Loader=SafeLoader)
        chosen_config = cam_intrinsic[config["variant"]]

    if chosen_config["distortion_model"] == 'plumb_bob':
        distortion = [
            chosen_config["d_1"],
            chosen_config["d_2"],
            chosen_config["d_3"],
            chosen_config["d_4"],
            chosen_config["d_5"],
        ]
    elif chosen_config["distortion_model"] == 'rational_polynomial':
        distortion = [
            chosen_config["d_1"],
            chosen_config["d_2"],
            chosen_config["d_3"],
            chosen_config["d_4"],
            chosen_config["d_5"],
            chosen_config["d_6"],
            chosen_config["d_7"],
            chosen_config["d_8"],
        ]
    else:
        raise ValueError('Distortion model must be plumb_bob or rational_polynomial')

    intrinsic = [
        chosen_config["f_x"],
        0.0,
        chosen_config["c_x"],
        0.0,
        chosen_config["f_y"],
        chosen_config["c_y"],
        0.0,
        0.0,
        1.0,
    ]
    rectification = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    projection = [
        chosen_config["f_x"],
        0.0,
        chosen_config["c_x"],
        0.0,
        0.0,
        chosen_config["f_y"],
        chosen_config["c_y"],
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
    ]

    node_started = False

    # Add camera node
    if (config["type"] == "usb") and check("usbcam"):
        camera_address = config["address"]
        cam_l_info = os.popen("cam -l").read()

        cam_model = config["model"]
        if cam_model not in cam_l_info:
            sys.exit(f"Camera {cam_model} not found.")

        # Attempt to auto detect address for this camera - works only for single camera setups
        if camera_address == "auto":
            result = re.findall(f"{cam_model}" + ".*" + r"\(.*?\)", cam_l_info)
            if len(result) != 1:
                sys.exit(f"Could not auto detect address for camera {cam_model}.")

            camera_address = result[0].split("(")[1].split(")")[0]

            print(f"Auto detected address for camera {cam_model}: {camera_address}")
        node_started = True
        nodes_to_launch.append(
            Node(
                package="usbcam",
                namespace=camera_topic_base_name,
                executable="libcamera_node",
                name="libcamera_node",
                arguments=["--ros-args", "--log-level", "info"],
                remappings=[
                    ("image_compressed", "compressed"),  # remap placeholder
                ],
                parameters=[
                    {
                        "camera_id": camera_address,
                        "pixel_format": "MJPEG",
                        "width": chosen_config["width"],
                        "height": chosen_config["height"],
                        "frame_id": f"camera_{camera_id}",
                        "decimation": 2,
                        "distortion_model": chosen_config["distortion_model"],
                        "distortion": distortion,
                        "intrinsic": intrinsic,
                        "rectification": rectification,
                        "projection": projection,
                    }
                ],
            )
        )
    elif (config["type"] == "oak") and check("usbcam"):
        node_started = True
        nodes_to_launch.append(
            Node(
                package="usbcam",
                namespace=camera_topic_base_name,
                executable="depthai_node",
                name="depthai_node",
                arguments=["--ros-args", "--log-level", "info"],
                remappings=[
                    ("image_compressed", "compressed"),  # remap placeholder
                ],
                parameters=[
                    {
                        "frame_id": f"camera_{camera_id}",
                        "distortion_model": chosen_config["distortion_model"],
                        "distortion": distortion,
                        "intrinsic": intrinsic,
                        "rectification": rectification,
                        "projection": projection,
                    }
                ],
            )
        )

    elif (config["type"] == "axis") and check("axis_rtsp"):
        node_started = True
        nodes_to_launch.append(
            Node(
                package="axis_rtsp",
                namespace=camera_topic_base_name,
                executable="rtsp_image",
                name="rtsp_image",
                arguments=["--ros-args", "--log-level", "info"],
                parameters=[
                    {
                        "ip": config["ip"],
                        "user": config["user"],
                        "password": config["password"],
                        "frame_id": f"camera_{camera_id}",
                        # Camera info (example values)
                        "width": chosen_config["width"],
                        "height": chosen_config["height"],
                        "distortion_model": chosen_config["distortion_model"],
                        "distortion": distortion,
                        "intrinsic": intrinsic,
                        "rectification": rectification,
                        "projection": projection,
                        "format": config["format"],
                    }
                ],
            )
        )

    # TF transform map-camera
    if node_started:
        add_transform_node(
            nodes_to_launch,
            base_link,
            f"camera_{camera_id}",
            translation,
            rotation,
        )
