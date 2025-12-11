# Copyright (c) Sensrad 2023
"""Utility function to check if a node is available to launch"""
from ament_index_python.packages import get_package_share_directory
import ament_index_python.packages


def check(node_name):
    """Check if node is available to launch"""
    node_is_available = True
    try:
        _ = get_package_share_directory(node_name)
    except ament_index_python.packages.PackageNotFoundError:
        node_is_available = False
    return node_is_available
