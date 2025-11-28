# Copyright (c) Sensrad 2023
#
# oden_interfaces/msg/GroundPlane
#
# A custom message type for Sensrad GroundPlane data

# Header for time and frame information
std_msgs/Header header

# Coefficients of the ground plane
shape_msgs/Plane plane

# Validity flag
bool is_valid