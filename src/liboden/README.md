Copyright (c) Sensrad 2023

## liboden

This is a library designed for advanced radar perception tasks. It
serves as the core processing unit for radar data, enabling robust and efficient
perception capabilities.

## Functional Overview

The library offers a range of radar data processing and perception functionalities:

- Ego-motion: Determines the 3D velocity of the platform carrying the radar.
- Clustering: Identifies moving objects in the radar data and groups them into clusters
for further analysis.
- Ground-plane estimation: Calculates the plane representing the ground surface in radar
data.
- Free-space: Estimates the area in front of the radar which is free from vertical
obstacles
- Disturbance suppression: Mitigates the effects of vertical arches in radar data,
specifically for firmware versions <=1.8.1.
- Local-max: Reduces the spread of the radar point cloud to enhance data quality.
- Tracking: estimation of the 3D position, velocity, and extent of dynamic objects.

## Integration with ROS2

The perception library seamlessly integrates with the oden ROS2 node, providing the
underlying perception functionalities. For detailed instructions on using liboden
within the oden ROS2 node, please review the oden README.
