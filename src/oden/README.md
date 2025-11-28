Copyright (c) Sensrad 2023-2024



# oden

The `oden` package in Sensrad's ROS2 suite is designed for advanced radar data processing, providing a range of perception functionalities to enhance the utility and accuracy of radar data.



### Key Functionalities

- **Ego-Motion Estimation**: Determines the 3D velocity of the platform carrying the radar.

- **Ground-Plane Estimation**: Computes the plane representing the ground in radar data.

- **Clustering and Detections**: Identifies and groups moving objects detected by the radar.

- **Free Space Estimation**: Analyzes radar data to estimate free space areas.

- **Local-Max Filtering**: Reduces the spread of the radar point cloud for more precise data.

- **Disturbance Reduction**: Minimizes the impact of "vertical arches" in radars with firmware version <=1.8.1.

- **Tracking of Dynamic Objects**: Estimates the 3D position and velocity and extent of objects which are in motion.

- **Classification of Dynamic Objects**: Estimates the type of tracked object, *i.e.* if it is a pedestrian, bicycle, car or truck.

  

### Published Topics

The `oden` node publishes several topics, each providing valuable data for perception and analysis:

- **ego_motion**: Outputs 3D velocities in the radar coordinate frame (x: right, y: forward, z: upward), along with a boolean `is_valid` flag indicating the validity of the data.
- **detections**: A list of detections derived from the radar data, intended for use by tracking systems.
- **extended_point_cloud**: An enhanced PointCloud2 topic that includes the original point cloud data along with additional attributes:
  - `motion_status`: Indicates the motion state of each point (0 = static, 1 = unknown, 2 = dynamic).
  - `delta_velocity`: The radial velocity in meters per second, adjusted for ego-motion.
  - `available_for_tracker`: A flag indicating whether a point was used in the clustering algorithm.
  - `cluster_idx`: Integer values representing the cluster to which a point belongs. A value of "-1" denotes a noise
  - `distance_to_ground_plane`: Signed orthogonal distance to the estimated ground-plane.
  point or a static point.
  - `disturbance_std`: A value >=0 indicating whether a point is affected by a disturbance.
  - `multi_path`: Boolean indicating a point is due to multi-path.
- **ground_plane_data**: Provides a model of the ground plane in the vicinity of the radar, formatted as Ax+By+Cz+D=0, along with a boolean `is_valid` flag.
- **free_space**: A polygon representation of the estimated free space area.
- **object_tracks**: A list of tracked dynamic objects. Each object is tracked in the 3D Cartesian radar coordinate frame and is represented with a 3D position and a 3D velocity. The extent of the object is also estimated and modeled as a 3D ellipsoid which is represented by a 3x3 symmetric positive definite matrix. 

These topics together offer essential data for effective radar-based perception and environmental understanding.
For details regarding the published data, please consult `oden_interfaces` and `liboden/include/liboden/IO_Types.hpp`.



### Configurable Parameters

The following ROS2 parameters can be adjusted either in the launch file or live:

- `radar_topic`: Topic name for radar point cloud data.
- `frame_id`: Frame identifier for the published extended point cloud.
- `dynamic`: Boolean to indicate if the platform is moving.
- `local_max`: Boolean to activate the local max algorithm.
- `suppress_disturbances`: Boolean to omit points affected by vertical arches.
- `vertical_height`: The vertical height of the radar installation above ground.
- `elevation_angle`: The elevation angle of the radar's boresight.

### Prerequisites

- Eigen3 version 3.3

- `oden_interfaces` package must be built and sourced.

- Availability of the `liboden` module.

  

## Install Instructions

To install and set up the `oden` package, follow these steps:

1. Build the package using colcon:

```bash
colcon build --packages-select oden
```

2. Source the installed packages to update your environment:

```bash
source install/setup.bash
```

3. To launch the radar perception functionality, use the provided launch file:

```bash
ros2 launch oden standard_launch.py
```
