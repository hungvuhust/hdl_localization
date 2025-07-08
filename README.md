# HDL Localization

A ROS2 package for real-time 3D localization using point cloud data from LiDAR sensors.

## Overview

HDL Localization provides real-time 3D localization capabilities using point cloud matching algorithms. It supports various registration methods including NDT (Normal Distribution Transform) and GICP (Generalized Iterative Closest Point).

## Features

- Real-time 3D localization using point cloud data
- Support for multiple registration algorithms (NDT, GICP)
- IMU integration for improved prediction
- Global localization support
- Configurable parameters via YAML file

## Dependencies

- ROS2 (tested with Humble)
- PCL (Point Cloud Library)
- Eigen3
- OpenMP
- ndt_omp
- fast_gicp
- hdl_global_localization

## Installation

```bash
# Clone the repository
cd ~/ros2_ws/src
git clone <repository_url>

# Install dependencies
rosdep install --from-paths . --ignore-src -r -y

# Build the package
cd ~/ros2_ws
colcon build --packages-select hdl_localization
```

## Usage

### 1. Configuration

Edit the parameter file to match your setup:

```bash
nano config/hdl_localization_params.yaml
```

Key parameters to configure:
- `points_topic`: Point cloud topic from your LiDAR
- `imu_topic`: IMU topic (if using IMU)
- `odom_child_frame_id`: Frame ID for odometry child frame
- `globalmap_pcd`: Path to your map file

### 2. Launch the Node

```bash
ros2 launch hdl_localization hdl_localization.launch.py
```

### 3. Provide Initial Pose

Set the initial pose using RViz or publish to `/initialpose` topic:

```bash
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
  header: {frame_id: "map"},
  pose: {
    pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}
  }
}'
```

## Topics

### Subscribed Topics

- `/livox/lidar` (sensor_msgs/PointCloud2): Input point cloud
- `/livox/imu` (sensor_msgs/Imu): IMU data (optional)
- `/initialpose` (geometry_msgs/PoseWithCovarianceStamped): Initial pose
- `/globalmap` (sensor_msgs/PointCloud2): Global map point cloud

### Published Topics

- `/odom` (nav_msgs/Odometry): Localization result
- `/aligned_points` (sensor_msgs/PointCloud2): Aligned point cloud
- `/status` (hdl_localization/ScanMatchingStatus): Localization status
- `/globalmap` (sensor_msgs/PointCloud2): Global map (from server)

## Parameters

### HDL Localization Node

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `points_topic` | string | `/livox/lidar` | Point cloud topic |
| `imu_topic` | string | `/livox/imu` | IMU topic |
| `odom_child_frame_id` | string | `base_link` | Child frame for odometry |
| `robot_odom_frame_id` | string | `odom` | Robot odometry frame |
| `use_imu` | bool | `false` | Enable IMU integration |
| `reg_method` | string | `NDT_OMP` | Registration method |
| `ndt_resolution` | double | `1.0` | NDT resolution |
| `downsample_resolution` | double | `0.1` | Downsampling resolution |

### Globalmap Server Node

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `globalmap_pcd` | string | `map.pcd` | Path to map file |
| `convert_utm_to_local` | bool | `true` | Convert UTM to local coordinates |
| `downsample_resolution` | double | `0.1` | Map downsampling resolution |

## File Structure

```
hdl_localization/
├── config/
│   └── hdl_localization_params.yaml    # Configuration file
├── include/
│   └── hdl_localization/
│       ├── hdl_localization_node.hpp
│       └── globalmap_server_node.hpp
├── launch/
│   └── hdl_localization.launch.py      # Launch file
├── src/
│   ├── hdl_localization_main.cpp       # Main executable
│   ├── hdl_localization_node.cpp
│   └── globalmap_server_node.cpp
└── msg/
    └── ScanMatchingStatus.msg
```

## Troubleshooting

### Common Issues

1. **No localization output**: Check if initial pose is set and global map is loaded
2. **Poor localization accuracy**: Adjust NDT resolution and downsampling parameters
3. **High CPU usage**: Reduce point cloud density or increase downsampling resolution

### Debug Topics

Monitor these topics for debugging:
- `/status`: Localization status and matching scores
- `/aligned_points`: Visualize point cloud alignment in RViz

## License

BSD License

## Maintainer

Hung Vu (crusadertrask2k@gmail.com)