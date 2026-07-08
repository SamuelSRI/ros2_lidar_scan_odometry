# ros2_lidar_scan_odometry

Simple ROS2 C++ package that estimates local 2D odometry from a 2D LiDAR `LaserScan`.

The node subscribes to a LiDAR scan topic, performs a basic 2D ICP scan matching between consecutive scans, and publishes a `nav_msgs/msg/Odometry` message on `/lidar/odom`.

This package is intended to provide a LiDAR odometry input for an EKF, for example with `robot_localization`.

## Overview

Recommended localization architecture:

```text
/scan
  -> ros2_lidar_scan_odometry
  -> /lidar/odom
  -> robot_localization EKF
  -> /odometry/filtered
```

The package does **not** publish TF by default.
The EKF should remain the only node publishing the `odom -> base_link` transform.

## Topics

### Subscribed

| Topic   | Type                        | Description         |
| ------- | --------------------------- | ------------------- |
| `/scan` | `sensor_msgs/msg/LaserScan` | Input 2D LiDAR scan |

### Published

| Topic         | Type                    | Description              |
| ------------- | ----------------------- | ------------------------ |
| `/lidar/odom` | `nav_msgs/msg/Odometry` | Estimated LiDAR odometry |

## Parameters

| Parameter                     |       Default | Description                                  |
| ----------------------------- | ------------: | -------------------------------------------- |
| `scan_topic`                  |       `/scan` | Input scan topic                             |
| `odom_topic`                  | `/lidar/odom` | Output odometry topic                        |
| `odom_frame`                  |        `odom` | Odometry frame                               |
| `base_frame`                  |   `base_link` | Robot base frame                             |
| `min_range`                   |        `0.10` | Minimum valid LiDAR range                    |
| `max_range`                   |        `12.0` | Maximum valid LiDAR range                    |
| `max_points`                  |         `500` | Maximum number of scan points used by ICP    |
| `icp_iterations`              |          `10` | Number of ICP iterations per scan            |
| `max_correspondence_distance` |        `0.35` | Maximum distance for point matching          |
| `max_translation_per_scan`    |        `0.50` | Reject odometry jumps above this translation |
| `max_rotation_per_scan`       |        `0.50` | Reject odometry jumps above this rotation    |
| `xy_covariance`               |        `0.05` | Pose covariance for X and Y                  |
| `yaw_covariance`              |        `0.05` | Pose covariance for yaw                      |

## Build

Clone the package inside a ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone git@github.com:SamuelSRI/ros2_lidar_scan_odometry.git
```

Build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select ros2_lidar_scan_odometry
source install/setup.bash
```

## Run

Make sure your LiDAR is already publishing a `LaserScan` topic:

```bash
ros2 topic list | grep scan
ros2 topic info /scan
```

Launch the odometry node:

```bash
ros2 launch ros2_lidar_scan_odometry lidar_scan_odometry.launch.py
```

Check the output:

```bash
ros2 topic info /lidar/odom
ros2 topic echo /lidar/odom --once
```

Expected output type:

```text
nav_msgs/msg/Odometry
```

## Configuration

The main configuration file is:

```text
config/lidar_scan_odometry.yaml
```

Example:

```yaml
lidar_scan_odometry_node:
  ros__parameters:
    scan_topic: /scan
    odom_topic: /lidar/odom

    odom_frame: odom
    base_frame: base_link

    min_range: 0.10
    max_range: 12.0
    max_points: 500

    icp_iterations: 10
    max_correspondence_distance: 0.35

    max_translation_per_scan: 0.50
    max_rotation_per_scan: 0.50

    xy_covariance: 0.05
    yaw_covariance: 0.05
```

If your LiDAR publishes on another topic, change:

```yaml
scan_topic: /your_scan_topic
```

## Integration with robot_localization

The `/lidar/odom` topic can be added as an odometry input in an EKF configuration.

Example:

```yaml
odom1: /lidar/odom

odom1_config: [
  true,  true,  false,
  false, false, true,
  false, false, false,
  false, false, false,
  false, false, false
]

odom1_queue_size: 10
odom1_differential: false
odom1_relative: false
odom1_pose_rejection_threshold: 3.0
odom1_twist_rejection_threshold: 2.0
```

The EKF can then fuse:

```text
/odom
/imu/raw
/lidar/odom
```

and publish:

```text
/odometry/filtered
```

## TF behavior

This package does not publish TF.

Only one node should publish the `odom -> base_link` transform.
In the recommended setup, this is handled by `robot_localization`.

## Notes

This package implements a simple ICP-based 2D scan odometry. It is useful for testing and for adding a LiDAR-based odometry source to a local EKF.

For more robust outdoor localization, a dedicated LiDAR odometry or SLAM package may be more appropriate.

## License

MIT
