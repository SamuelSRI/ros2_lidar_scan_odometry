from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share_dir = get_package_share_directory("ros2_lidar_scan_odometry")

    config_file = os.path.join(
        package_share_dir,
        "config",
        "lidar_scan_odometry.yaml"
    )

    lidar_scan_odometry_node = Node(
        package="ros2_lidar_scan_odometry",
        executable="lidar_scan_odometry_node",
        name="lidar_scan_odometry_node",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription([
        lidar_scan_odometry_node,
    ])
