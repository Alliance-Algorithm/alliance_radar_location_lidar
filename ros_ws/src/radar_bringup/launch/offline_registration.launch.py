#!/usr/bin/env python3
"""Launch offline registration visualization for Foxglove.

Usage:
    ros2 launch radar_bringup offline_registration.launch.py \
        map_path:=/path/to/map.pcd \
        scan_path:=/path/to/scan.pcd
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    radar_dir = get_package_share_directory("radar_lidar")

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(radar_dir, "config", "offline_registration.yaml"),
        description="Offline registration parameter YAML",
    )
    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value="",
        description="Absolute path to static map PCD",
    )
    scan_path_arg = DeclareLaunchArgument(
        "scan_path",
        default_value="",
        description="Absolute path to scan PCD",
    )

    offline_node = Node(
        package="radar_lidar",
        executable="offline_test_node",
        name="offline_test_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "map_path": LaunchConfiguration("map_path"),
                "scan_path": LaunchConfiguration("scan_path"),
            },
        ],
    )

    return LaunchDescription([
        params_arg,
        map_path_arg,
        scan_path_arg,
        offline_node,
    ])
