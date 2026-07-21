#!/usr/bin/env python3
"""radar_camera 目标检测启动文件。

用法:
    ros2 launch radar_bringup radar_camera.launch.py
    ros2 launch radar_bringup radar_camera.launch.py config_file:=/path/to/custom_config.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("radar_bringup")

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=os.path.join(bringup_dir, "config", "camera", "radar_camera.yaml"),
        description="radar_camera 配置文件路径",
    )

    camera_node = Node(
        package="radar_camera",
        executable="radar_camera_node",
        name="radar_camera_node",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
    )

    return LaunchDescription([
        config_file_arg,
        camera_node,
    ])
