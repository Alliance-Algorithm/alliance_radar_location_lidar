#!/usr/bin/env python3
"""Hikcamera ROS driver 启动文件。

按 Odin 驱动模式，配置文件路径通过 ROS 参数 config_file 传入。

用法:
    ros2 launch radar_bringup hikcamera.launch.py
    ros2 launch radar_bringup hikcamera.launch.py config_file:=/path/to/custom_config.yaml
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
        default_value=os.path.join(bringup_dir, "config", "camera", "hikcamera.yaml"),
        description="hikcamera 驱动配置文件路径（YAML）",
    )

    hikcamera_node = Node(
        package="hikcamera_ros_driver",
        executable="hikcamera_ros_driver",
        name="hikcamera_ros_driver_node",
        output="screen",
        parameters=[{
            "config_file": LaunchConfiguration("config_file"),
        }],
    )

    return LaunchDescription([
        config_file_arg,
        hikcamera_node,
    ])
