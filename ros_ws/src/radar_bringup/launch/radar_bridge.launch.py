#!/usr/bin/env python3
"""radar_bridge ROS2 ↔ ZMQ 桥接启动文件。

配置文件路径通过 ROS 参数文件加载（符合项目 convention：YAML → ros__parameters）。

用法:
    ros2 launch radar_bringup radar_bridge.launch.py
    ros2 launch radar_bringup radar_bridge.launch.py config_file:=/path/to/custom_config.yaml
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
        default_value=os.path.join(bringup_dir, "config", "bridge", "radar_bridge.yaml"),
        description="radar_bridge 配置文件路径（ROS2 parameter YAML）",
    )

    bridge_node = Node(
        package="radar_bridge",
        executable="radar_bridge_node",
        name="radar_bridge_node",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
    )

    return LaunchDescription([
        config_file_arg,
        bridge_node,
    ])
