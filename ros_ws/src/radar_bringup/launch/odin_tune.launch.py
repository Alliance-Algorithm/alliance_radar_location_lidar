#!/usr/bin/env python3
"""Odin1 直出点云聚类调参 launch：odin 驱动（cloud_raw + odometry）+ odin_tune_node。
不影响比赛主链路。用法:
    ros2 launch radar_bringup odin_tune.launch.py
调参: ros2 param set /odin_tune_node <param> <value>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("radar_bringup")
    radar_dir   = get_package_share_directory("radar_lidar")

    tune_params_arg = DeclareLaunchArgument(
        "tune_params",
        default_value=os.path.join(radar_dir, "config", "odin_tune.yaml"),
        description="odin_tune_node parameter YAML",
    )
    odin_config_arg = DeclareLaunchArgument(
        "odin_config",
        default_value=os.path.join(bringup_dir, "config", "lidar", "odin_driver_tune.yaml"),
        description="Odin driver control_command.yaml (senddtof + sendodom)",
    )

    odin_node = Node(
        package="odin_ros_driver",
        executable="host_sdk_sample",
        name="host_sdk_sample",
        output="screen",
        parameters=[{"config_file": LaunchConfiguration("odin_config")}],
    )

    tune_node = Node(
        package="radar_lidar",
        executable="odin_tune_node",
        name="odin_tune_node",
        output="screen",
        parameters=[LaunchConfiguration("tune_params")],
    )

    return LaunchDescription([
        tune_params_arg,
        odin_config_arg,
        odin_node,
        tune_node,
    ])
