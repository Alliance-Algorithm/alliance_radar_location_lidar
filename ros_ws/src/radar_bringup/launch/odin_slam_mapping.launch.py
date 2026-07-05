#!/usr/bin/env python3
"""Odin SLAM 建图启动文件。

赛前离线走图，产出 .bin 地图供重定位模式使用。只启动 Odin 驱动，不启动
radar_lidar（建图阶段不需要定位）。

用法:
    ros2 launch radar_bringup odin_slam_mapping.launch.py

走图结束后，在另一个终端执行：
    ros2 run odin_ros_driver set_param.sh save_map 1
（或参照 odin_ros_driver README 中 set_param.sh 的用法）
地图默认保存至 mapping_result_dest_dir（未指定时为驱动包内默认路径）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("radar_bringup")

    odin_config_arg = DeclareLaunchArgument(
        "odin_config",
        default_value=os.path.join(bringup_dir, "config", "lidar", "odin_driver_slam.yaml"),
        description="Odin 驱动 control_command.yaml（SLAM 建图模式）",
    )

    odin_node = Node(
        package="odin_ros_driver",
        executable="host_sdk_sample",
        name="host_sdk_sample",
        output="screen",
        parameters=[{"config_file": LaunchConfiguration("odin_config")}],
    )

    return LaunchDescription([
        odin_config_arg,
        odin_node,
    ])
