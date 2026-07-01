#!/usr/bin/env python3
"""radar_lidar 通用定位启动文件。

按 sensor 参数选择对应的参数 YAML，并按需启动传感器驱动。

用法:
    ros2 launch radar_bringup localization.launch.py sensor:=odin map_path:=/path/to/map.pcd
    ros2 launch radar_bringup localization.launch.py sensor:=mid70 map_path:=/path/to/map.pcd

参数:
    sensor   传感器型号: odin | mid70  (默认 odin)
    map_path 地图 PCD 绝对路径          (默认 /workspace/model/generated/map.pcd)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _make_radar_node(context: LaunchContext):
    sensor_val = LaunchConfiguration("sensor").perform(context)
    radar_dir = get_package_share_directory("radar_lidar")
    radar_params = os.path.join(radar_dir, "config", sensor_val + ".yaml")
    map_path = LaunchConfiguration("map_path").perform(context)

    return [
        Node(
            package="radar_lidar",
            executable="radar_lidar_node",
            name="radar_lidar_node",
            output="screen",
            parameters=[
                radar_params,
                {"map_path": map_path},
            ],
        )
    ]


def generate_launch_description():
    bringup_dir = get_package_share_directory("radar_bringup")

    sensor_arg = DeclareLaunchArgument(
        "sensor",
        default_value="odin",
        description="传感器型号: odin | mid70",
    )
    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value="/workspace/model/generated/map.pcd",
        description="地图 PCD 绝对路径",
    )

    odin_node = Node(
        package="odin_ros_driver",
        executable="host_sdk_sample",
        name="host_sdk_sample",
        output="screen",
        parameters=[
            {
                "config_file": os.path.join(
                    bringup_dir, "config", "lidar", "odin_driver.yaml"
                ),
            }
        ],
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration("sensor"), "' == 'odin'"]
            )
        ),
    )

    radar_node = OpaqueFunction(function=_make_radar_node)

    return LaunchDescription([
        sensor_arg,
        map_path_arg,
        odin_node,
        radar_node,
    ])
