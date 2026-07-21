#!/usr/bin/env python3
"""competition.launch.py — 比赛全流程启动。

链路: 相机 → 雷达配准 → 传感器融合 → ZMQ 桥接 (+ 可选视觉检测)

用法:
    ros2 launch radar_bringup competition.launch.py side:=red
    ros2 launch radar_bringup competition.launch.py side:=blue map_path:=/path/to/map.pcd
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("radar_bringup")
    fusion_dir  = get_package_share_directory("radar_fusion")

    side_lc     = LaunchConfiguration("side")
    map_path_lc = LaunchConfiguration("map_path")
    sensor_lc   = LaunchConfiguration("sensor")

    return LaunchDescription([
        DeclareLaunchArgument("side", default_value="red",
            description="场地侧: red | blue"),
        DeclareLaunchArgument("map_path",
            default_value="/workspace/model/generated/map.pcd",
            description="地图 PCD 路径"),
        DeclareLaunchArgument("sensor", default_value="odin",
            description="雷达型号: odin | mid70"),

        # 1. 相机驱动
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "hikcamera.launch.py"))),

        # 2. 雷达配准 (LiDAR 驱动 + GICP + static TF)
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "localization.launch.py")),
            launch_arguments={
                "sensor":   sensor_lc,
                "map_path": map_path_lc,
                "side":     side_lc,
            }.items()),

        # 3. 传感器融合
        Node(package="radar_fusion", executable="radar_fusion_node",
             name="radar_fusion_node", output="screen",
             parameters=[os.path.join(fusion_dir, "config", "runtime.yaml")]),

        # 4. ZMQ 桥接
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "radar_bridge.launch.py"))),

        # 5. 视觉检测 (需 OpenVINO; 单独用 radar_camera.launch.py 启动)
    ])
