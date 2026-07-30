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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("radar_bringup")
    fusion_dir  = get_package_share_directory("radar_fusion")

    side_lc                 = LaunchConfiguration("side")
    map_path_lc             = LaunchConfiguration("map_path")
    sensor_lc               = LaunchConfiguration("sensor")
    registration_timeout_lc = LaunchConfiguration("registration_timeout_sec")
    camera_config_lc        = LaunchConfiguration("camera_config")
    fusion_config_lc        = LaunchConfiguration("fusion_config")
    bridge_config_lc        = LaunchConfiguration("bridge_config")
    enable_camera_lc        = LaunchConfiguration("enable_camera")
    enable_legacy_video_lc  = LaunchConfiguration("enable_legacy_video")

    return LaunchDescription([
        DeclareLaunchArgument("side", default_value="red",
            description="场地侧: red | blue"),
        DeclareLaunchArgument("map_path",
            default_value="/workspace/model/generated/map.pcd",
            description="地图 PCD 路径"),
        DeclareLaunchArgument("sensor", default_value="odin",
            description="雷达型号: odin | mid70"),
        DeclareLaunchArgument("registration_timeout_sec", default_value="30.0",
            description="Maximum seconds allowed for LiDAR registration"),
        DeclareLaunchArgument("camera_config",
            default_value=os.path.join(bringup_dir, "config", "camera", "radar_camera.yaml"),
            description="radar_camera runtime parameter YAML"),
        DeclareLaunchArgument("fusion_config",
            default_value=os.path.join(fusion_dir, "config", "runtime.yaml"),
            description="radar_fusion runtime parameter YAML"),
        DeclareLaunchArgument("bridge_config",
            default_value=os.path.join(bringup_dir, "config", "bridge", "radar_bridge.yaml"),
            description="radar_bridge runtime parameter YAML"),
        DeclareLaunchArgument("enable_camera", default_value="true",
            description="Start camera inference"),
        DeclareLaunchArgument("enable_legacy_video", default_value="false",
            description="Enable the legacy bridge video stream"),

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
                "registration_timeout_sec": registration_timeout_lc,
            }.items()),

        # 3. 视觉检测
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "radar_camera.launch.py")),
            launch_arguments={"config_file": camera_config_lc}.items(),
            condition=IfCondition(enable_camera_lc)),

        # 4. 传感器融合
        Node(package="radar_fusion", executable="radar_fusion_node",
             name="radar_fusion_node", output="screen",
             parameters=[fusion_config_lc]),

        # 5. ZMQ 桥接
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "radar_bridge.launch.py")),
            launch_arguments={
                "config_file": bridge_config_lc,
                "enable_video_stream": enable_legacy_video_lc,
            }.items()),
    ])
