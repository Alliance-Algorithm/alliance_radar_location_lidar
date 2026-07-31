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
    camera_config = os.path.join(bringup_dir, "config", "camera", "radar_camera.yaml")

    side_lc     = LaunchConfiguration("side")
    map_path_lc = LaunchConfiguration("map_path")
    sensor_lc   = LaunchConfiguration("sensor")
    recording_parameters = {
        "enable_raw_recording": LaunchConfiguration("enable_raw_recording"),
        "recording_output_dir": LaunchConfiguration("recording_output_dir"),
        "recording_width": LaunchConfiguration("recording_width"),
        "recording_height": LaunchConfiguration("recording_height"),
        "recording_fps": LaunchConfiguration("recording_fps"),
        "recording_bitrate": LaunchConfiguration("recording_bitrate"),
        "recording_gop": LaunchConfiguration("recording_gop"),
        "recording_encoder": LaunchConfiguration("recording_encoder"),
        "recording_segment_duration_sec": LaunchConfiguration(
            "recording_segment_duration_sec"),
        "recording_buffer_pool_frames": LaunchConfiguration("recording_buffer_pool_frames"),
        "recording_max_buffer_bytes": LaunchConfiguration("recording_max_buffer_bytes"),
    }

    return LaunchDescription([
        DeclareLaunchArgument("side", default_value="red",
            description="场地侧: red | blue"),
        DeclareLaunchArgument("map_path",
            default_value="/workspace/model/generated/map.pcd",
            description="地图 PCD 路径"),
        DeclareLaunchArgument("sensor", default_value="odin",
            description="雷达型号: odin | mid70"),
        DeclareLaunchArgument("enable_raw_recording", default_value="false",
            description="启用原始相机录制"),
        DeclareLaunchArgument("recording_output_dir", default_value="/data/competition/recordings"),
        DeclareLaunchArgument("recording_width", default_value="5472"),
        DeclareLaunchArgument("recording_height", default_value="3648"),
        DeclareLaunchArgument("recording_fps", default_value="20"),
        DeclareLaunchArgument("recording_bitrate", default_value="40000000"),
        DeclareLaunchArgument("recording_gop", default_value="20"),
        DeclareLaunchArgument("recording_encoder", default_value="h264_nvenc"),
        DeclareLaunchArgument("recording_segment_duration_sec", default_value="60"),
        DeclareLaunchArgument("recording_buffer_pool_frames", default_value="8"),
        DeclareLaunchArgument("recording_max_buffer_bytes", default_value="480000000"),

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

        # 3. 相机检测与可选原始录制
        Node(package="radar_camera", executable="radar_camera_node",
             name="radar_camera_node", output="screen",
             parameters=[camera_config, recording_parameters]),

        # 4. 传感器融合
        Node(package="radar_fusion", executable="radar_fusion_node",
             name="radar_fusion_node", output="screen",
             parameters=[os.path.join(fusion_dir, "config", "runtime.yaml")]),

        # 5. ZMQ 桥接
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "radar_bridge.launch.py"))),

    ])
