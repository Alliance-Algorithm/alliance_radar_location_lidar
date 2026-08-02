#!/usr/bin/env python3
"""competition.launch.py — 比赛全流程启动。

完整链路:
  相机驱动 → LiDAR 配准 (GICP + static TF) →
  视觉检测 (radar_camera_node) + LiDAR 聚类 (radar_lidar_node) →
  传感器融合 (radar_fusion_node) →
  ZMQ 桥接 (radar_bridge_node)

用法:
    ros2 launch radar_bringup competition.launch.py side:=red
    ros2 launch radar_bringup competition.launch.py side:=blue map_path:=/path/to/map.pcd
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_camera_node(context: LaunchContext):
    side_val   = LaunchConfiguration("side").perform(context)
    bringup_dir = get_package_share_directory("radar_bringup")
    fusion_dir  = get_package_share_directory("radar_fusion")
    camera_config = os.path.join(bringup_dir, "config", "camera", "radar_camera.yaml")

    # enemy_color: 我方红→打蓝, 我方蓝→打红
    enemy_color = "red" if side_val == "blue" else "blue"

    # 相机在地图中的绝对位姿由 side 决定（由 GICP 初始位姿 + 传感器外参推算）
    camera_pose_yaml = os.path.join(
        bringup_dir, "config", "camera", f"{side_val}_camera_pose.yaml"
    )
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

    return [
        # 3. 视觉检测 (L1/L2/L3 TensorRT)
        Node(
            package="radar_camera",
            executable="radar_camera_node",
            name="radar_camera_node",
            output="screen",
            parameters=[
                camera_config,
                camera_pose_yaml,
                {"pub_topic_name": "/radar_camera/robot_pose"},
                {"enemy_color": enemy_color},
                recording_parameters,
            ],
        ),

        # 4. 传感器融合 (camera + lidar → fused tracks → LidarLocation)
        Node(
            package="radar_fusion",
            executable="radar_fusion_node",
            name="radar_fusion_node",
            output="screen",
            parameters=[
                os.path.join(fusion_dir, "config", "runtime.yaml"),
                # 未观测到的目标槽位按比赛时间填充官方数据众数格默认位置
                {"default_positions_path": "/workspace/model/default_positions.sqlite"},
                {"enemy_color": enemy_color},
            ],
        ),
    ]


def generate_launch_description():
    bringup_dir = get_package_share_directory("radar_bringup")

    side_lc     = LaunchConfiguration("side")
    map_path_lc = LaunchConfiguration("map_path")
    sensor_lc   = LaunchConfiguration("sensor")
    return LaunchDescription([
        DeclareLaunchArgument("side", default_value="red",
            description="场地侧: red | blue"),
        DeclareLaunchArgument("map_path",
            default_value="/workspace/model/generated/jinan_field_map_reg.pcd",
            description="地图 PCD 路径 (默认济南场地配准地图，无墙版实测配准更准)"),
        DeclareLaunchArgument("sensor", default_value="odin",
            description="雷达型号: odin | mid70"),
        DeclareLaunchArgument("enable_raw_recording", default_value="true",
            description="启用原始相机录制（默认开，比赛录像回放复盘用）"),
        DeclareLaunchArgument("recording_output_dir", default_value="/model/devio"),
        DeclareLaunchArgument("recording_width", default_value="5472"),
        DeclareLaunchArgument("recording_height", default_value="3648"),
        DeclareLaunchArgument("recording_fps", default_value="20"),
        DeclareLaunchArgument("recording_bitrate", default_value="40000000"),
        DeclareLaunchArgument("recording_gop", default_value="20"),
        DeclareLaunchArgument("recording_encoder", default_value="h264_nvenc"),
        DeclareLaunchArgument("recording_segment_duration_sec", default_value="60"),
        DeclareLaunchArgument("recording_buffer_pool_frames", default_value="8"),
        DeclareLaunchArgument("recording_max_buffer_bytes", default_value="480000000"),

        # 1. 相机驱动 (SHM 写 /hikcamera_shm)
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "hikcamera.launch.py"))),

        # 2. LiDAR 配准 (驱动 + GICP 定位 + static TF camera↔lidar↔map)
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "localization.launch.py")),
            launch_arguments={
                "sensor":   sensor_lc,
                "map_path": map_path_lc,
                "side":     side_lc,
            }.items()),

        # 3+4. 视觉检测 + 融合 (side 决定相机位姿和打击颜色)
        OpaqueFunction(function=_make_camera_node),

        # 5. ZMQ 桥接 (LidarLocation → 裁判系统)
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "radar_bridge.launch.py"))),
    ])
