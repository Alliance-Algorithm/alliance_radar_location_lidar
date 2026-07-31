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

    # enemy_color: 我方红→打蓝, 我方蓝→打红
    enemy_color = "red" if side_val == "blue" else "blue"

    # 相机在地图中的绝对位姿由 side 决定（由 GICP 初始位姿 + 传感器外参推算）
    camera_pose_yaml = os.path.join(
        bringup_dir, "config", "camera", f"{side_val}_camera_pose.yaml"
    )

    return [
        # 3. 视觉检测 (L1/L2/L3 TensorRT)
        Node(
            package="radar_camera",
            executable="radar_camera_node",
            name="radar_camera_node",
            output="screen",
            parameters=[
                os.path.join(bringup_dir, "config", "camera", "radar_camera.yaml"),
                camera_pose_yaml,                           # 覆盖 rotation / translation
                {"pub_topic_name": "/camera/detection"},    # 与 fusion 订阅对齐
                {"enemy_color": enemy_color},
            ],
        ),

        # 4. 传感器融合 (camera + lidar → fused tracks → LidarLocation)
        Node(
            package="radar_fusion",
            executable="radar_fusion_node",
            name="radar_fusion_node",
            output="screen",
            parameters=[os.path.join(fusion_dir, "config", "runtime.yaml")],
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
            default_value="/workspace/model/generated/map.pcd",
            description="地图 PCD 路径"),
        DeclareLaunchArgument("sensor", default_value="odin",
            description="雷达型号: odin | mid70"),

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
