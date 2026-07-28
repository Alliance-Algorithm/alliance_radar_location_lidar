#!/usr/bin/env python3
"""Odin1 + FAST-LIVO2 建图启动文件。

启动 Odin1 官方 ROS2 driver launch（原始 dToF + IMU + 内置相机）+
radar_fast_livo2 节点。
第一阶段建议先用 slam_mode=1（ONLY_LIO）验证 Odin1 数据适配，
确认无误后再切到 odin_livo2.yaml 里的 slam_mode=2（LIVO）接入相机。

用法:
    ros2 launch radar_bringup odin_fast_livo2_mapping.launch.py

    # 覆盖参数文件
    ros2 launch radar_bringup odin_fast_livo2_mapping.launch.py \\
        livo2_config:=/path/to/custom_livo2.yaml

建图结束后地图会依据 odin_livo2.yaml 中 pcd_save_en / map_save_path 自动保存。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory("radar_bringup")
    livo2_dir = get_package_share_directory("radar_fast_livo2")

    # Odin1 驱动配置：原始 dToF + IMU + 内置相机，不开板载 SLAM。
    # 官方 driver launch 使用同一个 host_sdk_sample，但会同时启动其配套节点，
    # 并沿用 driver 自己的 config/calib.yaml 查找路径，保证 undistorted image
    # 的标定文件在 driver 初始化前可见。
    odin_config_arg = DeclareLaunchArgument(
        "odin_config",
        default_value=os.path.join(bringup_dir, "config", "lidar", "odin_driver_livo.yaml"),
        description="Odin 驱动 control_command.yaml（cloud_raw+imu+RGB）",
    )

    # FAST-LIVO2 节点参数
    livo2_config_arg = DeclareLaunchArgument(
        "livo2_config",
        default_value=os.path.join(livo2_dir, "config", "odin_builtin_camera_livo.yaml"),
        description="radar_fast_livo2 参数文件（Odin 内置相机 LIVO）",
    )

    rgb_config_arg = DeclareLaunchArgument(
        "rgb_config",
        default_value=os.path.join(
            get_package_share_directory("radar_fast_livo2_rgb"),
            "config", "odin_rgb_map.yaml"),
        description="Odin1 RGB colorizer 参数文件",
    )

    pcd_save_en_arg = DeclareLaunchArgument(
        "pcd_save_en",
        default_value="false",
        description="Enable geometry PCD accumulation and shutdown save",
    )
    map_save_path_arg = DeclareLaunchArgument(
        "map_save_path",
        default_value="/tmp/fast_livo2_map.pcd",
        description="Geometry PCD path used when the LIVO node exits",
    )
    pcd_save_warmup_frames_arg = DeclareLaunchArgument(
        "pcd_save_warmup_frames",
        default_value="30",
        description="Frames skipped before geometry PCD accumulation",
    )

    odin_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("odin_ros_driver"),
                "launch",
                "odin1_ros2.launch.py",
            )
        ),
        launch_arguments={
            "config_file": LaunchConfiguration("odin_config"),
        }.items(),
    )

    livo2_node = Node(
        package="radar_fast_livo2",
        executable="radar_fast_livo2_node",
        name="radar_fast_livo2_node",
        output="screen",
        parameters=[
            LaunchConfiguration("livo2_config"),
            {
                "pcd_save_en": ParameterValue(
                    LaunchConfiguration("pcd_save_en"), value_type=bool),
                "map_save_path": LaunchConfiguration("map_save_path"),
                "pcd_save_warmup_frames": ParameterValue(
                    LaunchConfiguration("pcd_save_warmup_frames"), value_type=int),
            },
        ],
    )

    rgb_node = Node(
        package="radar_fast_livo2_rgb",
        executable="rgb_colorizer_node",
        name="rgb_colorizer_node",
        output="screen",
        parameters=[LaunchConfiguration("rgb_config")],
    )

    # GTSAM loop closure backend (decoupled from LIO, aligned with SPARK-FAST-LIO)
    gtsam_node = Node(
        package="radar_fast_livo2",
        executable="gtsam_backend_node",
        name="gtsam_backend_node",
        output="screen",
        parameters=[{
            "odom_topic": "/fast_livo2/odom",
            "scan_topic": "/fast_livo2/cloud_lidar",
            "keyframe_dist": 0.5,
            "loop_radius": 1.0,
            "loop_min_skip": 20,
            "map_frame": "map",
        }],
    )

    # 静态 TF（base_link → odin1 等外参树），与其他 launch 保持一致
    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "static_tf.launch.py")
        )
    )

    return LaunchDescription([
        odin_config_arg,
        livo2_config_arg,
        rgb_config_arg,
        pcd_save_en_arg,
        map_save_path_arg,
        pcd_save_warmup_frames_arg,
        static_tf_launch,
        odin_launch,
        livo2_node,
        rgb_node,
        gtsam_node,
    ])
