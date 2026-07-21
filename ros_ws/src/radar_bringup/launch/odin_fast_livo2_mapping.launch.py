#!/usr/bin/env python3
"""Odin1 + FAST-LIVO2 建图启动文件。

启动 Odin1 驱动（原始 dToF + IMU 模式，不用板载SLAM）+ radar_fast_livo2 节点。
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


def generate_launch_description():
    bringup_dir = get_package_share_directory("radar_bringup")
    livo2_dir = get_package_share_directory("radar_fast_livo2")

    # Odin1 驱动配置：原始 dToF + IMU，不开板载SLAM（odin_driver.yaml 关闭了IMU，
    # 仅用于纯点云调试；ONLY_LIO 必须用 odin_driver_lio.yaml 才能拿到 /odin1/imu）
    odin_config_arg = DeclareLaunchArgument(
        "odin_config",
        default_value=os.path.join(bringup_dir, "config", "lidar", "odin_driver_lio.yaml"),
        description="Odin 驱动 control_command.yaml（ONLY_LIO 数据模式：cloud_raw+imu）",
    )

    # FAST-LIVO2 节点参数
    livo2_config_arg = DeclareLaunchArgument(
        "livo2_config",
        default_value=os.path.join(livo2_dir, "config", "odin_livo2.yaml"),
        description="radar_fast_livo2 参数文件",
    )

    odin_node = Node(
        package="odin_ros_driver",
        executable="host_sdk_sample",
        name="host_sdk_sample",
        output="screen",
        parameters=[{"config_file": LaunchConfiguration("odin_config")}],
    )

    livo2_node = Node(
        package="radar_fast_livo2",
        executable="radar_fast_livo2_node",
        name="radar_fast_livo2_node",
        output="screen",
        parameters=[LaunchConfiguration("livo2_config")],
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
        static_tf_launch,
        odin_node,
        livo2_node,
    ])
