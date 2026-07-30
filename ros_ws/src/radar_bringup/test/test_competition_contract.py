import importlib.util
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node


EXPECTED_ARGUMENTS = {
    "side",
    "sensor",
    "map_path",
    "registration_timeout_sec",
    "camera_config",
    "fusion_config",
    "bridge_config",
    "enable_camera",
    "enable_legacy_video",
}


def load_launch(package: str, filename: str):
    launch_path = Path(get_package_share_directory(package)) / "launch" / filename
    spec = importlib.util.spec_from_file_location(filename, launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def argument_names(description):
    return {
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }


def argument_defaults(description):
    context = LaunchContext()
    return {
        entity.name: perform_substitutions(context, entity.default_value)
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }


def include_with_arguments(description, expected_names):
    for entity in description.entities:
        if not isinstance(entity, IncludeLaunchDescription):
            continue
        if forwarded_argument_names(entity) == expected_names:
            return entity
    raise AssertionError(f"missing include with arguments {expected_names}")


def forwarded_argument_names(include):
    return {name for name, _ in include.launch_arguments}


def resolved_forwarded_arguments(include):
    sentinels = {name: f"sentinel-{name}" for name in EXPECTED_ARGUMENTS}
    context = LaunchContext()
    context.launch_configurations.update(sentinels)
    return {
        name: perform_substitutions(context, normalize_to_list_of_substitutions(value))
        for name, value in include.launch_arguments
    }


def test_competition_exposes_complete_argument_contract_without_h264_controls():
    description = load_launch("radar_bringup", "competition.launch.py")
    names = argument_names(description)

    assert names == EXPECTED_ARGUMENTS
    assert all("h264" not in name.lower() for name in names)
    defaults = argument_defaults(description)
    assert defaults["side"] == "red"
    assert defaults["sensor"] == "odin"
    assert defaults["map_path"] == "/workspace/model/generated/map.pcd"
    assert defaults["registration_timeout_sec"] == "30.0"
    assert defaults["enable_camera"] == "true"
    assert defaults["enable_legacy_video"] == "false"
    assert defaults["camera_config"].endswith("/config/camera/radar_camera.yaml")
    assert defaults["fusion_config"].endswith("/config/runtime.yaml")
    assert defaults["bridge_config"].endswith("/config/bridge/radar_bridge.yaml")


def test_competition_forwards_registration_and_explicit_runtime_configs():
    description = load_launch("radar_bringup", "competition.launch.py")

    localization_names = {
        "sensor",
        "map_path",
        "side",
        "registration_timeout_sec",
    }
    localization = include_with_arguments(description, localization_names)
    assert resolved_forwarded_arguments(localization) == {
        name: f"sentinel-{name}" for name in localization_names
    }

    camera = include_with_arguments(description, {"config_file"})
    assert resolved_forwarded_arguments(camera) == {
        "config_file": "sentinel-camera_config"
    }
    assert isinstance(camera.condition, IfCondition)

    bridge = include_with_arguments(description, {"config_file", "enable_video_stream"})
    assert resolved_forwarded_arguments(bridge) == {
        "config_file": "sentinel-bridge_config",
        "enable_video_stream": "sentinel-enable_legacy_video",
    }

    fusion_nodes = [
        entity
        for entity in description.entities
        if isinstance(entity, Node) and entity.node_package == "radar_fusion"
    ]
    assert len(fusion_nodes) == 1
    fusion_parameter_files = fusion_nodes[0]._Node__parameters
    assert len(fusion_parameter_files) == 1
    context = LaunchContext()
    context.launch_configurations["fusion_config"] = "sentinel-fusion_config"
    assert fusion_parameter_files[0].evaluate(context) == Path("sentinel-fusion_config")


def test_nested_launches_accept_forwarded_config_and_video_controls():
    camera = load_launch("radar_bringup", "radar_camera.launch.py")
    bridge = load_launch("radar_bringup", "radar_bridge.launch.py")

    assert argument_names(camera) == {"config_file"}
    assert argument_names(bridge) == {"config_file", "enable_video_stream"}


def test_all_default_runtime_configs_are_installed():
    bringup_share = Path(get_package_share_directory("radar_bringup"))
    fusion_share = Path(get_package_share_directory("radar_fusion"))

    assert (bringup_share / "config" / "camera" / "radar_camera.yaml").is_file()
    assert (bringup_share / "config" / "bridge" / "radar_bridge.yaml").is_file()
    fusion_config = fusion_share / "config" / "runtime.yaml"
    assert fusion_config.is_file()

    with fusion_config.open(encoding="utf-8") as stream:
        parameters = yaml.safe_load(stream)["radar_fusion_node"]["ros__parameters"]
    assert parameters == {
        "gate_distance": 1.0,
        "track_timeout_sec": 1.5,
        "min_hits_to_confirm": 3,
        "max_misses_before_delete": 2,
        "max_tracks": 20,
        "enable_camera_fusion": False,
        "map_to_rm_offset_x": 14.0,
        "map_to_rm_offset_y": 7.5,
    }
