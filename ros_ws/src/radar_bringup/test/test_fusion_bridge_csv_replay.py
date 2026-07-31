import csv
import json
import os
import socket
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
import yaml
import zmq
from geometry_msgs.msg import Point
from radar_interfaces.msg import CameraDetection, CameraDetectionArray, LidarLocation
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


LIDAR_LOCATION_FIELDS = (
    "cmd_id",
    "opponent_hero_x",
    "opponent_hero_y",
    "opponent_engineer_x",
    "opponent_engineer_y",
    "opponent_infantry_3_x",
    "opponent_infantry_3_y",
    "opponent_infantry_4_x",
    "opponent_infantry_4_y",
    "opponent_aerial_x",
    "opponent_aerial_y",
    "opponent_sentry_x",
    "opponent_sentry_y",
    "ally_hero_x",
    "ally_hero_y",
    "ally_engineer_x",
    "ally_engineer_y",
    "ally_infantry_3_x",
    "ally_infantry_3_y",
    "ally_infantry_4_x",
    "ally_infantry_4_y",
    "ally_aerial_x",
    "ally_aerial_y",
    "ally_sentry_x",
    "ally_sentry_y",
)


def free_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def read_replay_rows(csv_path: Path, image_dir: Path):
    selected = {}
    with csv_path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            if row["hit_ok"] != "1" or row["class_name"] not in {"hero_b", "eng_r", "inf3_b"}:
                continue
            selected.setdefault(row["class_name"], row)

    missing = [
        row["path"]
        for row in selected.values()
        if not any(
            (image_dir / candidate).is_file()
            for candidate in (row["path"], row["path"].replace(".jpg", "_det1280.jpg"))
        )
    ]
    assert not missing, f"CSV replay images missing: {missing}"
    assert set(selected) == {"hero_b", "eng_r", "inf3_b"}, selected
    return selected


def camera_detection(name: str, row: dict) -> CameraDetection:
    detection = CameraDetection()
    detection.position.x = float(row["map_x"])
    detection.position.y = float(row["map_y"])
    detection.position.z = 0.0
    detection.confidence = float(row["conf"])
    detection.team = CameraDetection.TEAM_BLUE if name.endswith("_b") else CameraDetection.TEAM_RED
    detection.semantic_class = {
        "hero_b": CameraDetection.CLASS_HERO,
        "eng_r": CameraDetection.CLASS_ENGINEER,
        "inf3_b": CameraDetection.CLASS_INFANTRY_3,
    }[name]
    return detection


def lidar_location_fields(msg: LidarLocation) -> dict:
    return {field: int(getattr(msg, field)) for field in LIDAR_LOCATION_FIELDS}


def bridge_fields(payload: dict) -> dict:
    return {key: int(value) for key, value in payload.items() if key != "cmd_id"}


class ReplayNode(Node):
    def __init__(self):
        super().__init__("fusion_bridge_csv_replay")
        self.camera_pub = self.create_publisher(CameraDetectionArray, "/camera/detection", 10)
        self.cluster_pub = self.create_publisher(PointCloud2, "/lidar/cluster", 10)
        self.location = None
        self.locations = []
        self.location_sub = self.create_subscription(
            LidarLocation, "/lidar/location", self.on_location, 10
        )

    def on_location(self, msg):
        self.location = msg
        self.locations.append(msg)

    def publish_frame(self, rows: dict, stamp):
        camera = CameraDetectionArray()
        camera.header.stamp = stamp.to_msg()
        camera.header.frame_id = "map"
        for name, row in rows.items():
            camera.detections.append(camera_detection(name, row))
        self.camera_pub.publish(camera)

        points = [(float(row["map_x"]), float(row["map_y"]), 0.0) for row in rows.values()]
        cloud = point_cloud2.create_cloud_xyz32(camera.header, points)
        self.cluster_pub.publish(cloud)


def test_csv_replay_fusion_and_bridge_transport():
    csv_path = Path(os.environ["RADAR_CAMERA_RAY_CSV"])
    image_dir = Path(os.environ["RADAR_REPLAY_IMAGE_DIR"])
    rows = read_replay_rows(csv_path, image_dir)
    expected_cm = {
        name: (float(row["rm_x_cm"]), float(row["rm_y_cm"])) for name, row in rows.items()
    }

    pub_port = free_port()
    sub_port = free_port()
    with tempfile.TemporaryDirectory(prefix="radar_replay_") as tmp:
        tmp_path = Path(tmp)
        fusion_yaml = tmp_path / "fusion.yaml"
        fusion_yaml.write_text(
            yaml.safe_dump(
                {
                    "radar_fusion_node": {
                        "ros__parameters": {
                            "gate_distance": 2.0,
                            "track_timeout_sec": 2.0,
                            "min_hits_to_confirm": 1,
                            "max_misses_before_delete": 2,
                            "max_tracks": 20,
                            "enable_camera_fusion": True,
                            "camera_timeout_sec": 2.0,
                            "map_to_rm_offset_x": 14.0,
                            "map_to_rm_offset_y": 7.5,
                        }
                    }
                }
            )
        )
        bridge_yaml = tmp_path / "bridge.yaml"
        bridge_yaml.write_text(
            yaml.safe_dump(
                {
                    "radar_bridge_node": {
                        "ros__parameters": {
                            "zmq_pub_address": f"tcp://127.0.0.1:{pub_port}",
                            "zmq_sub_addresses": [f"tcp://127.0.0.1:{sub_port}"],
                            "shm_name": "/unused_replay_shm",
                            "video_pub_address": "tcp://127.0.0.1:59999",
                            "image_topic": "/unused",
                            "video_width": 5472,
                            "video_height": 3648,
                            "enable_video_stream": False,
                        }
                    }
                }
            )
        )

        context = zmq.Context()
        subscriber = context.socket(zmq.SUB)
        subscriber.connect(f"tcp://127.0.0.1:{pub_port}")
        subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
        fusion = subprocess.Popen(["ros2", "run", "radar_fusion", "radar_fusion_node", "--ros-args", "--params-file", str(fusion_yaml)])
        bridge = subprocess.Popen(["ros2", "run", "radar_bridge", "radar_bridge_node", "--ros-args", "--params-file", str(bridge_yaml)])
        try:
            rclpy.init()
            node = ReplayNode()
            deadline = time.monotonic() + 20.0
            while time.monotonic() < deadline and (node.camera_pub.get_subscription_count() < 1 or node.cluster_pub.get_subscription_count() < 1):
                rclpy.spin_once(node, timeout_sec=0.1)
            assert node.camera_pub.get_subscription_count() >= 1
            assert node.cluster_pub.get_subscription_count() >= 1

            for _ in range(5):
                node.publish_frame(rows, node.get_clock().now())
                rclpy.spin_once(node, timeout_sec=0.15)

            deadline = time.monotonic() + 10.0
            payload = None
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if node.location is not None:
                    try:
                        payload = json.loads(subscriber.recv_string(flags=zmq.NOBLOCK))
                        break
                    except zmq.Again:
                        pass
            assert node.location is not None, "fusion did not publish /lidar/location"
            assert payload is not None, "bridge did not publish ZMQ location JSON"
            assert payload["cmd_id"] == 0x2001

            payload_fields = bridge_fields(payload)
            matching_ros_fields = next(
                (
                    lidar_location_fields(location)
                    for location in node.locations
                    if payload_fields
                    == {
                        key: value
                        for key, value in lidar_location_fields(location).items()
                        if key != "cmd_id"
                    }
                ),
                None,
            )
            assert matching_ros_fields is not None, "bridge JSON did not match any ROS location message"
            ros_fields = matching_ros_fields

            observed = {
                "opponent_hero": (ros_fields["opponent_hero_x"], ros_fields["opponent_hero_y"]),
                "opponent_infantry_3": (ros_fields["opponent_infantry_3_x"], ros_fields["opponent_infantry_3_y"]),
                "ally_engineer": (ros_fields["ally_engineer_x"], ros_fields["ally_engineer_y"]),
            }
            expected_fields = {
                "opponent_hero": tuple(int(value) for value in expected_cm["hero_b"]),
                "opponent_infantry_3": tuple(int(value) for value in expected_cm["inf3_b"]),
                "ally_engineer": tuple(int(value) for value in expected_cm["eng_r"]),
            }
            assert observed == expected_fields, (
                f"semantic centimeter mismatch: expected={expected_fields} observed={observed}"
            )
            print(f"CSV official cm oracle: {expected_cm}")
            print(f"Fusion output fields: {observed}")
            print("Bridge JSON matches the ROS LidarLocation fields and semantic centimeter slots.")
        finally:
            node.destroy_node()
            rclpy.shutdown()
            fusion.terminate()
            bridge.terminate()
            fusion.wait(timeout=5)
            bridge.wait(timeout=5)
            subscriber.close()
            context.term()
