#!/usr/bin/env python3
"""Record radar_fusion camera-track output to CSV for continuity analysis.

Subscribes /lidar/location (LidarLocation), /radar_camera/robot_pose
(CameraDetectionPose), /fusion/tracks (MarkerArray) and writes one CSV
row per /lidar/location message.

CSV columns:
  stamp_ns, hero_x, hero_y, hero_conf, eng_x, eng_y, eng_conf,
  inf3_x, inf3_y, inf3_conf, inf4_x, inf4_y, inf4_conf,
  sentry_x, sentry_y, sentry_conf, drone_x, drone_y, drone_conf,
  n_tracks
"""
import argparse
import csv
import signal
import sys
import time

import rclpy
from rclpy.node import Node

from radar_interfaces.msg import CameraDetectionPose, LidarLocation
from visualization_msgs.msg import MarkerArray


class Recorder(Node):
    def __init__(self, out_path: str, duration: float):
        super().__init__("location_recorder")
        self.out_path = out_path
        self.duration = duration
        self.start_ns = time.time_ns()
        self.n_rows = 0
        self.f = open(out_path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow([
            "stamp_ns", "hero_x", "hero_y", "hero_conf",
            "eng_x", "eng_y", "eng_conf",
            "inf3_x", "inf3_y", "inf3_conf",
            "inf4_x", "inf4_y", "inf4_conf",
            "sentry_x", "sentry_y", "sentry_conf",
            "drone_x", "drone_y", "drone_conf",
            "n_tracks",
        ])
        self.sub_loc = self.create_subscription(
            LidarLocation, "/lidar/location", self.on_location, 10)
        self.sub_pose = self.create_subscription(
            CameraDetectionPose, "/radar_camera/robot_pose", self.on_pose, 10)
        self.sub_tracks = self.create_subscription(
            MarkerArray, "/fusion/tracks", self.on_tracks, 10)
        self.get_logger().info(f"recording to {out_path}")

    def on_pose(self, msg: CameraDetectionPose):
        self.latest_pose = msg

    def on_tracks(self, msg: MarkerArray):
        self.latest_tracks = msg

    def on_location(self, msg: LidarLocation):
        p = getattr(self, "latest_pose", None)
        t = getattr(self, "latest_tracks", None)
        row = [
            self.get_clock().now().nanoseconds,
            msg.opponent_hero_x, msg.opponent_hero_y,
            p.hero_confidence if p else 0.0,
            msg.opponent_engineer_x, msg.opponent_engineer_y,
            p.engine_confidence if p else 0.0,
            msg.opponent_infantry_3_x, msg.opponent_infantry_3_y,
            p.infantry_3_confidence if p else 0.0,
            msg.opponent_infantry_4_x, msg.opponent_infantry_4_y,
            p.infantry_4_confidence if p else 0.0,
            msg.opponent_sentry_x, msg.opponent_sentry_y,
            p.sentry_confidence if p else 0.0,
            msg.opponent_aerial_x, msg.opponent_aerial_y,
            p.drone_confidence if p else 0.0,
            len(t.markers) if t else 0,
        ]
        self.w.writerow(row)
        self.n_rows += 1
        if self.n_rows % 100 == 0:
            self.get_logger().info(f"{self.n_rows} rows")
        elapsed = (time.time_ns() - self.start_ns) / 1e9
        if self.duration > 0 and elapsed >= self.duration:
            self.shutdown()

    def shutdown(self):
        if self.f.closed:
            return
        self.f.close()
        self.get_logger().info(f"done: {self.n_rows} rows -> {self.out_path}")
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=60.0)
    args = ap.parse_args()

    rclpy.init()
    node = Recorder(args.out, args.duration)
    signal.signal(signal.SIGINT, lambda *_: node.shutdown())

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()


if __name__ == "__main__":
    main()
