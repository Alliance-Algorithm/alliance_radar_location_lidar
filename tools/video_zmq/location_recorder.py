#!/usr/bin/env python3
"""Record match-time logs for offline review.

Subscribes:
  /lidar/location      (LidarLocation)          -> locations.csv (coordinates)
  /localization/pose   (PoseWithCovarianceStamped) -> localization.csv (GICP pose)
  /localization/status (DiagnosticStatus)       -> status.csv (fusion/定位状态)
  /radar_camera/robot_pose + /fusion/tracks     -> confidence columns in locations.csv

Output layout (one dir per match):
  <out_dir>/<YYYYmmdd_HHMMSS>/
      locations.csv      one row per /lidar/location message
      localization.csv   one row per /localization/pose message
      status.csv         one row per /localization/status message
      match.log          node output

Usage:
  python3 location_recorder.py [--out /workspace/model/logs] [--duration 0]
  --duration 0 = unlimited (match end stops via SIGINT/Ctrl-C)
"""
import argparse
import csv
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from radar_interfaces.msg import CameraDetectionPose, LidarLocation
from visualization_msgs.msg import MarkerArray


class MatchRecorder(Node):
    def __init__(self, out_dir: str, duration: float):
        super().__init__("match_recorder")
        self.duration = duration
        self.start_ns = time.time_ns()
        self.n_rows = {"locations": 0, "localization": 0, "status": 0}

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = Path(out_dir) / stamp
        self.session_dir.mkdir(parents=True, exist_ok=True)

        self.f_locations = open(self.session_dir / "locations.csv", "w", newline="")
        self.f_localization = open(self.session_dir / "localization.csv", "w", newline="")
        self.f_status = open(self.session_dir / "status.csv", "w", newline="")
        self.w_loc = csv.writer(self.f_locations)
        self.w_loc.writerow([
            "stamp_ns", "hero_x", "hero_y", "hero_conf",
            "eng_x", "eng_y", "eng_conf",
            "inf3_x", "inf3_y", "inf3_conf",
            "inf4_x", "inf4_y", "inf4_conf",
            "sentry_x", "sentry_y", "sentry_conf",
            "drone_x", "drone_y", "drone_conf",
            "n_tracks",
        ])
        self.w_local = csv.writer(self.f_localization)
        self.w_local.writerow([
            "stamp_ns", "x", "y", "z", "qx", "qy", "qz", "qw", "cov_diag",
        ])
        self.w_status = csv.writer(self.f_status)
        self.w_status.writerow(["stamp_ns", "level", "name", "message", "mode", "values"])

        self.sub_loc = self.create_subscription(
            LidarLocation, "/lidar/location", self.on_location, 10)
        self.sub_pose = self.create_subscription(
            PoseWithCovarianceStamped, "/localization/pose", self.on_pose, 10)
        self.sub_status = self.create_subscription(
            DiagnosticStatus, "/localization/status", self.on_status, 10)
        self.sub_cam = self.create_subscription(
            CameraDetectionPose, "/radar_camera/robot_pose", self.on_camera, 10)
        self.sub_tracks = self.create_subscription(
            MarkerArray, "/fusion/tracks", self.on_tracks, 10)

        self.get_logger().info(
            f"match recorder -> {self.session_dir} (duration={duration}s)")

    def on_camera(self, msg: CameraDetectionPose):
        self.latest_pose = msg

    def on_tracks(self, msg: MarkerArray):
        self.latest_tracks = msg

    def on_pose(self, msg: PoseWithCovarianceStamped):
        c = msg.pose.covariance
        diag = ",".join(f"{c[i]:.4f}" for i in (0, 7, 14, 21, 28, 35))
        self.w_local.writerow([
            self.get_clock().now().nanoseconds,
            f"{msg.pose.pose.position.x:.4f}", f"{msg.pose.pose.position.y:.4f}",
            f"{msg.pose.pose.position.z:.4f}",
            f"{msg.pose.pose.orientation.x:.4f}", f"{msg.pose.pose.orientation.y:.4f}",
            f"{msg.pose.pose.orientation.z:.4f}", f"{msg.pose.pose.orientation.w:.4f}",
            diag,
        ])
        self.n_rows["localization"] += 1

    def on_status(self, msg: DiagnosticStatus):
        kv = ";".join(f"{v.key}={v.value}" for v in msg.values)
        self.w_status.writerow([
            self.get_clock().now().nanoseconds,
            msg.level, msg.name, msg.message, kv,
        ])
        self.n_rows["status"] += 1

    def on_location(self, msg: LidarLocation):
        # 与 bridge 转发一致：0x0305 官方频率上限 5Hz，日志限频到 5Hz
        now_ns = time.time_ns()
        if now_ns - getattr(self, "last_location_ns", 0) < 200_000_000:
            return
        self.last_location_ns = now_ns
        p = getattr(self, "latest_pose", None)
        t = getattr(self, "latest_tracks", None)
        self.w_loc.writerow([
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
        ])
        self.n_rows["locations"] += 1
        if self.n_rows["locations"] % 100 == 0:
            self.get_logger().info(
                f"locations={self.n_rows['locations']} "
                f"localization={self.n_rows['localization']} "
                f"status={self.n_rows['status']}")
        elapsed = (time.time_ns() - self.start_ns) / 1e9
        if self.duration > 0 and elapsed >= self.duration:
            self.shutdown()

    def shutdown(self):
        if self.f_locations.closed:
            return
        self.f_locations.close()
        self.f_localization.close()
        self.f_status.close()
        self.get_logger().info(
            f"done: locations={self.n_rows['locations']} "
            f"localization={self.n_rows['localization']} "
            f"status={self.n_rows['status']} -> {self.session_dir}")
        # SIGINT 时 rclpy 已因信号关闭 context；这里只幂等地触发一次，重复调用忽略。
        try:
            rclpy.shutdown()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/workspace/model/logs",
                    help="root dir; per-match subdir auto-created")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="0 = unlimited")
    args = ap.parse_args()

    rclpy.init()
    node = MatchRecorder(args.out, args.duration)
    signal.signal(signal.SIGINT, lambda *_: node.shutdown())

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()


if __name__ == "__main__":
    main()
