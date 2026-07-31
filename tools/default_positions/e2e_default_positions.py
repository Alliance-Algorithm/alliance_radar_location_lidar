#!/usr/bin/env python3
"""E2E: verify radar_fusion fills unobserved LidarLocation slots with defaults.

Run against a live radar_fusion_node with default_positions_path set:

    ros2 run radar_fusion radar_fusion_node --ros-args \
        -p default_positions_path:=<path>/default_positions.sqlite \
        -p enemy_color:=red -p enable_camera_fusion:=false &
    python3 e2e_default_positions.py

Expected:
  PRE : all-zero (defaults suppressed before match start)
  POST: non-zero (mode-cell defaults fill unobserved slots after game_progress==4)
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from radar_interfaces.msg import GameState, LidarLocation
from sensor_msgs.msg import PointCloud2, PointField

class Probe(Node):
    def __init__(self):
        super().__init__("e2e_default_probe")
        self.game_pub = self.create_publisher(GameState, "/bridge/game_state", 10)
        self.cluster_pub = self.create_publisher(PointCloud2, "/lidar/cluster", 10)
        self.loc_sub = self.create_subscription(LidarLocation, "/lidar/location", self.on_loc, 10)
        self.loc = None

    def on_loc(self, msg):
        self.loc = msg

    def send_game_state(self, progress, remain):
        m = GameState(); m.game_progress = progress; m.stage_remain_time = remain
        self.game_pub.publish(m)

    def send_empty_cluster(self):
        p = PointCloud2()
        p.header = Header(); p.header.stamp = self.get_clock().now().to_msg()
        p.height = 1; p.width = 0
        p.fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1)]
        p.is_bigendian = False; p.point_step = 8; p.row_step = 0; p.is_dense = True
        self.cluster_pub.publish(p)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wait-start", type=float, default=1.5,
                    help="seconds between game_state(4,420) and the trigger cluster")
    args = ap.parse_args()
    rclpy.init()
    probe = Probe()
    time.sleep(2.0)  # discovery
    probe.send_empty_cluster()
    for _ in range(50): rclpy.spin_once(probe, timeout_sec=0.05)
    pre = probe.loc
    probe.send_game_state(4, 420)
    time.sleep(args.wait_start)
    probe.send_empty_cluster()
    for _ in range(50): rclpy.spin_once(probe, timeout_sec=0.05)
    post = probe.loc
    def snap(m):
        if m is None: return None
        return (m.opponent_hero_x, m.opponent_hero_y, m.ally_hero_x, m.ally_hero_y,
                m.opponent_sentry_x, m.opponent_sentry_y)
    print("PRE :", snap(pre))
    print("POST:", snap(post))
    if post is None:
        print("RESULT: FAIL - no /lidar/location received"); return 1
    pre_zero = pre is None or all(v == 0 for v in snap(pre))
    post_nonzero = any(v > 0 for v in snap(post))
    print(f"pre all-zero: {pre_zero}  post any-nonzero: {post_nonzero}")
    ok = post_nonzero and pre_zero
    print("RESULT:", "PASS" if ok else "FAIL")
    rclpy.shutdown()
    return 0 if ok else 1

if __name__ == "__main__":
    raise SystemExit(main())
