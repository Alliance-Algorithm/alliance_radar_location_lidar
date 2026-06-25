#!/usr/bin/env python3
"""配准效果可视化 — 读地图 + 扫描 + pose.json，画配准前/后对比

用法:
    python3 view_registration.py <map.pcd> <scan.pcd> <pose.json>

pose.json 由 registration_tool --pose-out 生成，包含 translation + rotation_quaternion。
脚本会显示两个窗口：
  1. 配准前：地图(红) + 原始扫描(绿)
  2. 配准后：地图(红) + 变换后扫描(蓝)
"""
import os
os.environ.pop('WAYLAND_DISPLAY', None)

import sys
import json
import numpy as np
import open3d as o3d


def load_pose_json(path):
    with open(path) as f:
        pose = json.load(f)
    t = np.array(pose["translation"])
    q = np.array(pose["rotation_quaternion"])
    # quaternion [x, y, z, w] → Open3D 格式
    T = np.eye(4)
    T[:3, :3] = o3d.geometry.get_rotation_matrix_from_quaternion(q)
    T[:3, 3] = t
    return T, pose["converged"], pose.get("fitness_score", -1)


def main():
    if len(sys.argv) < 4:
        print(f"Usage: {sys.argv[0]} <map.pcd> <scan.pcd> <pose.json>", file=sys.stderr)
        sys.exit(1)

    map_path, scan_path, pose_path = sys.argv[1], sys.argv[2], sys.argv[3]

    map_pcd = o3d.io.read_point_cloud(map_path)
    scan_pcd = o3d.io.read_point_cloud(scan_path)

    print(f"Map:  {len(map_pcd.points)} points")
    print(f"Scan: {len(scan_pcd.points)} points")

    T, converged, fitness = load_pose_json(pose_path)
    print(f"Pose: converged={converged}, fitness={fitness}")
    print(f"T:\n{T}")

    # 配准前
    map_before = o3d.geometry.PointCloud(map_pcd)
    map_before.paint_uniform_color([1, 0, 0])       # red
    scan_before = o3d.geometry.PointCloud(scan_pcd)
    scan_before.paint_uniform_color([0, 1, 0])      # green

    # 配准后
    scan_after = o3d.geometry.PointCloud(scan_pcd)
    scan_after.transform(T)
    scan_after.paint_uniform_color([0, 0, 1])        # blue
    map_after = o3d.geometry.PointCloud(map_pcd)
    map_after.paint_uniform_color([1, 0, 0])         # red

    print("\n[Window 1] Before registration (red=map, green=scan)")
    o3d.visualization.draw_geometries([map_before, scan_before],
                                      window_name="Before Registration")

    print("[Window 2] After registration (red=map, blue=scan)")
    o3d.visualization.draw_geometries([map_after, scan_after],
                                      window_name="After Registration")


if __name__ == "__main__":
    main()
