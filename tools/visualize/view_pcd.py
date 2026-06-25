#!/usr/bin/env python3
"""查看 PCD 文件 — 支持1~2个文件叠加显示

用法:
    python3 view_pcd.py <map.pcd>
    python3 view_pcd.py <map.pcd> <scan.pcd>          # 双窗口对比
    python3 view_pcd.py <map.pcd> <scan.pcd> --aligned # 叠加显示
"""
import os
os.environ.pop('WAYLAND_DISPLAY', None)

import sys
import argparse
import open3d as o3d


def main():
    parser = argparse.ArgumentParser(description="PCD viewer using Open3D")
    parser.add_argument("pcd", nargs="+", help="1 or 2 PCD files")
    parser.add_argument("--aligned", action="store_true",
                        help="overlay both clouds in same window (default: side-by-side)")
    args = parser.parse_args()

    clouds = []
    colors = [[1, 0, 0], [0, 1, 0]]  # red, green

    for path in args.pcd:
        pcd = o3d.io.read_point_cloud(path)
        n = len(pcd.points)
        if n == 0:
            print(f"ERROR: {path} has 0 points or failed to load", file=sys.stderr)
            sys.exit(1)
        pcd.paint_uniform_color(colors[len(clouds) % 2])
        clouds.append((path, pcd, n))
        print(f"Loaded: {path}  ({n} points)")

    if args.aligned or len(clouds) == 1:
        geometries = [pcd for _, pcd, _ in clouds]
        o3d.visualization.draw_geometries(geometries, window_name="PCD Viewer")
    else:
        # side-by-side
        for path, pcd, n in clouds:
            o3d.visualization.draw_geometries([pcd], window_name=f"{path} ({n} pts)")


if __name__ == "__main__":
    main()
