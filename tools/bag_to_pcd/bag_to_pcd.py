#!/usr/bin/env python3
"""ros2 bag → PCD 导出工具

从录制的 ros2 bag 中导出 PointCloud2 topic 到 PCD 文件。
主要用于把 Odin1 SLAM 模式的 /odin1/cloud_slam 导出为场地地图 PCD。

用法:
    # 导出单个 PCD（所有帧合并）
    python3 bag_to_pcd.py <bag_dir> [--topic /odin1/cloud_slam] [--output map.pcd]

    # 导出每帧为单独的 PCD文件（用于离线配准测试）
    python3 bag_to_pcd.py <bag_dir> --per-frame --output-dir frames/

需要在 ROS2 环境中运行（source /opt/ros/jazzy/setup.bash）。
不依赖 open3d，用纯 numpy + ASCII PCD 写入。
"""
import sys
import argparse
from pathlib import Path
import numpy as np

def main():
    parser = argparse.ArgumentParser(description="Export PointCloud2 from ros2 bag to PCD")
    parser.add_argument("bag", help="ros2 bag directory")
    parser.add_argument("--topic", default="/odin1/cloud_slam", help="PointCloud2 topic name")
    parser.add_argument("--output", default="map.pcd", help="output PCD path (merged mode)")
    parser.add_argument("--per-frame", action="store_true", help="save each frame as separate PCD")
    parser.add_argument("--output-dir", default="frames", help="output dir for per-frame mode")
    args = parser.parse_args()

    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ImportError:
        print("ERROR: pip install rosbags", file=sys.stderr)
        sys.exit(1)

    typestore = get_typestore(Stores.ROS2_JAZZY)
    PointCloud2 = typestore.types['sensor_msgs/msg/PointCloud2']

    all_points = []
    frame_count = 0

    with AnyReader([Path(args.bag)]) as reader:
        connections = [c for c in reader.connections if c.topic == args.topic]
        if not connections:
            print(f"ERROR: topic {args.topic} not found in bag", file=sys.stderr)
            print(f"Available topics: {[c.topic for c in reader.connections]}", file=sys.stderr)
            sys.exit(1)

        print(f"Reading bag: {args.bag}")
        print(f"Topic: {args.topic}")

        for conn, timestamp, rawdata in reader.messages(connections=connections):
            msg = typestore.deserialize_cdr(rawdata, conn.msgtype)

            # 解析 PointCloud2 → numpy points
            point_step = msg.point_step
            data = np.frombuffer(msg.data, dtype=np.uint8)

            # 假设 XYZ 是前3个 float32 字段
            fields = {}
            for f in msg.fields:
                fields[f.name] = f.offset

            if 'x' not in fields or 'y' not in fields or 'z' not in fields:
                continue

            n_points = len(data) // point_step
            if n_points == 0:
                continue

            cloud_data = data[:n_points * point_step].reshape(n_points, point_step)

            x = np.frombuffer(cloud_data[:, fields['x']].tobytes(), dtype=np.float32)
            y = np.frombuffer(cloud_data[:, fields['y']].tobytes(), dtype=np.float32)
            z = np.frombuffer(cloud_data[:, fields['z']].tobytes(), dtype=np.float32)

            # 过滤 NaN/Inf 和零点
            valid = (np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
                     & ((x**2 + y**2 + z**2) > 1e-12))
            pts = np.stack([x[valid], y[valid], z[valid]], axis=-1)

            if len(pts) == 0:
                continue

            all_points.append(pts)
            frame_count += 1

            if args.per_frame:
                import os
                os.makedirs(args.output_dir, exist_ok=True)
                out_path = f"{args.output_dir}/frame_{frame_count:06d}.pcd"
                _write_pcd_ascii(out_path, pts)
                if frame_count % 50 == 0:
                    print(f"  Frame {frame_count}: {len(pts)} points → {out_path}")

    if frame_count == 0:
        print("ERROR: no valid frames found", file=sys.stderr)
        sys.exit(1)

    print(f"\nTotal frames: {frame_count}")

    if not args.per_frame:
        merged = np.concatenate(all_points, axis=0)
        print(f"Merged points: {len(merged)}")
        _write_pcd_ascii(args.output, merged)
        print(f"Output: {args.output}")


def _write_pcd_ascii(path, points):
    """简易 ASCII PCD 写入（不依赖 open3d）"""
    with open(path, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        for p in points:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")


if __name__ == "__main__":
    main()
