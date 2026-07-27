#!/usr/bin/env python3
"""
Extract LIO odometry from ROS2 bag to CSV.

Output CSV format: timestamp,x,y,z,qx,qy,qz,qw

Usage:
  python3 extract_poses.py lio_data_bag/ --topic /fast_livo2/odom
"""

import argparse
import sys
import os
import struct
import numpy as np


def extract_from_bag(bag_path: str, topic: str) -> list:
    """Extract odometry poses from ROS2 bag."""
    try:
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    except ImportError:
        print("ERROR: rosbag2_py not available. Run inside container or install ros-jazzy-rosbag2")
        sys.exit(1)

    reader = SequentialReader()
    storage_id = "mcap" if bag_path.endswith(".mcap") else ""
    reader.open(
        StorageOptions(uri=bag_path, storage_id=storage_id),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )

    # Check available topics
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    available = list(topic_types.keys())
    if topic not in topic_types:
        print(f"Topic '{topic}' not found. Available: {available}")
        sys.exit(1)

    # Deserialize nav_msgs/Odometry
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message

    poses = []
    count = 0
    while reader.has_next():
        topic_name, data, _ = reader.read_next()
        if topic_name != topic:
            continue
        msg = deserialize_message(data, Odometry)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        poses.append((ts, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        count += 1

    print(f"Extracted {count} odometry messages from {bag_path}")
    return poses


def extract_from_pcd_dir(pcd_dir: str) -> list:
    """Extract approximate poses from PCD world points (no odometry needed).
    
    For each periodically-saved world cloud PCD, compute the centroid as proxy pose.
    Consecutive centroids form the trajectory.
    """
    import glob
    
    pcd_files = sorted(glob.glob(os.path.join(pcd_dir, "fast_livo2_map.*.pcd")))
    if not pcd_files:
        pcd_files = sorted(glob.glob(os.path.join(pcd_dir, "lio_map.*.pcd")))
    if not pcd_files:
        print(f"No PCD segment files found in {pcd_dir}")
        sys.exit(1)
    
    poses = []
    for i, f in enumerate(pcd_files):
        # Read binary PCD quickly: skip header, read points
        pts = read_pcd_xyz(f)
        if len(pts) == 0:
            continue
        centroid = np.mean(pts, axis=0)
        poses.append((float(i), float(centroid[0]), float(centroid[1]), float(centroid[2]), 
                       0.0, 0.0, 0.0, 1.0))
    
    print(f"Extracted {len(poses)} centroids from {len(pcd_files)} PCD segments")
    return poses


def read_pcd_xyz(path: str) -> np.ndarray:
    """Read x,y,z from binary PCD file. Returns Nx3 array."""
    points = []
    in_data = False
    point_step = 0
    x_off, y_off, z_off = 0, 4, 8  # defaults
    
    with open(path, "rb") as f:
        for line in f:
            line = line.decode("utf-8", errors="ignore").strip()
            if line.startswith("DATA"):
                in_data = True
                break
            if line.startswith("FIELDS"):
                fields = line.split()[1:]
                if "x" in fields: x_off = fields.index("x") * 4
                if "y" in fields: y_off = fields.index("y") * 4
                if "z" in fields: z_off = fields.index("z") * 4
                point_step = len(fields) * 4
        if not in_data:
            return np.array([])
        raw = f.read()
    
    n = len(raw) // point_step
    arr = np.frombuffer(raw, dtype=np.float32).reshape(n, point_step // 4)
    return arr[:, [x_off // 4, y_off // 4, z_off // 4]]


def main():
    parser = argparse.ArgumentParser(description="Extract LIO poses to CSV")
    parser.add_argument("input", help="ROS2 bag path or PCD directory")
    parser.add_argument("--topic", default="/fast_livo2/odom", help="Odometry topic name")
    parser.add_argument("--output", default="lio_poses.csv", help="Output CSV path")
    parser.add_argument("--pcd", action="store_true", help="Input is PCD directory (centroid-based)")
    args = parser.parse_args()

    if args.pcd or os.path.isdir(args.input) and not args.input.endswith(".mcap"):
        # Check if it looks like a bag directory
        if os.path.exists(os.path.join(args.input, "metadata.yaml")):
            poses = extract_from_bag(args.input, args.topic)
        else:
            poses = extract_from_pcd_dir(args.input)
    else:
        poses = extract_from_bag(args.input, args.topic)

    with open(args.output, "w") as f:
        for ts, x, y, z, qx, qy, qz, qw in poses:
            f.write(f"{ts},{x},{y},{z},{qx},{qy},{qz},{qw}\n")

    print(f"Saved {len(poses)} poses to {args.output}")


if __name__ == "__main__":
    main()
