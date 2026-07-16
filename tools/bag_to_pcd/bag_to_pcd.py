#!/usr/bin/env python3
"""ros2 bag → PCD 导出工具

从录制的 ros2 bag 中导出 PointCloud2 topic 到 PCD 文件。
主要用于把 Odin1 SLAM 模式的 /odin1/cloud_slam 导出为场地地图 PCD。

用法:
    # 导出单个 PCD（所有帧合并 + voxel 降采样 + binary 写入，默认）
    python3 bag_to_pcd.py <bag_dir> [--topic /odin1/cloud_slam] [--output map.pcd]

    # 带 RGB 颜色（cloud_slam 自带 packed float32 rgb 字段）
    python3 bag_to_pcd.py <bag_dir> --rgb --output map.pcd

    # 调整降采样体素大小 / 关闭降采样
    python3 bag_to_pcd.py <bag_dir> --voxel-size 0.05
    python3 bag_to_pcd.py <bag_dir> --no-downsample

    # 输出 ASCII 而非 binary
    python3 bag_to_pcd.py <bag_dir> --ascii

    # 导出每帧为单独的 PCD文件（用于离线配准测试）
    python3 bag_to_pcd.py <bag_dir> --per-frame --output-dir frames/

需要在 ROS2 环境中运行（source /opt/ros/jazzy/setup.bash）。
不依赖 open3d/PCL，纯 numpy 实现 PointCloud2 解析、voxel 降采样、PCD 写入。
"""
import sys
import time
import struct
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
    parser.add_argument("--rgb", action="store_true", help="extract packed float32 rgb field")
    parser.add_argument("--voxel-size", type=float, default=0.05,
                         help="voxel downsample leaf size in meters (default 0.05)")
    parser.add_argument("--no-downsample", action="store_true", help="disable voxel downsampling")
    parser.add_argument("--ascii", action="store_true", help="write ASCII PCD instead of binary")
    args = parser.parse_args()

    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ImportError:
        print("ERROR: pip install rosbags", file=sys.stderr)
        sys.exit(1)

    typestore = get_typestore(Stores.ROS2_JAZZY)

    all_xyz = []
    all_rgb = [] if args.rgb else None
    frame_count = 0
    t_start = time.time()

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

            xyz, rgb = _parse_cloud_slam(msg, want_rgb=args.rgb)
            if xyz is None or len(xyz) == 0:
                continue

            all_xyz.append(xyz)
            if args.rgb:
                all_rgb.append(rgb)
            frame_count += 1

            if args.per_frame:
                out_dir = Path(args.output_dir)
                out_dir.mkdir(parents=True, exist_ok=True)
                out_path = out_dir / f"frame_{frame_count:06d}.pcd"
                write_pcd(out_path, xyz, rgb, ascii_mode=args.ascii)
                if frame_count % 50 == 0:
                    print(f"  Frame {frame_count}: {len(xyz)} points -> {out_path}")

    if frame_count == 0:
        print("ERROR: no valid frames found", file=sys.stderr)
        sys.exit(1)

    print(f"\nTotal frames: {frame_count}")

    if args.per_frame:
        print(f"Elapsed: {time.time() - t_start:.1f}s")
        return

    merged_xyz = np.concatenate(all_xyz, axis=0)
    merged_rgb = np.concatenate(all_rgb, axis=0) if args.rgb else None
    print(f"Accumulated points: {len(merged_xyz)}")

    if not args.no_downsample:
        merged_xyz, merged_rgb = voxel_downsample(merged_xyz, merged_rgb, args.voxel_size)
        print(f"After voxel downsample ({args.voxel_size} m): {len(merged_xyz)} points")

    bbox_min = merged_xyz.min(axis=0)
    bbox_max = merged_xyz.max(axis=0)
    print(f"Bounding box: min={bbox_min}, max={bbox_max}")

    write_pcd(args.output, merged_xyz, merged_rgb, ascii_mode=args.ascii)
    print(f"Output: {args.output}")
    print(f"Elapsed: {time.time() - t_start:.1f}s")


def _parse_cloud_slam(msg, want_rgb):
    """解析 PointCloud2 消息为 (xyz, rgb) numpy 数组。

    按 point_step 步长逐字段提取，而非只读每个字段的首字节，
    修正了此前实现里字段偏移量未按完整 float32 宽度取值的问题。
    """
    point_step = msg.point_step
    data = np.frombuffer(msg.data, dtype=np.uint8)

    fields = {f.name: f.offset for f in msg.fields}
    if 'x' not in fields or 'y' not in fields or 'z' not in fields:
        return None, None

    n_points = len(data) // point_step
    if n_points == 0:
        return None, None

    cloud = data[:n_points * point_step].reshape(n_points, point_step)

    def field_f32(name):
        off = fields[name]
        raw = cloud[:, off:off + 4].tobytes()
        return np.frombuffer(raw, dtype=np.float32)

    x = field_f32('x')
    y = field_f32('y')
    z = field_f32('z')

    valid = (np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
             & ((x ** 2 + y ** 2 + z ** 2) > 1e-12))

    xyz = np.stack([x[valid], y[valid], z[valid]], axis=-1)

    rgb = None
    if want_rgb and 'rgb' in fields:
        packed = field_f32('rgb')[valid]
        rgb = _unpack_pcl_rgb(packed)
    elif want_rgb:
        rgb = np.zeros((len(xyz), 3), dtype=np.uint8)

    return xyz, rgb


def _unpack_pcl_rgb(packed_f32):
    """PCL 约定：rgb 以 float32 形式承载打包后的 uint32 (0x00RRGGBB)。"""
    as_uint32 = packed_f32.view(np.uint32)
    r = ((as_uint32 >> 16) & 0xFF).astype(np.uint8)
    g = ((as_uint32 >> 8) & 0xFF).astype(np.uint8)
    b = (as_uint32 & 0xFF).astype(np.uint8)
    return np.stack([r, g, b], axis=-1)


def voxel_downsample(xyz, rgb, leaf_size):
    """哈希网格降采样：每个体素保留一个点（该体素内首个点），纯 numpy 实现。"""
    if leaf_size <= 0:
        return xyz, rgb

    voxel_idx = np.floor(xyz / leaf_size).astype(np.int64)
    # 组合 3 个整数坐标为单个 hash key，用于 np.unique 去重
    keys = (voxel_idx[:, 0].astype(np.int64) * 73856093
            ^ voxel_idx[:, 1].astype(np.int64) * 19349663
            ^ voxel_idx[:, 2].astype(np.int64) * 83492791)

    _, first_idx = np.unique(keys, return_index=True)
    first_idx = np.sort(first_idx)

    xyz_ds = xyz[first_idx]
    rgb_ds = rgb[first_idx] if rgb is not None else None
    return xyz_ds, rgb_ds


def write_pcd(path, xyz, rgb=None, ascii_mode=False):
    """写入 PCD 文件，支持 xyz / xyzrgb，ascii 或 binary 格式。"""
    n = len(xyz)
    has_rgb = rgb is not None

    if has_rgb:
        fields_line = "FIELDS x y z rgb"
        size_line = "SIZE 4 4 4 4"
        type_line = "TYPE F F F U"
        count_line = "COUNT 1 1 1 1"
    else:
        fields_line = "FIELDS x y z"
        size_line = "SIZE 4 4 4"
        type_line = "TYPE F F F"
        count_line = "COUNT 1 1 1"

    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        f"{fields_line}\n"
        f"{size_line}\n"
        f"{type_line}\n"
        f"{count_line}\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
    )

    path = Path(path)

    if ascii_mode:
        with open(path, 'w') as f:
            f.write(header)
            f.write("DATA ascii\n")
            if has_rgb:
                packed = _pack_pcl_rgb(rgb)
                for i in range(n):
                    f.write(f"{xyz[i, 0]:.6f} {xyz[i, 1]:.6f} {xyz[i, 2]:.6f} {packed[i]}\n")
            else:
                for i in range(n):
                    f.write(f"{xyz[i, 0]:.6f} {xyz[i, 1]:.6f} {xyz[i, 2]:.6f}\n")
        return

    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(b"DATA binary\n")
        if has_rgb:
            packed = _pack_pcl_rgb(rgb).astype(np.uint32)
            record = np.empty(n, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
            record['x'] = xyz[:, 0]
            record['y'] = xyz[:, 1]
            record['z'] = xyz[:, 2]
            record['rgb'] = packed
        else:
            record = np.empty(n, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
            record['x'] = xyz[:, 0]
            record['y'] = xyz[:, 1]
            record['z'] = xyz[:, 2]
        f.write(record.tobytes())


def _pack_pcl_rgb(rgb_uint8):
    """将 (N,3) uint8 RGB 打包为 PCL 约定的 uint32 (0x00RRGGBB)。"""
    r = rgb_uint8[:, 0].astype(np.uint32)
    g = rgb_uint8[:, 1].astype(np.uint32)
    b = rgb_uint8[:, 2].astype(np.uint32)
    return (r << 16) | (g << 8) | b


if __name__ == "__main__":
    main()
