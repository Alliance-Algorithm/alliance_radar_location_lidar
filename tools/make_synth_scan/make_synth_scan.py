#!/usr/bin/env python3
"""从点云源合成雷达站视角扫描，用于离线算法验证。

支持两种输入源:
  1. 场地地图 (map_zup.pcd) — 纯几何验证，无真实目标
  2. 济南真实场馆扫描 (jinan_xyz.pcd，由 tools/jinan_to_scan/jinan_to_xyz 预处理) —
     包含真实人员/机器人，用于验证动态目标检测和聚类效果

流程:
  读源点云 (工作系 Z-up 米) → 用真值位姿把点变到雷达系
  → FOV 裁剪 → z-buffer 遮挡 → 加噪声 → 写 binary xyz PCD

用法:
  python3 make_synth_scan.py <source.pcd> <out.pcd> [options]

Options:
  --eye x,y,z          雷达站位置，默认 -13,1,4.2 (验证用偏离值)
  --look-at x,y,z      注视点，默认 0,0,0.5
  --extra-yaw deg      在 look-at yaw 基础上额外偏航 (默认 12 deg，用于测试搜索鲁棒性)
  --hfov deg           水平半 FOV，默认 60 (对应 Odin1 120° 总FOV)
  --vfov deg           垂直半 FOV，默认 45 (对应 Odin1 90° 总FOV)
  --ang-res deg        z-buffer 角分辨率，默认 0.05
  --noise-sigma m      测距噪声 sigma，默认 0.02m
  --range-min m        最小量程，默认 1.0m
  --range-max m        最大量程，默认 70m

典型工作流 (真实场地测试):
  # 1. 转换济南 PCD
  tools/jinan_to_scan/build/jinan_to_xyz \\
      model/pcd_rmuc2026_jinan.pcd model/generated/jinan_xyz.pcd \\
      --roi -62,62,-62,62,-1,6

  # 2. 合成雷达站视角扫描 (使用真实部署初值，无额外偏航)
  python3 tools/make_synth_scan/make_synth_scan.py \\
      model/generated/jinan_xyz.pcd model/generated/synth_scan_jinan.pcd \\
      --eye -14,0,4 --look-at 0,0,0.5 --extra-yaw 0

  # 3. 离线检测验证
  offline-detection model/generated/synth_scan_jinan.pcd \\
      --map model/generated/map_zup.pcd
"""
import numpy as np
import struct
import sys


def read_pcd_binary_xyz(path):
    with open(path, 'rb') as f:
        data = f.read()
    header_end = data.index(b'DATA binary\n') + len(b'DATA binary\n')
    header = data[:header_end].decode('ascii', 'replace')
    npts = None
    for line in header.splitlines():
        if line.startswith('POINTS'):
            npts = int(line.split()[1])
    body = data[header_end:]
    arr = np.frombuffer(body[:npts * 12], dtype=np.float32).reshape(npts, 3)
    return arr.astype(np.float64)


def write_pcd_binary_xyz(path, pts):
    pts = pts.astype(np.float32)
    n = len(pts)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\nDATA binary\n"
    )
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(pts.tobytes())


def rot_z(deg):
    r = np.deg2rad(deg)
    c, s = np.cos(r), np.sin(r)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rot_y(deg):
    r = np.deg2rad(deg)
    c, s = np.cos(r), np.sin(r)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def parse_vec3(s):
    parts = s.split(',')
    if len(parts) != 3:
        raise ValueError(f"expected x,y,z got '{s}'")
    return np.array([float(p) for p in parts])


def main():
    import argparse
    parser = argparse.ArgumentParser(
        description='Synthesize radar station scan from source point cloud',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('source', help='Source PCD (map_zup.pcd or jinan_xyz.pcd)')
    parser.add_argument('output', help='Output scan PCD')
    parser.add_argument('--eye', default='-13,1,4.2',
                        help='Radar station position x,y,z in work frame (default: -13,1,4.2 for validation)')
    parser.add_argument('--look-at', default='0,0,0.5',
                        help='Look-at point x,y,z (default: 0,0,0.5)')
    parser.add_argument('--extra-yaw', type=float, default=12.0,
                        help='Extra yaw offset in deg on top of look-at yaw (default: 12 for search robustness test; use 0 for real deployment)')
    parser.add_argument('--hfov', type=float, default=60.0,
                        help='Horizontal half-FOV in deg (default: 60, Odin1=±60°)')
    parser.add_argument('--vfov', type=float, default=45.0,
                        help='Vertical half-FOV in deg (default: 45, Odin1=±45°)')
    parser.add_argument('--ang-res', type=float, default=0.05,
                        help='z-buffer angular resolution in deg (default: 0.05)')
    parser.add_argument('--noise-sigma', type=float, default=0.02,
                        help='Range noise sigma in meters (default: 0.02)')
    parser.add_argument('--range-min', type=float, default=1.0,
                        help='Min range in meters (default: 1.0)')
    parser.add_argument('--range-max', type=float, default=70.0,
                        help='Max range in meters (default: 70.0)')
    args = parser.parse_args()

    eye    = parse_vec3(args.eye)
    target = parse_vec3(args.look_at)

    d = target - eye
    d /= np.linalg.norm(d)
    yaw_gt   = np.rad2deg(np.arctan2(d[1], d[0]))
    pitch_gt = np.rad2deg(np.arctan2(-d[2], np.hypot(d[0], d[1])))
    yaw_gt  += args.extra_yaw

    R_gt = rot_z(yaw_gt) @ rot_y(pitch_gt)
    t_gt = eye

    print(f"[synth] Source: {args.source}")
    print(f"[synth] Eye: {eye}  look-at: {target}")
    print(f"[synth] GT pose: yaw={yaw_gt:.3f}° pitch={pitch_gt:.3f}°  extra-yaw={args.extra_yaw}°")

    pts_src = read_pcd_binary_xyz(args.source)
    print(f"[synth] Source loaded: {len(pts_src)} points")

    # 工作系 → 雷达系
    pl = (pts_src - t_gt) @ R_gt

    # FOV + 量程裁剪
    x, y, z = pl[:, 0], pl[:, 1], pl[:, 2]
    rng = np.sqrt(x * x + y * y + z * z)
    az  = np.rad2deg(np.arctan2(y, x))
    el  = np.rad2deg(np.arctan2(z, np.hypot(x, y)))
    mask = (x > 0) & (np.abs(az) <= args.hfov) & (np.abs(el) <= args.vfov) \
         & (rng >= args.range_min) & (rng <= args.range_max)
    pl, az, el, rng = pl[mask], az[mask], el[mask], rng[mask]
    print(f"[synth] After FOV filter: {len(pl)} points")

    # z-buffer 遮挡
    ANG_RES = args.ang_res
    ai   = np.floor((az + args.hfov) / ANG_RES).astype(np.int64)
    ei   = np.floor((el + args.vfov) / ANG_RES).astype(np.int64)
    cell = ai * 100000 + ei
    order       = np.argsort(rng)
    cell_sorted = cell[order]
    _, first_idx = np.unique(cell_sorted, return_index=True)
    keep = order[first_idx]
    pl   = pl[keep]
    print(f"[synth] After z-buffer occlusion: {len(pl)} points")

    # 测距噪声
    if args.noise_sigma > 0:
        pl = pl + np.random.default_rng(1).normal(0, args.noise_sigma, pl.shape)

    write_pcd_binary_xyz(args.output, pl)
    print(f"[synth] Wrote {len(pl)} points → {args.output}")

    # 打印真值 quaternion (供验证)
    R = R_gt
    w  = np.sqrt(max(0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    qx = (R[2, 1] - R[1, 2]) / (4 * w)
    qy = (R[0, 2] - R[2, 0]) / (4 * w)
    qz = (R[1, 0] - R[0, 1]) / (4 * w)
    print(f"[synth] GT t_map_lidar translation = {t_gt}")
    print(f"[synth] GT quat xyzw = [{qx:.4f}, {qy:.4f}, {qz:.4f}, {w:.4f}]")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
