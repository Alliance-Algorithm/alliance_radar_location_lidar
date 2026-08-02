#!/usr/bin/env python3
"""生成带场上机器人点云的 odin1 视角合成扫描，用于 watchdog 离线测试。

流程:
  1. 读场地地图 (带墙) jinan_field_map_reg_walls.pcd
  2. 在场地内注入 N 个"机器人"点云（盒状聚类，模拟机器人 0.8×0.8×1.2m）
  3. 用 odin1 参数 (hfov=60, vfov=45, 120°×90° 扇形) 合成雷达站视角扫描
     （含 z-buffer 遮挡 + 测距噪声）

用法:
  python3 make_field_scan.py <map.pcd> <out.pcd> [--eye x,y,z] [--look-at x,y,z]
      [--robots N] [--noise-sigma m]

默认: eye=-14,0,4 (红方雷达站) look-at=0,0,0.5 robots=6
"""
import argparse
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


def make_robot(w=0.8, h=0.8, z0=0.0, z1=1.2, density=0.03):
    """盒状机器人点云（工作系，中心在原点）"""
    xs = np.arange(-w / 2, w / 2 + 1e-9, density)
    ys = np.arange(-h / 2, h / 2 + 1e-9, density)
    zs = np.arange(z0, z1 + 1e-9, density)
    pts = []
    for z in zs:
        for y in ys:
            for x in xs:
                pts.append([x, y, z])
    return np.array(pts, dtype=np.float64)


def main():
    parser = argparse.ArgumentParser(description='Field map + robots -> odin1 synth scan')
    parser.add_argument('source', help='Source map PCD (with walls)')
    parser.add_argument('output', help='Output scan PCD')
    parser.add_argument('--eye', default='-14,0,4', help='Radar station position (default -14,0,4)')
    parser.add_argument('--look-at', default='0,0,0.5', help='Look-at point (default 0,0,0.5)')
    parser.add_argument('--robots', type=int, default=6, help='Number of robots (default 6)')
    parser.add_argument('--hfov', type=float, default=60.0, help='Horizontal half-FOV (default 60)')
    parser.add_argument('--vfov', type=float, default=45.0, help='Vertical half-FOV (default 45)')
    parser.add_argument('--ang-res', type=float, default=0.05, help='z-buffer angular res (default 0.05)')
    parser.add_argument('--noise-sigma', type=float, default=0.02, help='Range noise sigma (default 0.02)')
    parser.add_argument('--range-min', type=float, default=1.0)
    parser.add_argument('--range-max', type=float, default=70.0)
    args = parser.parse_args()

    rng = np.random.default_rng(42)
    eye = parse_vec3(args.eye)
    target = parse_vec3(args.look_at)

    d = target - eye
    norm = np.linalg.norm(d)
    d /= norm
    yaw_gt = np.rad2deg(np.arctan2(d[1], d[0]))
    pitch_gt = np.rad2deg(np.arctan2(-d[2], np.hypot(d[0], d[1])))
    R_gt = rot_z(yaw_gt) @ rot_y(pitch_gt)
    t_gt = eye

    print(f"[synth] map: {args.source}  eye: {eye}  look-at: {target}")
    print(f"[synth] GT yaw={yaw_gt:.3f}° pitch={pitch_gt:.3f}°  robots={args.robots}")

    pts_src = read_pcd_binary_xyz(args.source)
    print(f"[synth] map loaded: {len(pts_src)} points")

    # 注入机器人点云（场地内，雷达站对面半场）
    robot_pts = []
    for i in range(args.robots):
        # 场地 x∈[-14,14], y∈[-7,7]; 机器人分布在对侧 (x>0 或 x<0 视视角)
        rx = rng.uniform(-12, 12)
        ry = rng.uniform(-6, 6)
        rz = 0.0
        robot = make_robot()
        robot = robot + np.array([rx, ry, rz])
        robot_pts.append(robot)
        print(f"[synth] robot {i}: center=({rx:.2f}, {ry:.2f}, {rz})  {len(robot)} pts")
    pts_all = np.vstack([pts_src] + robot_pts) if robot_pts else pts_src
    print(f"[synth] total with robots: {len(pts_all)} points")

    # 工作系 → 雷达系
    pl = (pts_all - t_gt) @ R_gt

    # FOV + 量程裁剪 (odin1 扇形)
    x, y, z = pl[:, 0], pl[:, 1], pl[:, 2]
    rng_v = np.sqrt(x * x + y * y + z * z)
    az = np.rad2deg(np.arctan2(y, x))
    el = np.rad2deg(np.arctan2(z, np.hypot(x, y)))
    mask = (x > 0) & (np.abs(az) <= args.hfov) & (np.abs(el) <= args.vfov) \
         & (rng_v >= args.range_min) & (rng_v <= args.range_max)
    pl, az, el, rng_v = pl[mask], az[mask], el[mask], rng_v[mask]
    print(f"[synth] after FOV: {len(pl)} points")

    # z-buffer 遮挡
    ai = np.floor((az + args.hfov) / args.ang_res).astype(np.int64)
    ei = np.floor((el + args.vfov) / args.ang_res).astype(np.int64)
    cell = ai * 100000 + ei
    order = np.argsort(rng_v)
    cell_sorted = cell[order]
    _, first_idx = np.unique(cell_sorted, return_index=True)
    keep = order[first_idx]
    pl = pl[keep]
    print(f"[synth] after occlusion: {len(pl)} points")

    if args.noise_sigma > 0:
        pl = pl + rng.normal(0, args.noise_sigma, pl.shape)

    write_pcd_binary_xyz(args.output, pl)
    print(f"[synth] wrote {len(pl)} points -> {args.output}")
    print(f"[synth] GT t_map_lidar = {t_gt}")
    print(f"[synth] GT quat xyzw = [{R_gt[2,1]-R_gt[1,2]:.4f}, {R_gt[0,2]-R_gt[2,0]:.4f}, {R_gt[1,0]-R_gt[0,1]:.4f}, {np.sqrt(max(0,1+R_gt[0,0]+R_gt[1,1]+R_gt[2,2]))/2:.4f}]")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
