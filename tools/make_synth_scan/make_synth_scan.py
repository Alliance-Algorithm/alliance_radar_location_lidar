#!/usr/bin/env python3
"""合成一个已知真值部署位姿下的 Odin scan，用于验证配准工具能否恢复 T_gt。

v2: 逼近真实 Odin1 传感器
  - 真实 FOV: 方位 ±60° (120°), 俯仰 ±45° (90°)
  - 量程: 1~70m (Odin1 官方 70m@90% 反射率)
  - 遮挡: 方位-俯仰角 z-buffer, 每格只保留最近点 (近处挡住远处, 真雷达看不穿)
  - 2cm 高斯噪声 (dTOF 测距噪声)

流程:
  读规范地图 (工作系 Z-up, 米) → 用真值位姿 T_gt 把地图点变到雷达系
  → FOV 裁剪 → z-buffer 遮挡 → 加噪声 → 写 binary XYZ PCD

节点算出的 T = T_target_source = T_map_lidar (scan/lidar -> map), 应 ≈ T_gt。
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


def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <map.pcd> <out.pcd>", file=sys.stderr)
        return 1

    map_path = sys.argv[1]
    out_path = sys.argv[2]

    # --- 真值部署位姿 T_gt (工作系: 中心原点/米/Z-up) ---
    eye = np.array([-13.0, 1.0, 4.2])        # 雷达真实位置 (故意偏离标称 -14,0,4)
    target = np.array([0.0, 0.0, 0.5])       # 注视场地中心
    d = target - eye
    d /= np.linalg.norm(d)
    yaw_gt = np.rad2deg(np.arctan2(d[1], d[0]))
    pitch_gt = np.rad2deg(np.arctan2(-d[2], np.hypot(d[0], d[1])))
    yaw_gt += 12.0                            # 额外偏航, 逼搜索去找

    R_gt = rot_z(yaw_gt) @ rot_y(pitch_gt)    # map<-lidar 旋转
    t_gt = eye
    # T_gt: lidar -> map.  p_map = R_gt @ p_lidar + t_gt
    # 逆: p_lidar = R_gt^T @ (p_map - t_gt)
    pts_map = read_pcd_binary_xyz(map_path)
    pl = (pts_map - t_gt) @ R_gt              # 行向量形式 = R_gt^T @ (p_map - t_gt)

    # --- Odin 真实 FOV + 量程裁剪 (雷达系 +x 前) ---
    x, y, z = pl[:, 0], pl[:, 1], pl[:, 2]
    rng = np.sqrt(x * x + y * y + z * z)
    az = np.rad2deg(np.arctan2(y, x))                 # 方位角
    el = np.rad2deg(np.arctan2(z, np.hypot(x, y)))    # 俯仰角
    HFOV, VFOV = 60.0, 45.0                           # Odin1: 120°x90° → 半角
    RMIN, RMAX = 1.0, 70.0                            # Odin1 量程
    mask = (x > 0) & (np.abs(az) <= HFOV) & (np.abs(el) <= VFOV) \
        & (rng >= RMIN) & (rng <= RMAX)
    pl, az, el, rng = pl[mask], az[mask], el[mask], rng[mask]

    # --- z-buffer 遮挡: 方位-俯仰角分格, 每格只保留最近点 ---
    # 真雷达每条射线只命中第一个表面, 近处物体挡住背后的点。
    ANG_RES = 0.2                                     # 角分辨率 (度)
    ai = np.floor((az + HFOV) / ANG_RES).astype(np.int64)
    ei = np.floor((el + VFOV) / ANG_RES).astype(np.int64)
    cell = ai * 100000 + ei
    order = np.argsort(rng)                           # 近→远
    cell_sorted = cell[order]
    _, first_idx = np.unique(cell_sorted, return_index=True)
    keep = order[first_idx]                           # 每格最近点
    pl = pl[keep]

    # --- 噪声 ---
    pl = pl + np.random.default_rng(1).normal(0, 0.02, pl.shape)  # 2cm 噪声

    write_pcd_binary_xyz(out_path, pl)
    print(f"[synth] GT eye={eye} yaw={yaw_gt:.3f}° pitch={pitch_gt:.3f}°")
    print(f"[synth] GT translation (t_map_lidar) = {t_gt}")
    print(f"[synth] FOV=±{HFOV}°x±{VFOV}° range={RMIN}-{RMAX}m occlusion@{ANG_RES}°")
    print(f"[synth] wrote {len(pl)} points -> {out_path}")

    R = R_gt
    w = np.sqrt(max(0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    qx = (R[2, 1] - R[1, 2]) / (4 * w)
    qy = (R[0, 2] - R[2, 0]) / (4 * w)
    qz = (R[1, 0] - R[0, 1]) / (4 * w)
    print(f"[synth] GT quat xyzw = [{qx:.4f}, {qy:.4f}, {qz:.4f}, {w:.4f}]")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
