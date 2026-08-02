#!/usr/bin/env python3
"""
LIO Pose Graph Optimization with Loop Closure

Pipelined offline SLAM backend: reads LIO odometry from ROS2 bag or CSV,
detects loop closures via pose proximity, runs Levenberg-Marquardt on
SE(3), and re-projects submap clouds into a globally-consistent map.

No external dependency beyond numpy.

Usage:
  # From ROS2 bag (recommended):
  python3 optimize_pose_graph.py --bag lio_data/ --output-dir out/

  # From pre-extracted CSV:
  python3 optimize_pose_graph.py --csv lio_poses.csv --output-dir out/

  # With PCD segments (coarse):
  python3 optimize_pose_graph.py --pcd-dir model/generated/ --output-dir out/
"""

import argparse
import os
import struct
import sys
from pathlib import Path
from typing import Optional, Tuple, List

import numpy as np


# ═══════════════════════════════════════════════════════════════════
# SE(3) Lie algebra
# ═══════════════════════════════════════════════════════════════════

def skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def quat_to_rot(qx, qy, qz, qw):
    return np.array([
        [1 - 2*qy*qy - 2*qz*qz,  2*qx*qy - 2*qz*qw,      2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw,      1 - 2*qx*qx - 2*qz*qz,  2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw,      2*qy*qz + 2*qx*qw,      1 - 2*qx*qx - 2*qy*qy]
    ])


def rot_to_quat(R):
    qw = np.sqrt(max(0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    if qw > 1e-8:
        return np.array([(R[2, 1] - R[1, 2]) / (4*qw),
                         (R[0, 2] - R[2, 0]) / (4*qw),
                         (R[1, 0] - R[0, 1]) / (4*qw), qw])
    return np.array([0.0, 0.0, 0.0, 1.0])


def se3_log(T: np.ndarray) -> np.ndarray:
    """SE(3) -> se(3): returns [tx, ty, tz, rx, ry, rz]."""
    R, t = T[:3, :3], T[:3, 3]
    theta = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    if abs(theta) < 1e-8:
        return np.array([t[0], t[1], t[2], 0.0, 0.0, 0.0])
    ln = theta / (2 * np.sin(theta)) * (R - R.T)
    rho = np.array([ln[2, 1], ln[0, 2], ln[1, 0]])
    a, b = np.sin(theta) / theta, (1 - np.cos(theta)) / (theta * theta)
    J = np.eye(3) + 0.5 * skew(rho) + (1 / (theta*theta)) * (1 - a / (2*b)) * skew(rho) @ skew(rho)
    return np.array([*np.linalg.solve(J, t), *rho])


def se3_exp(xi: np.ndarray) -> np.ndarray:
    """se(3) -> SE(3): xi = [tx, ty, tz, rx, ry, rz]."""
    rho, phi = xi[:3], xi[3:]
    theta = np.linalg.norm(phi)
    T = np.eye(4)
    if theta < 1e-8:
        T[:3, 3] = rho
        return T
    axis = phi / theta
    K = skew(axis)
    R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * K @ K
    V = np.eye(3) + (1 - np.cos(theta)) / theta * K + (theta - np.sin(theta)) / theta * K @ K
    T[:3, :3] = R
    T[:3, 3] = V @ rho
    return T


def se3_adj(T: np.ndarray) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    A = np.zeros((6, 6))
    A[:3, :3] = A[3:, 3:] = R
    A[:3, 3:] = skew(t) @ R
    return A


# ═══════════════════════════════════════════════════════════════════
# Data loading
# ═══════════════════════════════════════════════════════════════════

def load_poses_from_csv(path: str) -> np.ndarray:
    """Load poses from CSV: ts, x, y, z, qx, qy, qz, qw per row. Returns Nx4x4."""
    data = np.loadtxt(path, delimiter=",", dtype=float)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    poses = np.zeros((len(data), 4, 4))
    for i, row in enumerate(data):
        x, y, z = row[1:4]
        qx, qy, qz, qw = row[4:8]
        poses[i, :3, :3] = quat_to_rot(qx, qy, qz, qw)
        poses[i, :3, 3] = [x, y, z]
        poses[i, 3, 3] = 1.0
    return poses


def load_poses_from_bag(bag_path: str, topic: str = "/fast_livo2/odom") -> Tuple[np.ndarray, list]:
    """Read all odometry from a ROS2 bag. Returns (Nx4x4 poses, timestamps list)."""
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry

    reader = SequentialReader()
    storage_id = "mcap" if bag_path.endswith(".mcap") else ""
    reader.open(StorageOptions(uri=bag_path, storage_id=storage_id),
                ConverterOptions(input_serialization_format="cdr",
                                 output_serialization_format="cdr"))
    topics = {t.name for t in reader.get_all_topics_and_types()}
    if topic not in topics:
        print(f"ERROR: '{topic}' not in bag. Available: {sorted(topics)}")
        sys.exit(1)

    poses_list = []
    timestamps = []
    while reader.has_next():
        tn, data, _ = reader.read_next()
        if tn != topic:
            continue
        msg = deserialize_message(data, Odometry)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        T = np.eye(4)
        T[:3, :3] = quat_to_rot(q.x, q.y, q.z, q.w)
        T[:3, 3] = [p.x, p.y, p.z]
        poses_list.append(T)
        timestamps.append(ts)

    print(f"Loaded {len(poses_list)} poses from bag ({bag_path})")
    poses_arr = np.array(poses_list) if poses_list else np.zeros((0, 4, 4))
    return poses_arr, timestamps


def load_poses_from_pcd_centroids(pcd_dir: str) -> np.ndarray:
    """Extract per-segment centroids from saved PCD segments. Coarse approx."""
    import glob
    files = sorted(glob.glob(os.path.join(pcd_dir, "fast_livo2_map.*.pcd")))
    if not files:
        files = sorted(glob.glob(os.path.join(pcd_dir, "lio_map.*.pcd")))
    poses = []
    for f in files:
        pts = _read_pcd_xyz(f)
        if len(pts) == 0:
            continue
        c = np.mean(pts, axis=0)
        T = np.eye(4)
        T[:3, 3] = c
        poses.append(T)
    print(f"Extracted {len(poses)} centroids from {len(files)} PCD segments")
    return np.array(poses) if poses else np.zeros((0, 4, 4))


def _read_pcd_xyz(path: str) -> np.ndarray:
    points = []
    in_data = False
    x_off, y_off, z_off = 0, 1, 2
    step = 0
    with open(path, "rb") as f:
        for line in f:
            line = line.decode("utf-8", errors="ignore").strip()
            if line.startswith("DATA"):
                in_data = True
                break
            if line.startswith("FIELDS"):
                fields = line.split()[1:]
                step = len(fields)
                for k, n in enumerate(fields):
                    if n == "x": x_off = k
                    elif n == "y": y_off = k
                    elif n == "z": z_off = k
        if not in_data:
            return np.array([])
        raw = f.read()
    n = len(raw) // (step * 4)
    if n == 0:
        return np.array([])
    arr = np.frombuffer(raw, dtype=np.float32).reshape(n, step)
    return arr[:, [x_off, y_off, z_off]]


# ═══════════════════════════════════════════════════════════════════
# Pose Graph
# ═══════════════════════════════════════════════════════════════════

def extract_keyframes(poses: np.ndarray, dist_thresh: float = 0.5,
                      ang_thresh_deg: float = 15.0) -> np.ndarray:
    """Distance + angle based keyframe selection. Returns (K, 4, 4)."""
    if len(poses) < 2:
        return poses
    kf_indices = [0]
    last_p = poses[0, :3, 3]
    last_R = poses[0, :3, :3]
    ang_thresh = np.deg2rad(ang_thresh_deg)

    for i in range(1, len(poses)):
        p = poses[i, :3, 3]
        R = poses[i, :3, :3]
        dist = np.linalg.norm(p - last_p)
        ang = np.arccos(np.clip((np.trace(last_R.T @ R) - 1) / 2, -1, 1))
        if dist >= dist_thresh or ang >= ang_thresh:
            kf_indices.append(i)
            last_p = p
            last_R = R

    if kf_indices[-1] != len(poses) - 1:
        kf_indices.append(len(poses) - 1)

    print(f"Keyframes: {len(kf_indices)} (from {len(poses)} frames, "
          f"dist≥{dist_thresh}m, ang≥{ang_thresh_deg}°)")
    return poses[kf_indices], np.array(kf_indices, dtype=int)


class PoseGraph:
    def __init__(self, nodes: np.ndarray):
        self.nodes = nodes.copy()  # (N, 4, 4)
        self.edges = []  # list of (i, j, T_ij, info)

    def add_odometry_edges(self, info_diag=None):
        if info_diag is None:
            info_diag = [100.0, 100.0, 100.0, 1000.0, 1000.0, 1000.0]
        info = np.diag(info_diag)
        for k in range(len(self.nodes) - 1):
            T_ij = np.linalg.inv(self.nodes[k]) @ self.nodes[k + 1]
            self.edges.append((k, k + 1, T_ij, info, False))

    def add_loop_edges(self, radius: float = 2.0, min_skip: int = 20,
                       info_diag=None, heading_thresh_deg: float = 45.0):
        if info_diag is None:
            info_diag = [50.0, 50.0, 50.0, 200.0, 200.0, 200.0]
        info = np.diag(info_diag)
        heading_thresh = np.deg2rad(heading_thresh_deg)
        loops = 0
        for a in range(len(self.nodes)):
            p_a = self.nodes[a, :3, 3]
            for b in range(a + min_skip, len(self.nodes)):
                p_b = self.nodes[b, :3, 3]
                if np.linalg.norm(p_a - p_b) < radius:
                    ang = np.arccos(np.clip(
                        (np.trace(self.nodes[a, :3, :3].T @ self.nodes[b, :3, :3]) - 1) / 2,
                        -1, 1))
                    if ang < heading_thresh:
                        self.edges.append((a, b, np.eye(4), info, True))
                        loops += 1
        print(f"Loop edges: {loops} (radius={radius}m, min_skip={min_skip}, "
              f"heading<{heading_thresh_deg}°)")

    # ── Levenberg-Marquardt on SE(3) ──

    def optimize(self, max_iter: int = 50, lambda_init: float = 1e-3,
                 loop_reject_thresh: float = 1.0, reject_rounds: int = 6) -> np.ndarray:
        n = len(self.nodes)
        edges = list(self.edges)
        n_rejected = 0

        def compute_cost(state, edges_sub):
            cost = 0.0
            for i, j, T_ij_meas, info, _is_loop in edges_sub:
                T_ij_est = np.linalg.inv(state[i]) @ state[j]
                e = se3_log(np.linalg.inv(T_ij_meas) @ T_ij_est)
                cost += float(e @ info @ e)
            return cost

        def lm(edges_sub):
            """Levenberg-Marquardt with step acceptance（错误检查 + λ 升降）。

            纯 Gauss-Newton 在矛盾约束（假回环边）下线性化失效会发散，
            必须验证候选步是否真的降低代价：不降则拒绝并放大 λ。
            """
            state = self.nodes.copy()
            lamb = lambda_init
            prev_cost = None
            for it in range(max_iter):
                H = np.zeros((6 * n, 6 * n))
                b = np.zeros(6 * n)
                for i, j, T_ij_meas, info, _is_loop in edges_sub:
                    T_wi, T_wj = state[i], state[j]
                    T_ij_est = np.linalg.inv(T_wi) @ T_wj
                    e = se3_log(np.linalg.inv(T_ij_meas) @ T_ij_est)
                    Ji = -se3_adj(np.linalg.inv(T_ij_est))
                    Jj = se3_adj(np.linalg.inv(T_ij_est))
                    bi, bj = slice(6*i, 6*(i+1)), slice(6*j, 6*(j+1))
                    H[bi, bi] += Ji.T @ info @ Ji
                    H[bi, bj] += Ji.T @ info @ Jj
                    H[bj, bi] += Jj.T @ info @ Ji
                    H[bj, bj] += Jj.T @ info @ Jj
                    b[bi] += Ji.T @ info @ e
                    b[bj] += Jj.T @ info @ e
                H[:6, :6] += np.eye(6) * 1e6  # anchor first node

                accepted = False
                for _ in range(12):  # inner lambda escalation
                    try:
                        delta = np.linalg.solve(H + lamb * np.diag(np.diag(H)), -b)
                    except np.linalg.LinAlgError:
                        lamb *= 10.0
                        continue
                    cand = state.copy()
                    for i in range(n):
                        xi = delta[6*i:6*(i+1)]
                        cand[i] = cand[i] @ se3_exp(xi)
                    cost = compute_cost(cand, edges_sub)
                    if prev_cost is None or cost < prev_cost:
                        state = cand
                        prev_cost = cost
                        lamb = max(lamb * 0.7, 1e-9)
                        accepted = True
                        break
                    lamb *= 10.0
                if not accepted:
                    break
                if float(np.linalg.norm(delta)) < 1e-6:
                    break
            return state

        # 残差剔除循环：漂移位姿的近邻检测会产生假回环边（真实场景中两条
        # 相距 <2m 但不在同一位置的轨迹段），矛盾约束会把优化器拧飞（实测
        # 中间段关键帧被拉飞数百米）。每轮优化后丢掉平移残差超阈值的回环边，
        # 从原始位姿重新优化，直到没有超阈值边或达到轮次上限。
        for rnd in range(reject_rounds):
            state = lm(edges)
            worst = None
            for idx, (i, j, T_ij_meas, info, is_loop) in enumerate(edges):
                if not is_loop:
                    continue
                T_ij_est = np.linalg.inv(state[i]) @ state[j]
                e = se3_log(np.linalg.inv(T_ij_meas) @ T_ij_est)
                t_err = float(np.linalg.norm(e[:3]))
                if worst is None or t_err > worst[0]:
                    worst = (t_err, idx)
            if worst is None or worst[0] <= loop_reject_thresh:
                break
            n_rejected += 1
            print(f"  Rejected loop edge #{worst[1]} (residual {worst[0]:.2f}m)")
            del edges[worst[1]]

        if n_rejected:
            n_loop_kept = sum(1 for e in edges if e[4])
            print(f"Loop edge rejection: dropped {n_rejected}, kept {n_loop_kept}")
        print(f"Optimized {len(edges)} edges on {n} nodes, {max_iter} iters")
        return state


# ═══════════════════════════════════════════════════════════════════
# Point cloud re-projection
# ═══════════════════════════════════════════════════════════════════

def load_clouds_from_bag(bag_path: str, topic: str = "/fast_livo2/cloud_world"
                         ) -> list[Tuple[float, np.ndarray]]:
    """Read all world clouds from bag. Returns list of (timestamp_sec, Nx3 array)."""
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    reader = SequentialReader()
    storage_id = "mcap" if bag_path.endswith(".mcap") else ""
    reader.open(StorageOptions(uri=bag_path, storage_id=storage_id),
                ConverterOptions(input_serialization_format="cdr",
                                 output_serialization_format="cdr"))
    topic_names = {t.name for t in reader.get_all_topics_and_types()}
    if topic not in topic_names:
        print(f"WARN: '{topic}' not in bag, skip cloud re-projection")
        return []

    cloud_list = []
    while reader.has_next():
        tn, data, _ = reader.read_next()
        if tn != topic:
            continue
        msg = deserialize_message(data, PointCloud2)
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pts = _parse_pointcloud2(msg)
        if len(pts) > 0:
            cloud_list.append((ts, pts))
    print(f"Loaded {len(cloud_list)} clouds from bag")
    return cloud_list


def _parse_pointcloud2(msg) -> np.ndarray:
    """Extract xyz from PointCloud2 into Nx3 float array."""
    from sensor_msgs.msg import PointField
    x_off, y_off, z_off = 0, 4, 8
    step = msg.point_step
    for f in msg.fields:
        if f.name == "x": x_off = f.offset
        elif f.name == "y": y_off = f.offset
        elif f.name == "z": z_off = f.offset
    n = len(msg.data) // step
    if n == 0:
        return np.array([])
    arr = np.frombuffer(msg.data, dtype=np.float32).reshape(n, step // 4)
    return arr[:, [x_off // 4, y_off // 4, z_off // 4]]


def re_project_maps(
    kf_poses_orig: np.ndarray,   # (K, 4, 4) original keyframe poses
    kf_poses_opt: np.ndarray,    # (K, 4, 4) optimized keyframe poses
    clouds: list[Tuple[float, np.ndarray]],  # (timestamp, Nx3)
    kf_timestamps: list[float],  # timestamps for each keyframe
    output_path: str
):
    """Re-project world clouds with delta transforms and merge into global map."""
    if not clouds:
        print("No clouds available, skip re-projection")
        return

    cloud_ts = np.array([c[0] for c in clouds])
    merged = []
    used = 0

    for k in range(len(kf_poses_orig)):
        # Find nearest cloud to this keyframe timestamp
        dt = np.abs(cloud_ts - kf_timestamps[k])
        idx = np.argmin(dt)
        if dt[idx] > 0.5:  # too far: skip
            continue

        pts = clouds[idx][1]
        T_orig = kf_poses_orig[k]
        T_opt = kf_poses_opt[k]

        # delta_T = T_opt @ T_orig^{-1}
        # P_opt = delta_T @ P_orig
        delta = T_opt @ np.linalg.inv(T_orig)
        R_delta, t_delta = delta[:3, :3], delta[:3, 3]
        pts_opt = (R_delta @ pts.T + t_delta.reshape(3, 1)).T
        merged.append(pts_opt)
        used += 1

    if not merged:
        print("No clouds matched to keyframes (time tolerance too tight?)")
        return

    merged = np.vstack(merged)
    with open(output_path, "wb") as f:
        header = (
            f"# .PCD v0.7 - Point Cloud Data file format\n"
            f"VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
            f"WIDTH {len(merged)}\nHEIGHT 1\n"
            f"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(merged)}\nDATA binary\n"
        )
        f.write(header.encode())
        f.write(merged.astype(np.float32).tobytes())
    print(f"Saved re-projected map: {output_path} ({len(merged)} pts from {used}/{len(kf_poses_orig)} keyframes)")


def re_project_pcd_segments(
    kf_poses_orig: np.ndarray,
    kf_poses_opt: np.ndarray,
    pcd_dir: str,
    output_path: str
):
    """Re-project periodic flush PCD segments with optimized poses."""
    import glob
    files = sorted(glob.glob(os.path.join(pcd_dir, "fast_livo2_map.*.pcd")))
    if not files:
        files = sorted(glob.glob(os.path.join(pcd_dir, "lio_map.*.pcd")))
    if not files:
        print("No PCD segments found")
        return

    # Each segment associates with the closest keyframe centroid
    merged = []
    for f_idx, f in enumerate(files):
        pts = _read_pcd_xyz(f)
        if len(pts) == 0:
            continue
        centroid = np.mean(pts, axis=0)
        # Find nearest keyframe
        dists = np.linalg.norm(kf_poses_orig[:, :3, 3] - centroid, axis=1)
        k = int(np.argmin(dists))

        T_orig = kf_poses_orig[k]
        T_opt = kf_poses_opt[k]
        delta = T_opt @ np.linalg.inv(T_orig)
        R_delta, t_delta = delta[:3, :3], delta[:3, 3]
        pts_opt = (R_delta @ pts.T + t_delta.reshape(3, 1)).T
        merged.append(pts_opt)

    merged = np.vstack(merged)
    with open(output_path, "wb") as f:
        header = (
            f"# .PCD v0.7 - Point Cloud Data file format\n"
            f"VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
            f"WIDTH {len(merged)}\nHEIGHT 1\n"
            f"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(merged)}\nDATA binary\n"
        )
        f.write(header.encode())
        f.write(merged.astype(np.float32).tobytes())
    print(f"Saved re-projected map: {output_path} ({len(merged)} pts from {len(files)} segments)")

def save_poses_csv(poses: np.ndarray, path: str):
    with open(path, "w") as f:
        for T in poses:
            q = rot_to_quat(T[:3, :3])
            f.write(f"{T[0,3]:.6f},{T[1,3]:.6f},{T[2,3]:.6f},{q[0]:.6f},{q[1]:.6f},{q[2]:.6f},{q[3]:.6f}\n")
    print(f"Saved: {path} ({len(poses)} poses)")


def print_drift_report(poses_before: np.ndarray, poses_after: np.ndarray):
    """Compare start->end displacement before/after optimization."""
    p_beg = poses_before[0, :3, 3]
    p_end = poses_before[-1, :3, 3]
    drift_before = np.linalg.norm(p_end - p_beg)
    path_before = np.sum(np.linalg.norm(
        poses_before[1:, :3, 3] - poses_before[:-1, :3, 3], axis=1))

    p_opt_end = poses_after[-1, :3, 3]
    drift_after = np.linalg.norm(p_opt_end - p_beg)
    path_after = np.sum(np.linalg.norm(
        poses_after[1:, :3, 3] - poses_after[:-1, :3, 3], axis=1))

    print()
    print("=" * 56)
    print("  Drift Report")
    print("=" * 56)
    print(f"  Total path:   {path_before:.1f}m")
    print(f"  Drift (before optimize): {drift_before:.3f}m  ({drift_before/path_before*100:.2f}%)")
    print(f"  Drift (after  optimize): {drift_after:.3f}m  ({drift_after/path_before*100:.2f}%)")
    print(f"  Improvement:  {(1 - drift_after/max(drift_before, 1e-6))*100:.1f}%")
    print("=" * 56)
    print()


# ═══════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="LIO Pose Graph Optimization with Loop Closure")
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument("--bag", help="ROS2 bag path")
    src.add_argument("--csv", help="Pre-extracted poses CSV")
    src.add_argument("--pcd-dir", help="PCD segment directory (centroid-based, coarse)")
    parser.add_argument("--topic", default="/fast_livo2/odom", help="Odometry topic in bag")
    parser.add_argument("--output-dir", default=".", help="Output directory")
    parser.add_argument("--keyframe-dist", type=float, default=0.3,
                        help="Keyframe distance threshold (m)")
    parser.add_argument("--keyframe-ang", type=float, default=10.0,
                        help="Keyframe angle threshold (deg)")
    parser.add_argument("--loop-radius", type=float, default=2.0,
                        help="Loop closure search radius (m)")
    parser.add_argument("--loop-min-skip", type=int, default=30,
                        help="Minimum keyframe index gap for loop detection")
    parser.add_argument("--reproject", action="store_true",
                        help="Re-project point clouds with optimized poses into global map")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    # ── Load ──
    raw_poses = None
    raw_timestamps = []
    if args.bag:
        raw_poses, raw_timestamps = load_poses_from_bag(args.bag, args.topic)
    elif args.csv:
        raw_poses = load_poses_from_csv(args.csv)
    else:
        raw_poses = load_poses_from_pcd_centroids(args.pcd_dir)

    if len(raw_poses) < 2:
        print("ERROR: too few poses"); sys.exit(1)

    # ── Keyframe ──
    kf_poses, kf_indices = extract_keyframes(
        raw_poses, args.keyframe_dist, args.keyframe_ang)

    # Compute keyframe timestamps
    kf_timestamps = [raw_timestamps[i] for i in kf_indices] if raw_timestamps else \
                    [float(i) for i in range(len(kf_poses))]

    # ── Build graph ──
    pg = PoseGraph(kf_poses)
    pg.add_odometry_edges()
    pg.add_loop_edges(args.loop_radius, args.loop_min_skip)
    n_odom = sum(1 for e in pg.edges if not np.allclose(e[2], np.eye(4)))
    n_loop = len(pg.edges) - n_odom
    print(f"Graph: {len(pg.nodes)} nodes, {n_odom} odom edges, {n_loop} loop edges")

    # ── Optimize ──
    opt_poses = pg.optimize()
    save_poses_csv(kf_poses, os.path.join(args.output_dir, "keyframe_before.csv"))
    save_poses_csv(opt_poses, os.path.join(args.output_dir, "keyframe_optimized.csv"))

    # ── Report ──
    print_drift_report(kf_poses, opt_poses)

    # ── Re-projection ──
    if args.reproject:
        if args.bag:
            clouds = load_clouds_from_bag(args.bag)
            if clouds:
                re_project_maps(kf_poses, opt_poses, clouds, kf_timestamps,
                                os.path.join(args.output_dir, "map_optimized.pcd"))
        elif args.pcd_dir:
            re_project_pcd_segments(kf_poses, opt_poses, args.pcd_dir,
                                    os.path.join(args.output_dir, "map_optimized.pcd"))

    # ── Trajectory visualisation CSV ──
    with open(os.path.join(args.output_dir, "trajectory_before.csv"), "w") as f:
        for T in kf_poses:
            f.write(f"{T[0,3]:.6f},{T[1,3]:.6f},{T[2,3]:.6f}\n")
    with open(os.path.join(args.output_dir, "trajectory_after.csv"), "w") as f:
        for T in opt_poses:
            f.write(f"{T[0,3]:.6f},{T[1,3]:.6f},{T[2,3]:.6f}\n")
    print(f"Trajectory CSVs saved to {args.output_dir}/")


if __name__ == "__main__":
    main()
