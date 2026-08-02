#!/usr/bin/env python3
"""把彩色地图（LIO 位姿着色）的 RGB 迁移到回环优化后的几何地图上。

用法:
    python3 color_transfer.py <colored.pcd> <geometry.pcd> --output out.pcd
        [--threshold 0.15] [--grid 0.05]

算法:
    彩色点按 grid 建网格索引（floor-divide + lexsort 分段，不用 XOR 哈希）；
    几何点先按 grid 去重（每格保留首个），再在所在 3×3×3 邻域网格内找
    欧氏最近彩色点，距离 <= threshold 则复制 rgb，否则丢弃该点——
    避免给优化几何上出"无中生有"的假色。

    为什么够用: 回环修正量与 LIO 漂移同量级（实测 40m 走圈 19mm），
    << threshold 0.15m；同一表面的彩色点与几何点几乎重合，27 邻域搜索
    必然命中。纯 numpy，无 scipy/KDTree。

输入输出均为 PCD v0.7（输入 ascii/binary 均可；输出 binary，
FIELDS x y z rgb，rgb 为 PCL 兼容 float32 位模式）。
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np

_PCD_DTYPE_MAP = {
    ('F', 4): '<f4', ('F', 8): '<f8',
    ('U', 1): 'u1', ('U', 2): '<u2', ('U', 4): '<u4', ('U', 8): '<u8',
    ('I', 1): 'i1', ('I', 2): '<i2', ('I', 4): '<i4', ('I', 8): '<i8',
}


def read_pcd(path):
    """读取 PCD（ASCII/binary），返回全部字段的 np.recarray（COUNT=1）。"""
    with open(path, 'rb') as f:
        header_lines = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"PCD 文件缺少 DATA 行: {path}")
            text = line.decode('ascii', errors='replace').strip()
            header_lines.append(text)
            if text.startswith('DATA'):
                data_mode = text.split()[1].lower()
                break
            if len(header_lines) > 50:
                raise ValueError(f"PCD 头部异常: {path}")
        header = {}
        for line in header_lines[:-1]:
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            header[parts[0]] = parts[1:]

        fields = header['FIELDS']
        sizes = [int(s) for s in header['SIZE']]
        types = header['TYPE']
        counts = [int(c) for c in header.get('COUNT', ['1'] * len(fields))]
        n_points = int(header['POINTS'][0])
        if any(c != 1 for c in counts):
            raise ValueError("本工具只支持 COUNT=1 的字段")

        dtype_fields = []
        for name, size, typ in zip(fields, sizes, types):
            key = (typ, size)
            if key not in _PCD_DTYPE_MAP:
                raise ValueError(f"不支持的字段类型 {name}: TYPE={typ} SIZE={size}")
            dtype_fields.append((name, _PCD_DTYPE_MAP[key]))
        dtype = np.dtype(dtype_fields)

        if data_mode == 'binary':
            raw = f.read(n_points * dtype.itemsize)
            records = np.frombuffer(raw, dtype=dtype, count=n_points)
        elif data_mode == 'ascii':
            rest = f.read().decode('ascii', errors='replace')
            rows = [line.split() for line in rest.splitlines() if line.strip()]
            records = np.zeros(n_points, dtype=dtype)
            for i, row in enumerate(rows[:n_points]):
                for (name, _), val in zip(dtype_fields, row):
                    records[name][i] = float(val)
        else:
            raise ValueError(f"不支持的 DATA 模式: {data_mode}")
    return records


def pack_rgb(r, g, b):
    """(r,g,b) uint8 数组 → PCL float32 位模式数组。"""
    packed = (r.astype(np.uint32) << 16) | (g.astype(np.uint32) << 8) | b.astype(np.uint32)
    return packed.view('<f4')


def unpack_rgb(arr):
    """float32 位模式数组 → uint32 位模式数组。"""
    return arr.view('<u4')


def write_pcd_rgb(path, xyz, rgb_packed):
    """写 PointXYZRGB binary PCD；rgb_packed 为 uint32 位模式。"""
    n = len(xyz)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z rgb\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    record = np.empty(n, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<f4')])
    record['x'] = xyz[:, 0]
    record['y'] = xyz[:, 1]
    record['z'] = xyz[:, 2]
    record['rgb'] = rgb_packed.astype(np.uint32).view('<f4')
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(record.tobytes())


def _cell_segments(xyz, grid):
    """按 grid 分格 + lexsort 分段，返回 (cell_idx, 排序后的点坐标,
    {cell_key: (start, end)})。"""
    cell = np.floor(xyz / grid).astype(np.int64)
    order = np.lexsort((cell[:, 2], cell[:, 1], cell[:, 0]))
    cell_s = cell[order]
    pts_s = xyz[order]
    n = len(pts_s)
    boundaries = np.concatenate([[0], 1 + np.where(
        (cell_s[1:, 0] != cell_s[:-1, 0]) |
        (cell_s[1:, 1] != cell_s[:-1, 1]) |
        (cell_s[1:, 2] != cell_s[:-1, 2])
    )[0], [n]])
    seg = {tuple(cell_s[boundaries[i]]): (boundaries[i], boundaries[i + 1])
           for i in range(len(boundaries) - 1)}
    return cell, cell_s, pts_s, seg


def _voxel_first(xyz, grid):
    """体素降采样：每格保留首个点。"""
    cell = np.floor(xyz / grid).astype(np.int64)
    _, first = np.unique(cell, axis=0, return_index=True)
    first = np.sort(first)
    return xyz[first]


def prealign(source_xyz, target_xyz, grid=1.0, iterations=25):
    """粗 ICP 预对齐（纯 numpy）：把 source 刚体变换到 target 坐标系。

    回环优化后的几何（target）相对 LIO 彩色图（source）有整体漂移
    （实测 40m 走圈累积 ~1.9m），超过颜色迁移阈值导致大面积失配。
    本函数做：体素降采样 → 质心预对齐（漂移主要是平移）→ 迭代最近点
    （27 邻域网格最近邻 + SVD/Umeyama 位姿求解）。

    返回 (R, t)：p' = R @ p + t 把 source 变换到 target 坐标系。
    """
    src = _voxel_first(source_xyz, grid).astype(np.float64)
    tgt = _voxel_first(target_xyz, grid).astype(np.float64)
    if len(src) < 20 or len(tgt) < 20:
        raise ValueError(f"prealign: 降采样后点数过少 (src={len(src)}, tgt={len(tgt)})")

    src_mean = src.mean(axis=0)
    tgt_mean = tgt.mean(axis=0)
    src = src - src_mean
    tgt_c = tgt - tgt_mean
    R = np.eye(3)
    prev_err = np.inf

    for _ in range(iterations):
        _, _, tgt_s, seg = _cell_segments(tgt_c, grid)
        src_cell = np.floor(src / grid).astype(np.int64)
        nn_idx = np.full(len(src), -1, dtype=np.int64)
        for i in range(len(src)):
            cx, cy, cz = src_cell[i]
            best_d2 = np.inf
            best_j = -1
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        se = seg.get((cx + dx, cy + dy, cz + dz))
                        if se is None:
                            continue
                        d2 = ((tgt_s[se[0]:se[1]] - src[i]) ** 2).sum(axis=1)
                        j = int(d2.argmin())
                        if d2[j] < best_d2:
                            best_d2 = float(d2[j])
                            best_j = se[0] + j
            nn_idx[i] = best_j

        valid = nn_idx >= 0
        n_valid = int(valid.sum())
        if n_valid < 20:
            break
        a = src[valid]
        b = tgt_s[nn_idx[valid]]
        ma, mb = a.mean(axis=0), b.mean(axis=0)
        H = (a - ma).T @ (b - mb)
        U, _, Vt = np.linalg.svd(H)
        D = np.eye(3)
        D[2, 2] = np.sign(np.linalg.det(U @ Vt))
        Rk = Vt.T @ D @ U.T
        tk = mb - Rk @ ma
        src = (Rk @ src.T).T + tk
        R = Rk @ R
        err = float(np.sqrt(((a - b) ** 2).sum(axis=1).mean()))
        if abs(prev_err - err) < 1e-4:
            break
        prev_err = err

    t = tgt_mean - R @ src_mean
    return R, t


def load_pose_csv(path):
    """读关键帧位姿 CSV（x,y,z,qx,qy,qz,qw），返回 (N,4,4) SE(3) 数组。"""
    rows = []
    with open(path) as f:
        for line in f:
            parts = line.strip().split(",")
            if len(parts) < 7:
                continue
            x, y, z, qx, qy, qz, qw = (float(v) for v in parts[:7])
            rows.append((x, y, z, qx, qy, qz, qw))
    if not rows:
        raise ValueError(f"位姿 CSV 为空: {path}")
    T = np.zeros((len(rows), 4, 4))
    T[:, 3, 3] = 1.0
    for i, (x, y, z, qx, qy, qz, qw) in enumerate(rows):
        T[i, 0, 3], T[i, 1, 3], T[i, 2, 3] = x, y, z
        R = quat_to_rot(qx, qy, qz, qw)
        T[i, :3, :3] = R
    return T


def warp_by_trajectory(cloud_xyz, before_poses, after_poses, max_correction=3.0):
    """按回环修正量逐点扭曲点云：p' = T_opt(最近关键帧) · T_lio(最近关键帧)⁻¹ · p。

    回环修正量沿轨迹逐帧变化（刚性预对齐无法表达），彩色图点没有帧号，
    用"空间上最近的关键帧"的修正量近似——关键帧间距 0.3m，近似足够。
    修正量超过 max_correction 的关键帧视为优化发散（实测中间段被假回环
    边拉飞数百米），按单位阵处理（不扭曲）。返回变换后的点云。
    """
    before_pos = before_poses[:, :3, 3]
    inv_before = np.linalg.inv(before_poses)
    corr = after_poses @ inv_before  # (N,4,4)：每个关键帧的修正量
    disp = np.linalg.norm(corr[:, :3, 3], axis=1)
    n_clamped = int((disp > max_correction).sum())
    if n_clamped:
        print(f"  钳制发散关键帧: {n_clamped}/{len(corr)}（修正量 > {max_correction}m 置单位阵）")
    corr[disp > max_correction] = np.eye(4)
    out = cloud_xyz.copy()
    chunk = 5000
    for lo in range(0, len(cloud_xyz), chunk):
        hi = min(lo + chunk, len(cloud_xyz))
        pts = cloud_xyz[lo:hi]
        d2 = ((pts[:, None, :] - before_pos[None, :, :]) ** 2).sum(axis=2)
        k = d2.argmin(axis=1)
        tfs = corr[k]
        out[lo:hi] = (tfs[:, :3, :3] @ pts[:, :, None])[:, :, 0] + tfs[:, :3, 3]
    return out


def quat_to_rot(qx, qy, qz, qw):
    """四元数 → 旋转矩阵。"""
    return np.array([
        [1 - 2*qy*qy - 2*qz*qz,  2*qx*qy - 2*qz*qw,      2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw,      1 - 2*qx*qx - 2*qz*qz,  2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw,      2*qy*qz + 2*qx*qw,      1 - 2*qx*qx - 2*qy*qy],
    ])


def sanitize_near_trajectory(cloud_xyz, traj_pos, radius=3.0):
    """剔除离 LIO 轨迹过远的点（优化发散产生的公里级垃圾簇）。

    traj_pos: (N,3) 轨迹点（trajectory_before.csv 的 x,y,z）。
    返回 (保留点, 掩码)。
    """
    cell = np.floor(traj_pos / radius).astype(np.int64)
    order = np.lexsort((cell[:, 2], cell[:, 1], cell[:, 0]))
    cell_s = cell[order]
    pts_s = traj_pos[order]
    n = len(pts_s)
    boundaries = np.concatenate([[0], 1 + np.where(
        (cell_s[1:, 0] != cell_s[:-1, 0]) |
        (cell_s[1:, 1] != cell_s[:-1, 1]) |
        (cell_s[1:, 2] != cell_s[:-1, 2])
    )[0], [n]])
    seg = {tuple(cell_s[boundaries[i]]): (boundaries[i], boundaries[i + 1])
           for i in range(len(boundaries) - 1)}
    keep = np.zeros(len(cloud_xyz), dtype=bool)
    cloud_cell = np.floor(cloud_xyz / radius).astype(np.int64)
    chunk = 20000
    for lo in range(0, len(cloud_xyz), chunk):
        hi = min(lo + chunk, len(cloud_xyz))
        for i in range(lo, hi):
            cx, cy, cz = cloud_cell[i]
            best_d2 = np.inf
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        se = seg.get((cx + dx, cy + dy, cz + dz))
                        if se is None:
                            continue
                        d2 = ((pts_s[se[0]:se[1]] - cloud_xyz[i]) ** 2).min()
                        if d2 < best_d2:
                            best_d2 = float(d2)
            keep[i] = best_d2 <= radius * radius
    return cloud_xyz[keep], keep


def nearest_rgb(colored_xyz, colored_rgb, geom_xyz, grid, threshold, keep_uncolored=False):
    """几何每点在其 3×3×3 邻域网格内找最近彩色点，<=threshold 复制 rgb。

    几何先按 grid 去重（每格保留首个点，保证每格至多一次查询）。返回
    (几何点, 对应 rgb uint32)。keep_uncolored=True 时保留全部几何点，
    未找到彩色邻居的点赋灰色 (128,128,128)，否则丢弃。
    """
    _, _, cpts, cseg = _cell_segments(colored_xyz, grid)
    crgb_s = colored_rgb  # 排序在 _cell_segments 内部完成，但 rgb 也要对齐
    # 修正: _cell_segments 内部排序了坐标，这里需要与排序顺序一致的 rgb
    cell_c = np.floor(colored_xyz / grid).astype(np.int64)
    order_c = np.lexsort((cell_c[:, 2], cell_c[:, 1], cell_c[:, 0]))
    crgb_s = colored_rgb[order_c]

    cell_g = np.floor(geom_xyz / grid).astype(np.int64)
    _, first = np.unique(cell_g, axis=0, return_index=True)
    first = np.sort(first)
    gxyz = geom_xyz[first]
    gcell = cell_g[first]

    t2 = threshold * threshold
    out_rgb = np.zeros(len(gxyz), dtype=np.uint32)
    keep = np.zeros(len(gxyz), dtype=bool)

    offsets = [(dx, dy, dz) for dx in (-1, 0, 1)
               for dy in (-1, 0, 1) for dz in (-1, 0, 1)]
    for i in range(len(gxyz)):
        base = gcell[i]
        best_d2 = np.inf
        best_rgb = 0
        for dx, dy, dz in offsets:
            se = cseg.get((base[0] + dx, base[1] + dy, base[2] + dz))
            if se is None:
                continue
            d2 = ((cpts[se[0]:se[1]] - gxyz[i]) ** 2).sum(axis=1)
            j = int(d2.argmin())
            if d2[j] < best_d2:
                best_d2 = float(d2[j])
                best_rgb = int(crgb_s[se[0] + j])
        if best_d2 <= t2:
            keep[i] = True
            out_rgb[i] = best_rgb
    if keep_uncolored:
        gray = int(pack_rgb(np.array([128], dtype=np.uint32),
                            np.array([128], dtype=np.uint32),
                            np.array([128], dtype=np.uint32)).view(np.uint32)[0])
        out_rgb[~keep] = gray
        return gxyz, out_rgb
    return gxyz[keep], out_rgb[keep]


def main():
    parser = argparse.ArgumentParser(description="彩色地图 RGB 迁移到优化几何地图")
    parser.add_argument("colored", help="彩色 PCD（PointXYZRGB，rgb 为 float32 位模式）")
    parser.add_argument("geometry", help="几何 PCD（回环优化后的 map_optimized.pcd）")
    parser.add_argument("--output", required=True, help="输出 PCD 路径")
    parser.add_argument("--threshold", type=float, default=0.15,
                        help="最大迁移距离，米（默认 0.15）")
    parser.add_argument("--grid", type=float, default=0.05,
                        help="网格边长，米（默认 0.05，邻域搜索 3×3×3）")
    parser.add_argument("--keep-uncolored", action="store_true",
                        help="保留未着色点并赋灰色 (128,128,128)，而非丢弃（地图更密）")
    parser.add_argument("--prealign", action="store_true",
                        help="先用粗 ICP 把彩色图刚体对齐到优化几何（回环修正有整体漂移时必用）")
    parser.add_argument("--warp-before", default=None,
                        help="优化前关键帧位姿 CSV（keyframe_before.csv）")
    parser.add_argument("--warp-after", default=None,
                        help="优化后关键帧位姿 CSV（keyframe_optimized.csv）")
    parser.add_argument("--warp-max", type=float, default=3.0,
                        help="warp 修正量钳制阈值（米），超过视为优化发散置单位阵")
    parser.add_argument("--sanitize-trajectory", default=None,
                        help="LIO 轨迹 CSV（trajectory_before.csv，x,y,z），裁剪离轨迹过远的几何点")
    parser.add_argument("--sanitize-radius", type=float, default=3.0,
                        help="轨迹裁剪半径（米），默认 3.0")
    args = parser.parse_args()

    t0 = time.time()
    for p in (args.colored, args.geometry):
        if not Path(p).exists():
            print(f"ERROR: 输入文件不存在: {p}", file=sys.stderr)
            sys.exit(1)

    print(f"读取彩色图: {args.colored}")
    crec = read_pcd(args.colored)
    if 'rgb' not in crec.dtype.names:
        print(f"ERROR: 彩色图缺少 rgb 字段: {args.colored}", file=sys.stderr)
        sys.exit(1)
    cxyz = np.stack([crec['x'], crec['y'], crec['z']], axis=-1).astype(np.float64)
    crgb = unpack_rgb(crec['rgb'])
    print(f"  {len(cxyz)} 点")

    print(f"读取几何图: {args.geometry}")
    grec = read_pcd(args.geometry)
    gxyz = np.stack([grec['x'], grec['y'], grec['z']], axis=-1).astype(np.float64)
    print(f"  {len(gxyz)} 点")

    if args.sanitize_trajectory:
        t0_san = time.time()
        try:
            traj = np.loadtxt(args.sanitize_trajectory, delimiter=",")
            if traj.ndim != 2 or traj.shape[1] < 3:
                raise ValueError("轨迹 CSV 格式应为 x,y,z 每行")
            n0 = len(gxyz)
            gxyz, _ = sanitize_near_trajectory(gxyz, traj[:, :3], args.sanitize_radius)
            print(f"轨迹裁剪: 剔除 {n0 - len(gxyz)}/{n0} 点（半径 {args.sanitize_radius}m, "
                  f"{time.time() - t0_san:.1f}s）")
        except (ValueError, OSError) as e:
            print(f"WARNING: 轨迹裁剪失败（{e}），跳过")

    if args.warp_before and args.warp_after:
        t0_warp = time.time()
        try:
            before_poses = load_pose_csv(args.warp_before)
            after_poses = load_pose_csv(args.warp_after)
            if len(before_poses) != len(after_poses):
                print(f"WARNING: 关键帧数量不一致 ({len(before_poses)} vs "
                      f"{len(after_poses)})，按短者对齐")
                n = min(len(before_poses), len(after_poses))
                before_poses, after_poses = before_poses[:n], after_poses[:n]
            max_disp = float(np.linalg.norm(
                after_poses[:, :3, 3] - before_poses[:, :3, 3], axis=1).max())
            cxyz = warp_by_trajectory(cxyz, before_poses, after_poses, args.warp_max)
            print(f"轨迹扭曲: {len(before_poses)} 关键帧, 最大修正量 {max_disp:.3f}m "
                  f"({time.time() - t0_warp:.1f}s)")
        except ValueError as e:
            print(f"WARNING: 轨迹扭曲失败（{e}），按原坐标迁移")

    if args.prealign:
        t0_align = time.time()
        try:
            R, t = prealign(cxyz, gxyz)
            cxyz = (R @ cxyz.T).T + t
            ang = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
            print(f"预对齐: 旋转 {ang:.2f}° 平移 [{t[0]:.3f} {t[1]:.3f} {t[2]:.3f}]m "
                  f"({time.time() - t0_align:.1f}s)")
        except ValueError as e:
            print(f"WARNING: 预对齐失败（{e}），按原坐标迁移")

    kept, rgb = nearest_rgb(cxyz, crgb, gxyz, args.grid, args.threshold,
                            keep_uncolored=args.keep_uncolored)
    write_pcd_rgb(args.output, kept, rgb)
    n_colored = int((rgb != int(pack_rgb(np.array([128], dtype=np.uint32),
                                         np.array([128], dtype=np.uint32),
                                         np.array([128], dtype=np.uint32)).view(np.uint32)[0])).sum())
    mode = "保留全部点" if args.keep_uncolored else f"保留 {len(kept)}/{len(gxyz)} 点"
    print(f"迁移完成: {mode}（其中 {n_colored} 点已着色） → {args.output}")
    print(f"耗时: {time.time() - t0:.1f}s")


if __name__ == "__main__":
    main()
