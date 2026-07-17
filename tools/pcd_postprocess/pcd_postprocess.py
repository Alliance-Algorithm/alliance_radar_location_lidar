#!/usr/bin/env python3
"""PCD 后处理：voxel 降采样去重 + 半径离群点剔除，导出干净的静态地图

odin-map-save 导出的是原始累积点云——每帧点云原样追加，同一块静止表面
被扫过几百帧就重复几百次，且含运动畸变/边缘噪声产生的离群点，不能直接
当"地图"用。本工具做两步后处理：

    1. voxel 降采样去重（复用 tools/bag_to_pcd 的哈希网格实现）
    2. 半径离群点剔除：以粗一档的体素网格统计每点 27 邻域（自身+26个
       相邻格子）内的总点数，低于阈值的判定为离群点剔除

不依赖 PCL/open3d/scipy（无 KDTree），纯 numpy 实现，用网格近邻计数
近似半径搜索——对于地图这种点云密度分布相对均匀的场景足够用，换来的是
不需要额外依赖、可在 Odin1 主机上直接跑。

用法:
    python3 pcd_postprocess.py <input.pcd> [--output map_clean.pcd]
        [--voxel-size 0.05] [--outlier-radius 0.2] [--outlier-min-neighbors 5]
        [--no-outlier-removal] [--ascii]

    # 只降采样，不做离群点剔除
    python3 pcd_postprocess.py raw_map.pcd --no-outlier-removal

只处理 COUNT=1 的字段（本项目内所有 PCD 生产者——bag_to_pcd.py /
livmapper_node.cpp 的 pcl::PointXYZINormal ——均满足）。
"""
import sys
import time
import argparse
from pathlib import Path
import numpy as np

# PCD TYPE+SIZE → numpy dtype（本项目所有生产者都只用 4 字节字段）
_PCD_DTYPE_MAP = {
    ('F', 4): '<f4', ('F', 8): '<f8',
    ('U', 1): 'u1', ('U', 2): '<u2', ('U', 4): '<u4', ('U', 8): '<u8',
    ('I', 1): 'i1', ('I', 2): '<i2', ('I', 4): '<i4', ('I', 8): '<i8',
}


def read_pcd(path):
    """读取 PCD 文件（ASCII 或 binary），返回 (xyz, extra_fields_dict)。

    extra_fields_dict: 除 x/y/z 外的其它字段名 -> 1D numpy 数组
    （如 intensity/normal_x/normal_y/normal_z/curvature，若存在）。
    """
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
                raise ValueError(f"PCD 头部异常（超过 50 行仍未找到 DATA）: {path}")
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
            raise ValueError(f"不支持的 DATA 模式: {data_mode}（只支持 ascii/binary）")

    xyz = np.stack([records['x'], records['y'], records['z']], axis=-1).astype(np.float64)
    extra = {name: records[name] for name in records.dtype.names if name not in ('x', 'y', 'z')}
    return xyz, extra


def voxel_downsample_with_extra(xyz, extra, leaf_size):
    """voxel 降采样去重，同步保留被选中点对应的 extra 字段值。"""
    if leaf_size <= 0:
        return xyz, extra

    voxel_idx = np.floor(xyz / leaf_size).astype(np.int64)
    # 用 np.unique(axis=0) 直接按 (vx, vy, vz) 去重，避免 XOR 哈希碰撞
    # （整数溢出/取模可能导致不同网格映射到同一 key）
    _, first_idx = np.unique(voxel_idx, axis=0, return_index=True)
    first_idx = np.sort(first_idx)

    xyz_ds = xyz[first_idx]
    extra_ds = {name: arr[first_idx] for name, arr in extra.items()}
    return xyz_ds, extra_ds


def radius_outlier_removal(xyz, extra, radius, min_neighbors):
    """网格近邻计数近似半径离群点剔除：以 radius 为格子边长分网格，
    每点的邻域点数 = 自身格子 + 26 个相邻格子内的总点数，低于
    min_neighbors 的点判定为离群点并剔除。

    用 lexsort + searchsorted 查表实现邻域计数，避免 XOR 哈希碰撞，
    不需要 KDTree，可以处理百万级点云。
    """
    if radius <= 0 or min_neighbors <= 0:
        return xyz, extra, 0

    cell_idx = np.floor(xyz / radius).astype(np.int64)
    n = len(xyz)

    # 用 lexsort 代替 XOR hash，避免不同网格映射到同一 key
    sort_idx = np.lexsort((cell_idx[:, 2], cell_idx[:, 1], cell_idx[:, 0]))
    sorted_cells = cell_idx[sort_idx]
    # 找出每个 grid cell 的起止位置（边界检测：与上一行不同即新 cell 起点）
    boundaries = np.concatenate([[0], 1 + np.where(
        (sorted_cells[1:, 0] != sorted_cells[:-1, 0]) |
        (sorted_cells[1:, 1] != sorted_cells[:-1, 1]) |
        (sorted_cells[1:, 2] != sorted_cells[:-1, 2])
    )[0], [n]])
    # cell_key → (start, end) 映射：用每个 cell 的第一个点做 key
    cell_starts = {tuple(sorted_cells[boundaries[i]]): (boundaries[i], boundaries[i+1])
                   for i in range(len(boundaries) - 1)}

    def lookup_count(query_cells):
        result = np.zeros(len(query_cells), dtype=np.int64)
        for i in range(len(query_cells)):
            key = (query_cells[i, 0], query_cells[i, 1], query_cells[i, 2])
            se = cell_starts.get(key)
            if se is not None:
                result[i] = se[1] - se[0]
        return result

    neighbor_count = np.zeros(n, dtype=np.int64)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                shifted = cell_idx + np.array([dx, dy, dz])
                neighbor_count += lookup_count(shifted)

    keep_mask = neighbor_count >= min_neighbors
    n_removed = int((~keep_mask).sum())
    xyz_clean = xyz[keep_mask]
    extra_clean = {name: arr[keep_mask] for name, arr in extra.items()}
    return xyz_clean, extra_clean, n_removed


def write_pcd(path, xyz, intensity=None, ascii_mode=False):
    """写入 PCD 文件，支持 xyz / xyzi，ascii 或 binary 格式。"""
    n = len(xyz)
    has_i = intensity is not None

    fields_line = "FIELDS x y z" + (" intensity" if has_i else "")
    size_line = "SIZE 4 4 4" + (" 4" if has_i else "")
    type_line = "TYPE F F F" + (" F" if has_i else "")
    count_line = "COUNT 1 1 1" + (" 1" if has_i else "")

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
            for i in range(n):
                if has_i:
                    f.write(f"{xyz[i, 0]:.6f} {xyz[i, 1]:.6f} {xyz[i, 2]:.6f} {intensity[i]:.6f}\n")
                else:
                    f.write(f"{xyz[i, 0]:.6f} {xyz[i, 1]:.6f} {xyz[i, 2]:.6f}\n")
        return

    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(b"DATA binary\n")
        if has_i:
            record = np.empty(n, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4')])
            record['intensity'] = intensity
        else:
            record = np.empty(n, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
        record['x'] = xyz[:, 0]
        record['y'] = xyz[:, 1]
        record['z'] = xyz[:, 2]
        f.write(record.tobytes())


def main():
    parser = argparse.ArgumentParser(description="PCD 后处理：降采样去重 + 离群点剔除")
    parser.add_argument("input", help="输入 PCD 文件路径（如 odin-map-save 导出的原始地图）")
    parser.add_argument("--output", default=None,
                         help="输出路径（默认 <input>_clean.pcd）")
    parser.add_argument("--voxel-size", type=float, default=0.05,
                         help="voxel 降采样叶大小，米（默认 0.05）")
    parser.add_argument("--no-downsample", action="store_true", help="跳过降采样步骤")
    parser.add_argument("--outlier-radius", type=float, default=0.2,
                         help="离群点剔除的邻域格子边长，米（默认 0.2，建议 >= voxel-size）")
    parser.add_argument("--outlier-min-neighbors", type=int, default=5,
                         help="27 邻域内最少点数，低于此值判定为离群点（默认 5）")
    parser.add_argument("--no-outlier-removal", action="store_true", help="跳过离群点剔除步骤")
    parser.add_argument("--ascii", action="store_true", help="输出 ASCII PCD（默认 binary）")
    args = parser.parse_args()

    t_start = time.time()
    input_path = Path(args.input)
    if not input_path.exists():
        print(f"ERROR: 输入文件不存在: {input_path}", file=sys.stderr)
        sys.exit(1)

    output_path = Path(args.output) if args.output else input_path.with_name(
        input_path.stem + "_clean" + input_path.suffix)

    print(f"读取: {input_path}")
    xyz, extra = read_pcd(input_path)
    print(f"原始点数: {len(xyz)}")
    if len(xyz) == 0:
        print("ERROR: 点云为空", file=sys.stderr)
        sys.exit(1)

    if not args.no_downsample:
        xyz, extra = voxel_downsample_with_extra(xyz, extra, args.voxel_size)
        print(f"降采样后 ({args.voxel_size}m): {len(xyz)} 点")

    if not args.no_outlier_removal:
        xyz, extra, n_removed = radius_outlier_removal(
            xyz, extra, args.outlier_radius, args.outlier_min_neighbors)
        print(f"离群点剔除 (半径={args.outlier_radius}m, 最少邻居={args.outlier_min_neighbors}): "
              f"剔除 {n_removed} 点，剩余 {len(xyz)} 点")

    if len(xyz) == 0:
        print("ERROR: 后处理后点云为空（参数可能过于激进）", file=sys.stderr)
        sys.exit(1)

    intensity = extra.get('intensity')
    bbox_min = xyz.min(axis=0)
    bbox_max = xyz.max(axis=0)
    print(f"输出点云 bounding box: min={bbox_min}, max={bbox_max}")

    write_pcd(output_path, xyz, intensity, ascii_mode=args.ascii)
    print(f"输出: {output_path}")
    print(f"耗时: {time.time() - t_start:.1f}s")


if __name__ == "__main__":
    main()
