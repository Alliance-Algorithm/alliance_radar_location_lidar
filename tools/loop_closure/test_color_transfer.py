#!/usr/bin/env python3
"""color_transfer.py 单元测试（python3 直接运行，零依赖）。

运行: python3 tools/loop_closure/test_color_transfer.py
"""
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import color_transfer as ct  # noqa: E402

FAILURES = []


def check(name, cond):
    print(("PASS " if cond else "FAIL ") + name)
    if not cond:
        FAILURES.append(name)


def make_colored_plane(grid=0.05):
    """x,y ∈ [0,1) 步长 grid 的平面，z=0；rgb 由 (x,y) 决定。"""
    xs = np.arange(0, 1.0, grid)
    ys = np.arange(0, 1.0, grid)
    xx, yy = np.meshgrid(xs, ys)
    xyz = np.stack([xx.ravel(), yy.ravel(), np.zeros(xx.size)], axis=-1)
    r = np.round(xx.ravel() * 255).astype(np.uint32)
    g = np.round(yy.ravel() * 255).astype(np.uint32)
    b = np.zeros(xx.size, np.uint32)
    rgb = ct.pack_rgb(r, g, b).view(np.uint32)
    return xyz, rgb


def test_nearest_transfer():
    cxyz, crgb = make_colored_plane()
    gxyz = cxyz + np.array([0.02, 0.0, 0.0])  # 位移 0.02m < 0.05 网格半宽
    kept, rgb = ct.nearest_rgb(cxyz, crgb, gxyz, grid=0.05, threshold=0.15)
    check("all_geometry_kept", len(kept) == len(gxyz))
    check("color_exact_match", np.array_equal(rgb, crgb))
    check("positions_unchanged", np.allclose(kept, gxyz))


def test_threshold_drop():
    cxyz, crgb = make_colored_plane()
    gxyz = cxyz + np.array([0.02, 0.0, 0.0])
    kept, rgb = ct.nearest_rgb(cxyz, crgb, gxyz, grid=0.05, threshold=0.01)
    check("all_dropped_below_threshold", len(kept) == 0)


def test_far_outlier_dropped():
    cxyz, crgb = make_colored_plane()
    far = np.array([[5.0, 5.0, 5.0], [5.1, 5.1, 5.1]])
    kept, rgb = ct.nearest_rgb(cxyz, crgb, far, grid=0.05, threshold=0.15)
    check("far_points_dropped", len(kept) == 0)


def test_roundtrip_file():
    cxyz, crgb = make_colored_plane()
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "c.pcd"
        ct.write_pcd_rgb(str(p), cxyz, crgb)
        rec = ct.read_pcd(str(p))
        check("points_roundtrip", len(rec) == len(cxyz))
        check("xyz_roundtrip", np.allclose(
            np.stack([rec['x'], rec['y'], rec['z']], axis=-1), cxyz))
        check("rgb_roundtrip", np.array_equal(
            rec['rgb'].view(np.uint32), crgb))


def test_keep_uncolored_gray():
    cxyz, crgb = make_colored_plane()
    far = np.array([[5.0, 5.0, 5.0], [5.1, 5.1, 5.1]])
    kept, rgb = ct.nearest_rgb(cxyz, crgb, far, grid=0.05, threshold=0.15,
                               keep_uncolored=True)
    check("keep_uncolored_keeps_all", len(kept) == 2)
    gray = int(ct.pack_rgb(np.array([128], dtype=np.uint32),
                           np.array([128], dtype=np.uint32),
                           np.array([128], dtype=np.uint32)).view(np.uint32)[0])
    check("keep_uncolored_gray_value", bool((rgb == gray).all()))


def test_prealign_recovers_transform():
    rng = np.random.default_rng(7)
    # 几何：两个平面（地面 + 墙面），模拟场景
    xy = rng.uniform(-5, 5, (3000, 2))
    floor = np.stack([xy[:, 0], xy[:, 1], np.zeros(3000)], axis=-1)
    wall = np.stack([xy[:, 0], np.full(3000, 3.0), xy[:, 1]], axis=-1)
    tgt = np.vstack([floor, wall])
    # 彩色图 = 几何 + 已知刚体变换（1.9m 平移 + 2° 旋转）+ 噪声
    ang = np.radians(2.0)
    R_true = np.array([[np.cos(ang), -np.sin(ang), 0],
                       [np.sin(ang), np.cos(ang), 0],
                       [0, 0, 1.0]])
    t_true = np.array([1.9, 0.3, -0.2])
    src = (R_true @ tgt.T).T + t_true + rng.normal(0, 0.02, tgt.shape)
    R, t = ct.prealign(src, tgt)
    # 变换回几何坐标系后应与 target 对齐（残差 << 1.9m）
    aligned = (R @ src.T).T + t
    nn = np.zeros(len(aligned))
    for i in range(len(aligned)):
        nn[i] = ((tgt - aligned[i]) ** 2).sum(axis=1).min()
    med = float(np.median(np.sqrt(nn)))
    check("prealign_residual_small", med < 0.3)
    # 旋转精度只要求粗对齐级别（主力对齐机制是 warp_by_trajectory）
    ang_err = float(np.degrees(np.arccos(np.clip((np.trace(R @ R_true.T) - 1) / 2, -1, 1))))
    check("prealign_rotation_coarse", ang_err < 6.0)


def make_pose(x, y, z):
    T = np.eye(4)
    T[0, 3], T[1, 3], T[2, 3] = x, y, z
    return T


def test_warp_by_trajectory():
    # 模拟：LIO 轨迹沿 x 走 40m，z 方向漂移累积 2m；优化轨迹无漂移
    n_kf = 41
    before = np.array([make_pose(i, 0, 0.05 * i) for i in range(n_kf)])
    after = np.array([make_pose(i, 0, 0.0) for i in range(n_kf)])
    # 优化几何：墙点沿 x 排列，z=0
    gxyz = np.stack([np.arange(0, 40, 0.5), np.ones(80), np.zeros(80)], axis=-1)
    # 彩色图（LIO 帧）：几何点按各自位置对应的修正量反变换（漂移帧下偏置）
    xs = gxyz[:, 0]
    kf_idx = np.round(xs).astype(int)
    drift = 0.05 * kf_idx
    cxyz = gxyz + np.stack([np.zeros(80), np.zeros(80), drift], axis=-1)
    crgb = np.full(80, 0xFF0000, dtype=np.uint32)
    warped = ct.warp_by_trajectory(cxyz, before, after)
    resid = float(np.abs(warped[:, 2] - gxyz[:, 2]).max())
    check("warp_recovers_z", resid < 0.02)
    # 扭曲后颜色迁移覆盖率应接近 100%；不扭曲时（阈值 0.15）大量失配
    kept_warped, _ = ct.nearest_rgb(warped, crgb, gxyz, 0.05, 0.15)
    kept_raw, _ = ct.nearest_rgb(cxyz, crgb, gxyz, 0.05, 0.15)
    check("warp_high_coverage", len(kept_warped) > 0.9 * len(gxyz))
    check("raw_low_coverage", len(kept_raw) < 0.5 * len(gxyz))


def main():
    test_nearest_transfer()
    test_threshold_drop()
    test_far_outlier_dropped()
    test_roundtrip_file()
    test_keep_uncolored_gray()
    test_prealign_recovers_transform()
    test_warp_by_trajectory()
    print()
    if FAILURES:
        print(f"FAILED: {len(FAILURES)} check(s): {FAILURES}")
        sys.exit(1)
    print("ALL PASS")


if __name__ == "__main__":
    main()
