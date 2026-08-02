#!/usr/bin/env python3
"""odin-imu-allan-analyze - 从 ros2 bag 提取 IMU 数据，算 Allan 方差，
输出可直接填入 odin_livo2.yaml 的 gyr_cov/acc_cov/gyr_bias_cov/acc_bias_cov。

用法:
    python3 odin-imu-allan-analyze.py /tmp/odin_imu_allan_XXXXXX

原理:
    Allan 方差 σ²(τ) 对 tau 做 log-log 曲线拟合：
    - 短 tau 段斜率 -1/2 → 角度/速度随机游走 (ARW/VRW) → 对应 gyr_cov/acc_cov
    - 曲线最低点（拐点）→ bias instability
    - 长 tau 段斜率 +1/2 → rate random walk → 对应 gyr_bias_cov/acc_bias_cov

    本项目 ESIKF 的 cov_w = noise_density^2 * dt^2（见 imu_processing.cpp:297-300），
    noise_density 单位是 "unit per sqrt(Hz)"，与 Allan 方差 ARW 系数（曲线上
    tau=1s 处的值，通常记作 N）直接对应：gyr_cov ≈ N_gyro^2（若配置项按方差存，
    需确认调用处是否已经平方——本项目 set_gyr_cov 直接存进 gyr_cov_，
    在 cov_w 里再乘一次自己，所以这里给的 gyr_cov 值应为 N^2）。
"""
import sys
import subprocess
import json
import math
import numpy as np


def extract_imu_from_bag(bag_dir):
    """用 ros2 bag 的 python API 读取 /odin1/imu，返回 (t, gyro[N,3], acc[N,3])"""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

    storage_options = StorageOptions(uri=bag_dir, storage_id="mcap")
    converter_options = ConverterOptions(input_serialization_format="cdr",
                                          output_serialization_format="cdr")
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    if "/odin1/imu" not in type_map:
        raise RuntimeError(f"/odin1/imu not found in bag. Topics: {list(type_map.keys())}")

    msg_type = get_message(type_map["/odin1/imu"])

    ts, gyro, acc = [], [], []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic != "/odin1/imu":
            continue
        msg = deserialize_message(data, msg_type)
        ts.append(t * 1e-9)
        gyro.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        acc.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

    return np.array(ts), np.array(gyro), np.array(acc)


def allan_variance(data, dt, taus):
    """
    Overlapping Allan variance. data: 1D array of samples at rate 1/dt.
    taus: array of tau values (seconds) to evaluate.
    Returns: (taus_valid, avar) — avar is Allan VARIANCE (not deviation).
    """
    n = len(data)
    # 先算累积和（角度/速度），Allan 方差基于积分量的差分
    theta = np.cumsum(data) * dt  # 累积积分：陀螺→角度，加速度计→速度

    taus_valid = []
    avars = []
    for tau in taus:
        m = int(round(tau / dt))  # 每个 cluster 的样本数
        if m < 1 or m >= n // 2:
            continue
        # overlapping Allan variance:
        # sigma^2(tau) = 1/(2*tau^2*(N-2m)) * sum_{i=1}^{N-2m} (theta[i+2m]-2*theta[i+m]+theta[i])^2
        theta_ip2m = theta[2 * m:]
        theta_ipm = theta[m:-m]
        theta_i = theta[:-2 * m]
        diffs = theta_ip2m - 2 * theta_ipm + theta_i
        if len(diffs) < 2:
            continue
        avar = np.sum(diffs ** 2) / (2.0 * (tau ** 2) * len(diffs))
        taus_valid.append(tau)
        avars.append(avar)

    return np.array(taus_valid), np.array(avars)


def fit_arw_from_avar(taus, avars):
    """
    ARW (angle/velocity random walk) 系数 N：短 tau 段 sigma(tau) = N / sqrt(tau)
    即 log(sigma) = log(N) - 0.5*log(tau)，取斜率最接近 -0.5 的区间线性拟合。
    返回 N（单位：unit / sqrt(Hz)，即本项目 cov 参数直接使用的量级）。
    """
    log_tau = np.log10(taus)
    log_sigma = 0.5 * np.log10(avars)  # sigma = sqrt(avar)

    # 用局部斜率找最接近 -0.5 的窗口（滑动窗口线性回归）
    best_slope_diff = float("inf")
    best_n = None
    window = max(3, len(taus) // 10)
    for i in range(len(taus) - window):
        x = log_tau[i:i + window]
        y = log_sigma[i:i + window]
        slope, intercept = np.polyfit(x, y, 1)
        diff = abs(slope - (-0.5))
        if diff < best_slope_diff:
            best_slope_diff = diff
            # sigma(tau=1) = N  =>  intercept 对应 log10(N) 当 slope=-0.5 exactly
            # 用实际拟合的 intercept 反推 tau=1 处的 sigma 值
            best_n = 10 ** (intercept + slope * 0)  # tau=1 => log_tau=0

    return best_n, best_slope_diff


def fit_bias_instability(taus, avars):
    """Bias instability B: Allan 方差曲线最低点对应 sigma_min = B * sqrt(2*ln(2)/pi) ≈ 0.664*B"""
    sigma = np.sqrt(avars)
    idx_min = np.argmin(sigma)
    sigma_min = sigma[idx_min]
    tau_min = taus[idx_min]
    b = sigma_min / 0.664
    return b, tau_min


def fit_rrw_from_avar(taus, avars):
    """Rate random walk 系数 K：长 tau 段 sigma(tau) = K * sqrt(tau/3)
    即 log(sigma) = log(K) - 0.5*log(3) + 0.5*log(tau)，取斜率最接近 +0.5 的区间线性拟合。
    返回 K（单位：unit/sqrt(s)，rate random walk 系数，cov = K²）。
    """
    log_tau = np.log10(taus)
    log_sigma = 0.5 * np.log10(avars)

    best_slope_diff = float("inf")
    best_k = None
    window = max(3, len(taus) // 10)
    for i in range(len(taus) - window):
        x = log_tau[i:i + window]
        y = log_sigma[i:i + window]
        slope, intercept = np.polyfit(x, y, 1)
        diff = abs(slope - 0.5)
        if diff < best_slope_diff:
            best_slope_diff = diff
            # sigma(tau=3) = K  → intercept = log10(K) when slope=+0.5
            best_k = 10 ** intercept  # tau=1 => log_tau=0, sigma(1) = K / sqrt(3)
            # correct: sigma(1) = K/sqrt(3), so K = sigma(1)*sqrt(3) = 10^intercept * sqrt(3)
            best_k *= math.sqrt(3)

    return best_k, best_slope_diff


def analyze_axis(data, dt, label):
    n = len(data)
    max_tau = dt * n / 10
    taus = np.logspace(math.log10(dt * 2), math.log10(max_tau), 50)

    taus_valid, avars = allan_variance(data, dt, taus)
    if len(taus_valid) < 5:
        print(f"    ⚠ {label}: 数据太短，无法算出有效 Allan 方差曲线")
        return None, None, None

    n_coef, slope_diff = fit_arw_from_avar(taus_valid, avars)
    b_coef, tau_min = fit_bias_instability(taus_valid, avars)
    k_coef, rrw_slope_diff = fit_rrw_from_avar(taus_valid, avars)

    print(f"    {label}: ARW/VRW(N)={n_coef:.6f}  斜率误差={slope_diff:.3f}  "
          f"bias不稳定性(B)={b_coef:.6f}@τ={tau_min:.1f}s  "
          f"RRW(K)={k_coef:.6f}  斜率误差={rrw_slope_diff:.3f}")
    return n_coef, b_coef, k_coef


def main():
    if len(sys.argv) < 2:
        print("用法: odin-imu-allan-analyze.py <bag_dir>")
        sys.exit(1)

    bag_dir = sys.argv[1]
    print(f"读取 bag: {bag_dir} ...")
    ts, gyro, acc = extract_imu_from_bag(bag_dir)

    if len(ts) < 100:
        print(f"✗ 数据点太少 ({len(ts)})，检查 IMU 是否正常录制")
        sys.exit(1)

    dt = float(np.median(np.diff(ts)))
    duration = ts[-1] - ts[0]
    print(f"采样点数: {len(ts)}  平均dt: {dt*1000:.2f}ms ({1/dt:.1f}Hz)  总时长: {duration/60:.1f}分钟")
    print("")

    if duration < 300:
        print(f"⚠ 警告: 数据时长仅 {duration/60:.1f} 分钟，Allan 方差长 tau 段（bias instability）")
        print(f"  可能不准确。建议采集 30-60 分钟数据用于正式标定，当前结果仅供快速参考。")
        print("")

    print("── 陀螺仪三轴 Allan 方差分析 ──")
    gyr_n, gyr_b, gyr_k = [], [], []
    for i, axis in enumerate(["x", "y", "z"]):
        n, b, k = analyze_axis(gyro[:, i], dt, f"gyro_{axis}")
        if n is not None:
            gyr_n.append(n)
            gyr_b.append(b)
            gyr_k.append(k)

    print("")
    print("── 加速度计三轴 Allan 方差分析 ──")
    acc_n, acc_b, acc_k = [], [], []
    for i, axis in enumerate(["x", "y", "z"]):
        n, b, k = analyze_axis(acc[:, i], dt, f"acc_{axis}")
        if n is not None:
            acc_n.append(n)
            acc_b.append(b)
            acc_k.append(k)

    if not gyr_n or not acc_n:
        print("✗ Allan 方差分析失败，数据不足或格式不对")
        sys.exit(1)

    gyr_n_avg = float(np.mean(gyr_n))
    gyr_b_avg = float(np.mean(gyr_b))
    gyr_k_avg = float(np.mean(gyr_k))
    acc_n_avg = float(np.mean(acc_n))
    acc_b_avg = float(np.mean(acc_b))
    acc_k_avg = float(np.mean(acc_k))

    gyr_cov_val = gyr_n_avg ** 2
    acc_cov_val = acc_n_avg ** 2
    gyr_bias_cov_val = gyr_k_avg ** 2
    acc_bias_cov_val = acc_k_avg ** 2

    print("")
    print("═══════════════════════════════════════════════════")
    print("  Allan 方差标定结果 → odin_livo2.yaml 建议值")
    print("═══════════════════════════════════════════════════")
    print(f"  gyr_cov:      {gyr_cov_val:.8f}   (ARW N={gyr_n_avg:.6f} rad/s/√Hz)")
    print(f"  acc_cov:      {acc_cov_val:.8f}   (VRW N={acc_n_avg:.6f} m/s²/√Hz)")
    print(f"  gyr_bias_cov: {gyr_bias_cov_val:.10f}   (RRW K={gyr_k_avg:.6f} rad/s/√s, B={gyr_b_avg:.8f} rad/s)")
    print(f"  acc_bias_cov: {acc_bias_cov_val:.10f}   (RRW K={acc_k_avg:.6f} m/s²/√s, B={acc_b_avg:.8f} m/s²)")
    print("═══════════════════════════════════════════════════")
    print("")
    print("  gyr_cov/acc_cov: ARW/VRW 白噪声方差密度 (N²)，对应传感器测量噪声")
    print("  gyr_bias_cov/acc_bias_cov: RRW 系数方差 (K²)，对应 bias 随机游走过程噪声")
    print("  bias_cov 量级对 ESIKF 收敛速度影响很大，建议以此结果为初始值")
    print("  再结合实际 LIO 运行效果做 ±2-5x 范围微调。")
    print("  完整曲线数据建议另存 CSV 做 log-log 图目视检查拐点是否合理。")

    result = {
        "gyr_cov": gyr_cov_val,
        "acc_cov": acc_cov_val,
        "gyr_bias_cov": gyr_bias_cov_val,
        "acc_bias_cov": acc_bias_cov_val,
        "gyr_arw_n": gyr_n_avg,
        "acc_vrw_n": acc_n_avg,
        "gyr_bias_b": gyr_b_avg,
        "acc_bias_b": acc_b_avg,
        "gyr_rrw_k": gyr_k_avg,
        "acc_rrw_k": acc_k_avg,
        "duration_sec": duration,
        "sample_rate_hz": 1.0 / dt,
    }
    out_path = bag_dir.rstrip("/") + "_allan_result.json"
    with open(out_path, "w") as f:
        json.dump(result, f, indent=2)
    print(f"\n  结果已保存: {out_path}")


if __name__ == "__main__":
    main()
