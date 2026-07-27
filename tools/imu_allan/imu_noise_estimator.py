#!/usr/bin/env python3
"""
IMU Noise Parameter Estimator via Allan Deviation

Estimates gyro/accel noise parameters from stationary IMU data
(ROS2 bag, CSV, or raw numpy) using Allan deviation analysis.

Parameters estimated:
  - gyr_cov / acc_cov : Angle/Velocity Random Walk (ARW/VRW)
  - gyr_bias_cov / acc_bias_cov : Bias Instability

Usage:
  # From ROS2 bag
  python imu_noise_estimator.py --bag path/to/imu.bag --topic /odin1/imu --duration 60

  # From CSV (timestamp, gx, gy, gz, ax, ay, az)
  python imu_noise_estimator.py --csv path/to/imu.csv --rate 400

Output:
  - Console: estimated noise parameters (ready for YAML)
  - imu_allan.png: Allan deviation plots
"""

import argparse
import sys
import os
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Allan deviation (no external dependency — self-contained)
# ---------------------------------------------------------------------------

def allan_deviation(data: np.ndarray, dt: float, max_tau_bins: int = 200) -> tuple[np.ndarray, np.ndarray]:
    """
    Compute overlapping Allan deviation.

    Args:
        data: 1-D array of integrated-rate samples (e.g. gyro [rad/s], accel [m/s²])
        dt:   sample interval in seconds
        max_tau_bins: number of tau clusters

    Returns:
        tau_values: averaging times (seconds)
        ad:         Allan deviation
    """
    n = len(data)
    # Cluster spacing: geometrically from 1 sample to ~10% of total
    tau_max = n // 9
    tau_values = np.unique(
        np.logspace(0, np.log10(tau_max), max_tau_bins, dtype=int)
    )
    # Keep only feasible taus
    tau_values = tau_values[tau_values >= 1]
    tau_values = tau_values[tau_values <= tau_max]

    ad = np.zeros(len(tau_values))
    for k, tau in enumerate(tau_values):
        m = n // tau
        if m < 2:
            ad[k] = np.nan
            continue
        clusters = data[: m * tau].reshape(m, tau)
        means = clusters.mean(axis=1)  # m length
        diffs = means[1:] - means[:-1]
        if len(diffs) < 1:
            ad[k] = np.nan
        else:
            ad[k] = np.sqrt(0.5 * np.mean(diffs**2))
    valid = ~np.isnan(ad)
    return tau_values[valid] * dt, ad[valid]


def fit_line(log_tau: np.ndarray, log_ad: np.ndarray, mask: np.ndarray) -> tuple[float, float]:
    """Fit slope and intercept to log-log Allan deviation in masked region."""
    x, y = log_tau[mask], log_ad[mask]
    if len(x) < 3:
        return np.nan, np.nan
    A = np.column_stack([x, np.ones_like(x)])
    slope, intercept = np.linalg.lstsq(A, y, rcond=None)[0]
    return slope, intercept


def estimate_parameters(tau: np.ndarray, ad: np.ndarray, dt: float) -> dict:
    """
    Estimate ARW/VRW and Bias Instability from Allan deviation.

    ARW/VRW → slope ≈ -1/2 region (read at τ=1s if available)
    Bias Inst. → slope ≈ 0 region (minimum of AD)
    """
    log_tau = np.log10(tau)
    log_ad = np.log10(ad)

    # ── Bias Instability: minimum of the flat region ──
    # Identify where slope is closest to 0
    slopes = np.zeros(len(tau) - 2)
    for i in range(1, len(tau) - 1):
        slopes[i - 1] = (log_ad[i + 1] - log_ad[i - 1]) / (log_tau[i + 1] - log_tau[i - 1])
    # Find region where |slope| < threshold
    flat_mask = np.zeros(len(tau), dtype=bool)
    flat_mask[1:-1] = np.abs(slopes) < 0.3
    if np.any(flat_mask):
        bias_instability = np.min(ad[flat_mask])
        bias_tau = tau[flat_mask][np.argmin(ad[flat_mask])]
    else:
        bias_instability = np.min(ad)
        bias_tau = tau[np.argmin(ad)]

    # ── ARW/VRW: extrapolate from -1/2 slope region to τ=1s ──
    nw_mask = np.zeros(len(tau), dtype=bool)
    # Find the first 1/3 of tau range where slope is near -1/2
    third = len(tau) // 3
    if third > 0:
        early_slopes = np.zeros(third - 2)
        for i in range(1, third - 1):
            early_slopes[i - 1] = (log_ad[i + 1] - log_ad[i - 1]) / (log_tau[i + 1] - log_tau[i - 1])
        nw_idx = np.where(np.abs(early_slopes + 0.5) < 0.25)[0]
        if len(nw_idx) > 0:
            nw_start = nw_idx[0] + 1
            nw_end = nw_idx[-1] + 2
            nw_mask[nw_start:nw_end] = True

    if np.sum(nw_mask) >= 3:
        slope, intercept = fit_line(log_tau, log_ad, nw_mask)
        # ARW at τ=1s
        arw = 10.0 ** (slope * 0.0 + intercept)  # log_tau=0 → τ=1s
    else:
        # Fallback: use first few points
        n_first = max(3, min(10, len(tau) // 10))
        slope, intercept = fit_line(log_tau, log_ad, np.arange(n_first))
        arw = 10.0 ** intercept

    return {
        "arw": arw,
        "bias_instability": bias_instability,
        "bias_tau": bias_tau,
        "dt": dt,
    }


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_allan(gyro_params: dict, accel_params: dict, tau: np.ndarray,
               ad_gyro: list, ad_accel: list, output: str):
    """Generate 2x3 Allan deviation plot."""
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    labels = ["X", "Y", "Z"]

    for i in range(3):
        ax = axes[0, i]
        tau_g, ad_g = tau[i], ad_gyro[i]
        ax.loglog(tau_g, ad_g, "b.-", markersize=3, label="Gyro")
        # ARW reference line
        arw = gyro_params["arw"][i]
        if not np.isnan(arw):
            ref_tau = np.array([min(tau_g), max(tau_g)])
            ax.loglog(ref_tau, arw / np.sqrt(ref_tau), "b--", alpha=0.5, label=f"ARW={arw:.2e}")
        # Bias instability
        bi = gyro_params["bias_instability"][i]
        ax.axhline(bi, color="red", linestyle=":", alpha=0.7, label=f"BI={bi:.2e}")
        ax.set_title(f"Gyro {labels[i]}")
        ax.set_xlabel("τ (s)")
        ax.set_ylabel("Allan Deviation (rad/s)")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    for i in range(3):
        ax = axes[1, i]
        tau_a, ad_a = tau[i], ad_accel[i]
        ax.loglog(tau_a, ad_a, "g.-", markersize=3, label="Accel")
        arw = accel_params["arw"][i]
        if not np.isnan(arw):
            ref_tau = np.array([min(tau_a), max(tau_a)])
            ax.loglog(ref_tau, arw / np.sqrt(ref_tau), "g--", alpha=0.5, label=f"VRW={arw:.2e}")
        bi = accel_params["bias_instability"][i]
        ax.axhline(bi, color="red", linestyle=":", alpha=0.7, label=f"BI={bi:.2e}")
        ax.set_title(f"Accel {labels[i]}")
        ax.set_xlabel("τ (s)")
        ax.set_ylabel("Allan Deviation (m/s²)")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    fig.suptitle("IMU Allan Deviation Analysis", fontsize=14)
    fig.tight_layout()
    fig.savefig(output, dpi=150)
    print(f"Saved plot: {output}")


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def load_from_csv(path: str, rate: float) -> tuple[np.ndarray, np.ndarray, float]:
    """Load IMU data from CSV: timestamp(s), gx, gy, gz, ax, ay, az"""
    data = np.loadtxt(path, delimiter=",")
    if data.ndim == 1:
        data = data.reshape(1, -1)
    ncols = data.shape[1]
    if ncols >= 7:
        gyro = data[:, 1:4]
        accel = data[:, 4:7]
    elif ncols == 6:
        gyro = data[:, 0:3]
        accel = data[:, 3:6]
    else:
        raise ValueError(f"Expected 6 or 7 columns, got {ncols}")
    dt = 1.0 / rate
    return gyro, accel, dt


def load_from_bag(path: str, topic: str, duration: float | None) -> tuple[np.ndarray, np.ndarray, float]:
    """Load IMU data from ROS2 bag using rosbag2_py."""
    try:
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Imu
    except ImportError:
        print("ERROR: rosbag2_py not available. Install: sudo apt install ros-jazzy-rosbag2")
        sys.exit(1)

    reader = SequentialReader()
    reader.open(StorageOptions(uri=path, storage_id="mcap"),
                ConverterOptions(input_serialization_format="cdr",
                                 output_serialization_format="cdr"))

    topic_types = reader.get_all_topics_and_types()
    topic_found = any(t.name == topic for t in topic_types)
    if not topic_found:
        available = [t.name for t in topic_types]
        print(f"ERROR: Topic '{topic}' not found. Available: {available}")
        sys.exit(1)

    gyro_list, accel_list, stamp_list = [], [], []
    start_time = None
    while reader.has_next():
        topic_name, data, stamp_ns = reader.read_next()
        if topic_name != topic:
            continue
        msg = deserialize_message(data, Imu)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if start_time is None:
            start_time = t
        if duration and t - start_time > duration:
            break
        stamp_list.append(t)
        gyro_list.append([msg.angular_velocity.x,
                          msg.angular_velocity.y,
                          msg.angular_velocity.z])
        accel_list.append([msg.linear_acceleration.x,
                           msg.linear_acceleration.y,
                           msg.linear_acceleration.z])

    gyro = np.array(gyro_list)
    accel = np.array(accel_list)
    stamps = np.array(stamp_list)

    # Estimate sample rate from inter-sample intervals
    if len(stamps) > 1:
        dt = np.median(np.diff(stamps))
    else:
        dt = 0.0025  # default 400Hz
    print(f"Loaded {len(gyro)} IMU samples from bag, dt≈{dt*1000:.2f}ms")
    return gyro, accel, dt


# ---------------------------------------------------------------------------
# Generating test data (square-wave modulation for demonstration)
# ---------------------------------------------------------------------------

def generate_test_data(duration_s: float = 60.0, rate: float = 400.0):
    """Generate synthetic IMU data with known noise parameters (for testing)."""
    dt = 1.0 / rate
    n = int(duration_s * rate)

    # Known parameters for synthetic data
    gyr_arw = 0.005   # rad/s/√Hz
    acc_vrw = 0.005   # m/s²/√Hz
    gyr_bias = 0.0001  # rad/s
    acc_bias = 0.0001  # m/s²

    rng = np.random.default_rng(42)
    gyro = np.zeros((n, 3))
    accel = np.zeros((n, 3))

    # White noise + bias random walk
    bias_g = np.zeros(3)
    bias_a = np.zeros(3)
    for i in range(n):
        # White noise component
        w_g = rng.normal(0, gyr_arw / np.sqrt(dt), 3)
        w_a = rng.normal(0, acc_vrw / np.sqrt(dt), 3)
        # Bias random walk
        bias_g += rng.normal(0, gyr_bias * np.sqrt(dt), 3)
        bias_a += rng.normal(0, acc_bias * np.sqrt(dt), 3)
        gyro[i] = w_g + bias_g
        accel[i] = w_a + bias_a

    return gyro, accel, dt


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="IMU Noise Parameter Estimator (Allan Deviation)")
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument("--bag", type=str, help="ROS2 bag file path (.mcap)")
    src.add_argument("--csv", type=str, help="CSV file: ts,gx,gy,gz,ax,ay,az (1 row per sample)")
    src.add_argument("--test", action="store_true", help="Generate synthetic test data")
    parser.add_argument("--topic", type=str, default="/odin1/imu", help="IMU topic name (bag mode)")
    parser.add_argument("--rate", type=float, default=400.0, help="IMU sample rate in Hz (csv mode)")
    parser.add_argument("--duration", type=float, default=None, help="Max seconds to read from bag")
    parser.add_argument("--output", type=str, default="imu_allan.png", help="Plot output path")
    args = parser.parse_args()

    # ── Load data ──
    if args.test:
        print("Generating synthetic test data (60s @ 400Hz)...")
        gyro, accel, dt = generate_test_data()
    elif args.bag:
        if not os.path.exists(args.bag):
            print(f"ERROR: Bag file not found: {args.bag}")
            sys.exit(1)
        gyro, accel, dt = load_from_bag(args.bag, args.topic, args.duration)
    elif args.csv:
        if not os.path.exists(args.csv):
            print(f"ERROR: CSV file not found: {args.csv}")
            sys.exit(1)
        gyro, accel, dt = load_from_csv(args.csv, args.rate)

    print(f"Estimating noise parameters from {len(gyro)} samples, dt={dt*1000:.2f}ms "
          f"(duration={len(gyro)*dt:.1f}s)")

    # ── Compute Allan deviation per axis ──
    tau_list, ad_gyro, ad_accel = [], [], []
    gyro_params = {"arw": [], "bias_instability": [], "bias_tau": [], "dt": dt}
    accel_params = {"arw": [], "bias_instability": [], "bias_tau": [], "dt": dt}

    for axis in range(3):
        tau_g, ad_g = allan_deviation(gyro[:, axis], dt)
        tau_a, ad_a = allan_deviation(accel[:, axis], dt)

        gp = estimate_parameters(tau_g, ad_g, dt)
        ap = estimate_parameters(tau_a, ad_a, dt)

        tau_list.append(tau_g)
        ad_gyro.append(ad_g)
        ad_accel.append(ad_a)

        gyro_params["arw"].append(gp["arw"])
        gyro_params["bias_instability"].append(gp["bias_instability"])
        gyro_params["bias_tau"].append(gp["bias_tau"])
        accel_params["arw"].append(ap["arw"])
        accel_params["bias_instability"].append(ap["bias_instability"])
        accel_params["bias_tau"].append(ap["bias_tau"])

    # ── Print results ──
    print()
    print("=" * 72)
    print("  IMU Noise Parameter Estimation (Allan Deviation)")
    print("=" * 72)

    gyr_arw = np.mean([v for v in gyro_params["arw"] if not np.isnan(v)])
    acc_arw = np.mean([v for v in accel_params["arw"] if not np.isnan(v)])
    gyr_bi = np.mean([v for v in gyro_params["bias_instability"] if not np.isnan(v)])
    acc_bi = np.mean([v for v in accel_params["bias_instability"] if not np.isnan(v)])

    print(f"\n  Sample rate:       {1.0/dt:.0f} Hz")
    print(f"  Duration:           {len(gyro)*dt:.1f}s")
    print()
    print(f"  {'Parameter':<25} {'X':>12} {'Y':>12} {'Z':>12}  {'Mean':>12}")
    print(f"  {'-'*25} {'-'*12} {'-'*12} {'-'*12}  {'-'*12}")
    for name, vals in [("Gyro ARW (rad/s/√Hz)", gyro_params["arw"]),
                        ("Accel VRW (m/s²/√Hz)", accel_params["arw"]),
                        ("Gyro Bias Inst. (rad/s)", gyro_params["bias_instability"]),
                        ("Accel Bias Inst. (m/s²)", accel_params["bias_instability"])]:
        line = f"  {name:<25}"
        for v in vals:
            line += f" {v:12.3e}" if not np.isnan(v) else "         N/A"
        mean_v = np.mean([v for v in vals if not np.isnan(v)])
        line += f"  {mean_v:12.3e}"
        print(line)

    print()
    print(f"  Recommended YAML config:")
    print(f"    gyr_cov:      {gyr_arw:.6f}")
    print(f"    acc_cov:      {acc_arw:.6f}")
    print(f"    gyr_bias_cov: {gyr_bi:.6f}")
    print(f"    acc_bias_cov: {acc_bi:.6f}")
    print()

    # ── Plot ──
    plot_allan(gyro_params, accel_params, tau_list, ad_gyro, ad_accel, args.output)


if __name__ == "__main__":
    main()
