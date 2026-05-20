#!/usr/bin/env python3
"""
可视化 nvblox_heightscan_stats CSV 日志文件。

用法:
    python3 plot_heightscan_stats.py [csv_path] [--save PATH]

默认读取路径: temp/logs/
  - 未指定文件时，自动读取该目录下最新的 nvblox_heightscan_stats_*.csv 文件
  - 可通过参数指定具体文件路径
"""

import argparse
import glob
import os
import sys

import matplotlib.pyplot as plt
import numpy as np


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
DEFAULT_LOG_DIR = os.path.join(PROJECT_ROOT, "temp", "logs")


def find_latest_log(log_dir=None):
    """在日志目录中查找最新的 CSV 日志文件。"""
    if log_dir is None:
        log_dir = DEFAULT_LOG_DIR
    pattern = os.path.join(log_dir, "nvblox_heightscan_stats_*.csv")
    files = sorted(glob.glob(pattern))
    if not files:
        return None
    return files[-1]


def load_data(csv_path):
    """读取 CSV，返回数据字典（所有字段均为 numpy 数组）。"""
    data = {
        "timestamp_sec": [],
        "mean_height": [],
        "max_deviation": [],
        "publish_freq_hz": [],
        "cpu_load_percent": [],
        "mem_used_mb": [],
        "mem_total_mb": [],
        "gpu_load_percent": [],
        "power_watt": [],
        "odom_tf_transform_freq_hz": [],
        "odom_topic_freq_hz": [],
        "odom_topic_transform_freq_hz": [],
        "odom_x": [],
        "odom_y": [],
        "odom_z": [],
    }

    with open(csv_path, "r") as f:
        header = f.readline().strip().split(",")
        for line in f:
            parts = line.strip().split(",")
            if len(parts) != len(header):
                continue
            row = dict(zip(header, parts))
            for key in data:
                if key in row and row[key] != "":
                    try:
                        data[key].append(float(row[key]))
                    except ValueError:
                        pass

    for key in data:
        data[key] = np.array(data[key], dtype=float)

    if len(data["timestamp_sec"]) > 0:
        data["timestamp_sec"] -= data["timestamp_sec"][0]

    return data


def moving_average(arr, window=15):
    """对一维数组做滑动平均滤波，边界处缩短窗口避免 NaN。"""
    if len(arr) < window or window < 2:
        return arr.copy()
    cumsum = np.cumsum(np.insert(arr, 0, 0.0))
    result = (cumsum[window:] - cumsum[:-window]) / float(window)
    prefix = [np.mean(arr[:i]) for i in range(1, window)]
    return np.concatenate([prefix, result])


def has_valid(data, key, valid_fn=None):
    """字段存在且有有效值。"""
    if key not in data:
        return False
    arr = data[key]
    if arr.size == 0:
        return False
    if valid_fn is None:
        return True
    mask = valid_fn(arr)
    return np.any(mask)


def valid_series(t, arr, valid_fn=None):
    """返回按有效值过滤后的 (t, arr)。"""
    if arr.size == 0:
        return np.array([]), np.array([])
    if valid_fn is None:
        n = min(len(t), len(arr))
        return t[:n], arr[:n]
    mask = valid_fn(arr)
    n = min(np.sum(mask), len(t))
    return t[:len(arr)][mask][:n], arr[mask][:n]


def save_figure(fig, save_path, suffix):
    base, ext = os.path.splitext(save_path)
    if ext:
        out_path = f"{base}_{suffix}{ext}"
    else:
        out_path = f"{save_path}_{suffix}.png"
    fig.savefig(out_path, dpi=300)
    print(f"Figure saved to: {out_path}")


def plot_height_status(data, t):
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.suptitle("Nvblox Height Status", fontsize=14)

    # 1) 高程均值与偏差
    ax = axes[0]
    if has_valid(data, "mean_height"):
        n = min(len(t), len(data["mean_height"]))
        mean_h = data["mean_height"][:n]
        tt = t[:n]
        ax.plot(tt, mean_h, label="mean_height", color="C0")

        if has_valid(data, "max_deviation"):
            d_n = min(n, len(data["max_deviation"]))
            dev = data["max_deviation"][:d_n]
            tt2 = t[:d_n]
            ax.fill_between(
                tt2,
                mean_h[:d_n] - dev / 2,
                mean_h[:d_n] + dev / 2,
                alpha=0.3,
                color="C0",
                label="max_deviation band",
            )
    else:
        ax.text(0.5, 0.5, "No valid mean_height data", ha="center", va="center", transform=ax.transAxes)

    ax.set_ylabel("Height (m)")
    ax.legend(loc="upper right")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_title("Mean Height & Max Deviation")

    # 2) HeightScan 发布频率
    ax = axes[1]
    if has_valid(data, "publish_freq_hz", lambda x: x >= 0):
        tt, freq = valid_series(t, data["publish_freq_hz"], lambda x: x >= 0)
        freq_smooth = moving_average(freq, window=15)
        ax.plot(tt, freq, color="C1", linewidth=0.8, alpha=0.4, label="raw")
        ax.plot(tt[:len(freq_smooth)], freq_smooth, color="C1", linewidth=1.8, label="filtered")
        ax.set_ylim(0, 100)
    else:
        ax.text(0.5, 0.5, "No valid publish_freq_hz data", ha="center", va="center", transform=ax.transAxes)

    ax.set_ylabel("Freq (Hz)")
    ax.set_xlabel("Time (s)")
    ax.legend(loc="upper right")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_title("HeightScan Publish Frequency")

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    return fig


def plot_system_status(data, t):
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle("Nvblox System Status", fontsize=14)

    # 1) CPU / GPU
    ax = axes[0]
    plotted = False
    if has_valid(data, "cpu_load_percent", lambda x: x >= 0):
        tt, cpu = valid_series(t, data["cpu_load_percent"], lambda x: x >= 0)
        cpu_smooth = moving_average(cpu, window=15)
        ax.plot(tt, cpu, color="C2", linewidth=0.8, alpha=0.4)
        ax.plot(tt[:len(cpu_smooth)], cpu_smooth, color="C2", linewidth=1.8, label="CPU")
        plotted = True
    if has_valid(data, "gpu_load_percent", lambda x: x >= 0):
        tt, gpu = valid_series(t, data["gpu_load_percent"], lambda x: x >= 0)
        gpu_smooth = moving_average(gpu, window=15)
        ax.plot(tt, gpu, color="C3", linewidth=0.8, alpha=0.4)
        ax.plot(tt[:len(gpu_smooth)], gpu_smooth, color="C3", linewidth=1.8, label="GPU")
        plotted = True
    if not plotted:
        ax.text(0.5, 0.5, "No valid CPU/GPU load data", ha="center", va="center", transform=ax.transAxes)

    ax.set_ylabel("Load (%)")
    ax.set_ylim(0, 105)
    ax.legend(loc="upper right")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_title("CPU & GPU Load")

    # 2) 内存
    ax = axes[1]
    if has_valid(data, "mem_used_mb", lambda x: x >= 0):
        tt, mem_used = valid_series(t, data["mem_used_mb"], lambda x: x >= 0)
        ax.plot(tt, mem_used, label="Used", color="C4")

        if has_valid(data, "mem_total_mb", lambda x: x > 0):
            total = data["mem_total_mb"][data["mem_total_mb"] > 0]
            if total.size > 0:
                ax.axhline(total[0], color="C4", linestyle="--", alpha=0.6, label="Total")
    else:
        ax.text(0.5, 0.5, "No valid memory data", ha="center", va="center", transform=ax.transAxes)

    ax.set_ylabel("Memory (MB)")
    ax.legend(loc="upper right")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_title("Memory Usage")

    # 3) 功率
    ax = axes[2]
    if has_valid(data, "power_watt", lambda x: x >= 0):
        tt, pw = valid_series(t, data["power_watt"], lambda x: x >= 0)
        pw_smooth = moving_average(pw, window=15)
        ax.plot(tt, pw, color="C5", linewidth=0.8, alpha=0.4, label="raw")
        ax.plot(tt[:len(pw_smooth)], pw_smooth, color="C5", linewidth=1.8, label="filtered")
    else:
        ax.text(0.5, 0.5, "No valid power data", ha="center", va="center", transform=ax.transAxes)

    ax.set_ylabel("Power (W)")
    ax.set_xlabel("Time (s)")
    ax.legend(loc="upper right")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_title("System Power Consumption")

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    return fig


def plot_odom_status(data, t):
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.suptitle("Nvblox Odometry Status", fontsize=14)

    # 1) 里程计位置
    ax = axes[0]
    plotted = False
    for key, color, label in [("odom_x", "C6", "X"), ("odom_y", "C7", "Y"), ("odom_z", "C8", "Z")]:
        if has_valid(data, key):
            n = min(len(t), len(data[key]))
            ax.plot(t[:n], data[key][:n], label=label, color=color)
            plotted = True
    if not plotted:
        ax.text(0.5, 0.5, "No valid odom position data", ha="center", va="center", transform=ax.transAxes)

    ax.set_ylabel("Position (m)")
    ax.legend(loc="upper right")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_title("Odometry X/Y/Z")

    # 2) 里程计频率
    ax = axes[1]
    freq_items = [
        ("odom_tf_transform_freq_hz", "C9", "TF transform"),
        ("odom_topic_freq_hz", "C10", "Odom topic"),
        ("odom_topic_transform_freq_hz", "C11", "Pose/transform topic"),
    ]
    plotted = False
    for key, color, name in freq_items:
        if has_valid(data, key, lambda x: x >= 0):
            tt, raw = valid_series(t, data[key], lambda x: x >= 0)
            smooth = moving_average(raw, window=15)
            ax.plot(tt, raw, color=color, linewidth=0.8, alpha=0.35, label=f"{name} raw")
            ax.plot(tt[:len(smooth)], smooth, color=color, linewidth=1.8, label=f"{name} filtered")
            plotted = True

    if not plotted:
        ax.text(0.5, 0.5, "No valid odometry frequency data", ha="center", va="center", transform=ax.transAxes)

    ax.set_ylabel("Freq (Hz)")
    ax.set_ylim(0, 100)
    ax.set_xlabel("Time (s)")
    ax.legend(loc="upper right")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_title("Odometry Frequency")

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    return fig


def plot(data, save_path=None):
    if len(data["timestamp_sec"]) == 0:
        print("Error: no valid timestamp data.", file=sys.stderr)
        return

    t = data["timestamp_sec"]

    fig_height = plot_height_status(data, t)
    fig_system = plot_system_status(data, t)
    fig_odom = plot_odom_status(data, t)

    if save_path:
        save_figure(fig_height, save_path, "height_status")
        save_figure(fig_system, save_path, "system_status")
        save_figure(fig_odom, save_path, "odometry_status")
        plt.close(fig_height)
        plt.close(fig_system)
        plt.close(fig_odom)
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Plot HeightScan statistics from CSV log.")
    parser.add_argument(
        "csv",
        nargs="?",
        default=None,
        help="Path to the CSV log file. If omitted, auto-detects the latest "
        f"file in {DEFAULT_LOG_DIR}",
    )
    parser.add_argument(
        "--save",
        "-s",
        metavar="PATH",
        help="Save figures to files instead of displaying",
    )
    args = parser.parse_args()

    csv_path = args.csv
    if csv_path is None:
        csv_path = find_latest_log()
        if csv_path is None:
            print(f"Error: no CSV files found in {DEFAULT_LOG_DIR}", file=sys.stderr)
            print("Specify a file path explicitly: python3 plot_heightscan_stats.py <path>", file=sys.stderr)
            sys.exit(1)
        print(f"Auto-detected latest log: {csv_path}")

    try:
        data = load_data(csv_path)
    except FileNotFoundError:
        print(f"Error: file not found: {csv_path}", file=sys.stderr)
        sys.exit(1)

    if len(data["timestamp_sec"]) == 0:
        print("Error: no valid data rows found in CSV.", file=sys.stderr)
        sys.exit(1)

    print(f"Loaded {len(data['timestamp_sec'])} records from {csv_path}")
    plot(data, save_path=args.save)


if __name__ == "__main__":
    main()
