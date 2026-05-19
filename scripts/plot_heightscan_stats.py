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

# 默认日志目录（相对于脚本所在项目的根目录）
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
    return files[-1]  # 按文件名排序（含日期时间），最新的在最后


def load_data(csv_path):
    """读取 CSV，返回字典列表。"""
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
                if key in row:
                    data[key].append(float(row[key]))

    # 转为 numpy 数组
    for key in data:
        data[key] = np.array(data[key])

    # 时间轴归零（以第一帧为起点）
    if len(data["timestamp_sec"]) > 0:
        data["timestamp_sec"] -= data["timestamp_sec"][0]

    return data


def moving_average(arr, window=15):
    """对一维数组做滑动平均滤波，边界处缩短窗口避免 NaN。"""
    if len(arr) < window or window < 2:
        return arr.copy()
    # 使用 cumsum 实现 O(n) 滑动平均
    cumsum = np.cumsum(np.insert(arr, 0, 0))
    result = (cumsum[window:] - cumsum[:-window]) / float(window)
    # 边界处理：前 window-1 个点用逐渐增大的窗口
    prefix = []
    for i in range(1, window):
        prefix.append(np.mean(arr[:i]))
    return np.concatenate([prefix, result])


def plot(data, save_path=None):
    # Determine which data fields are available
    has_power_data = len(data["power_watt"]) > 0
    has_odom_data = len(data["odom_x"]) > 0
    has_odom_tf_transform_freq = len(data["odom_tf_transform_freq_hz"]) > 0
    has_odom_topic_freq = len(data["odom_topic_freq_hz"]) > 0
    has_odom_topic_transform_freq = len(data["odom_topic_transform_freq_hz"]) > 0
    has_odom_any_freq = has_odom_tf_transform_freq or has_odom_topic_freq or has_odom_topic_transform_freq
    has_cpu = len(data["cpu_load_percent"]) > 0 and np.any(data["cpu_load_percent"] >= 0)
    has_gpu = len(data["gpu_load_percent"]) > 0 and np.any(data["gpu_load_percent"] >= 0)
    has_mem = len(data["mem_used_mb"]) > 0 and np.any(data["mem_used_mb"] >= 0)

    # Count charts: base 4 (height, freq, cpu/gpu, mem) + optional power + optional odom
    n_plots = 4
    if has_power_data:
        n_plots += 1
    if has_odom_data:
        n_plots += 1
    if has_odom_any_freq:
        n_plots += 1

    if not has_power_data:
        print("[INFO] No 'power_watt' column found in CSV (old log format); skipping power chart.")
    if not has_odom_data:
        print("[INFO] No 'odom_x/y/z' columns found in CSV (old log format); skipping odom chart.")
    if not has_odom_any_freq:
        print("[INFO] No 'odom_tf_transform_freq_hz/odom_topic_freq_hz/odom_topic_transform_freq_hz' columns found in CSV (old log format); skipping odom freq chart.")

    fig, axes = plt.subplots(n_plots, 1, figsize=(12, 14), sharex=True)
    fig.suptitle("Nvblox LocomotionHeightScan Statistics", fontsize=14)

    t = data["timestamp_sec"]
    ax_idx = 0

    # 1. 高程均值与最大偏差
    ax = axes[ax_idx]; ax_idx += 1
    ax.plot(t, data["mean_height"], label="mean_height", color="C0")
    ax.fill_between(
        t,
        data["mean_height"] - data["max_deviation"] / 2,
        data["mean_height"] + data["max_deviation"] / 2,
        alpha=0.3,
        color="C0",
        label="max_deviation band",
    )
    ax.set_ylabel("Height (m)")
    ax.legend(loc="upper right")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_title("Mean Height & Max Deviation")

    # 2. 发布频率
    ax = axes[ax_idx]; ax_idx += 1
    freq_raw = data["publish_freq_hz"]
    freq_smooth = moving_average(freq_raw, window=15)
    ax.plot(t, freq_raw, color="C1", linewidth=0.8, alpha=0.4, label="raw")
    ax.plot(t, freq_smooth, color="C1", linewidth=1.8, label="filtered")
    ax.set_ylabel("Freq (Hz)")
    ax.set_ylim(0, 100)
    ax.legend(loc="upper right")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_title("Publish Frequency")

    # 3. CPU / GPU 负载
    ax = axes[ax_idx]; ax_idx += 1
    if has_cpu:
        cpu_raw = data["cpu_load_percent"][data["cpu_load_percent"] >= 0]
        t_cpu = t[data["cpu_load_percent"] >= 0]
        cpu_smooth = moving_average(cpu_raw, window=15)
        ax.plot(t_cpu, cpu_raw, color="C2", linewidth=0.8, alpha=0.4)
        ax.plot(t_cpu, cpu_smooth, color="C2", linewidth=1.8, label="CPU")
    else:
        print("[INFO] No valid 'cpu_load_percent' data found; skipping CPU plot.")
    if has_gpu:
        gpu_raw = data["gpu_load_percent"][data["gpu_load_percent"] >= 0]
        t_gpu = t[data["gpu_load_percent"] >= 0]
        gpu_smooth = moving_average(gpu_raw, window=15)
        ax.plot(t_gpu, gpu_raw, color="C3", linewidth=0.8, alpha=0.4)
        ax.plot(t_gpu, gpu_smooth, color="C3", linewidth=1.8, label="GPU")
    else:
        print("[INFO] No valid 'gpu_load_percent' data found; skipping GPU plot.")
    ax.set_ylabel("Load (%)")
    ax.set_ylim(0, 105)
    ax.legend(loc="upper right")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_title("CPU & GPU Load")

    # 4. 内存使用
    ax = axes[ax_idx]; ax_idx += 1
    valid_mem = data["mem_used_mb"] >= 0
    if has_mem:
        ax.plot(t[valid_mem], data["mem_used_mb"][valid_mem], label="Used", color="C4")
        if len(data["mem_total_mb"]) > 0 and data["mem_total_mb"][0] > 0:
            ax.axhline(
                data["mem_total_mb"][0],
                color="C4",
                linestyle="--",
                alpha=0.6,
                label="Total",
            )
    else:
        print("[INFO] No valid 'mem_used_mb' data found; skipping memory plot.")
    ax.set_ylabel("Memory (MB)")
    ax.legend(loc="upper right")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_title("Memory Usage")

    # 5. 整机功率
    if has_power_data:
        ax = axes[ax_idx]; ax_idx += 1
        valid_power = data["power_watt"] >= 0
        if np.any(valid_power):
            p_raw = data["power_watt"][valid_power]
            t_power = t[valid_power]
            p_smooth = moving_average(p_raw, window=15)
            ax.plot(t_power, p_raw, color="C5", linewidth=0.8, alpha=0.4, label="raw")
            ax.plot(t_power, p_smooth, color="C5", linewidth=1.8, label="filtered")
        else:
            print("[INFO] No valid 'power_watt' data found; power sensor may not be available.")
            ax.text(0.5, 0.5, "No power data available", ha='center', va='center',
                    transform=ax.transAxes, fontsize=14, alpha=0.5)
        ax.set_ylabel("Power (W)")
        ax.legend(loc="upper right")
        ax.grid(True, linestyle="--", alpha=0.5)
        ax.set_title("System Power Consumption")

    # 6. 里程计 x/y/z
    if has_odom_data:
        ax = axes[ax_idx]; ax_idx += 1
        if len(data["odom_x"]) > 0:
            ax.plot(t[:len(data["odom_x"])], data["odom_x"], label="X", color="C6")
        if len(data["odom_y"]) > 0:
            ax.plot(t[:len(data["odom_y"])], data["odom_y"], label="Y", color="C7")
        if len(data["odom_z"]) > 0:
            ax.plot(t[:len(data["odom_z"])], data["odom_z"], label="Z", color="C8")
        ax.set_ylabel("Position (m)")
        ax.legend(loc="upper right")
        ax.grid(True, linestyle="--", alpha=0.5)
        ax.set_title("cuVSLAM Odometry X/Y/Z")

    # 7. 里程计频率（TF / 里程计话题 / pose-transform 话题同图）
    if has_odom_any_freq:
        ax = axes[ax_idx]; ax_idx += 1
        if has_odom_tf_transform_freq and len(data["odom_tf_transform_freq_hz"]) > 0:
            odom_tf_raw = data["odom_tf_transform_freq_hz"]
            odom_tf_smooth = moving_average(odom_tf_raw, window=15)
            ax.plot(t[:len(odom_tf_raw)], odom_tf_raw, color="C9", linewidth=0.8, alpha=0.35, label="TF raw")
            ax.plot(t[:len(odom_tf_smooth)], odom_tf_smooth, color="C9", linewidth=1.8, label="TF filtered")
        if has_odom_topic_freq and len(data["odom_topic_freq_hz"]) > 0:
            odom_topic_raw = data["odom_topic_freq_hz"]
            odom_topic_smooth = moving_average(odom_topic_raw, window=15)
            ax.plot(t[:len(odom_topic_raw)], odom_topic_raw, color="C10", linewidth=0.8, alpha=0.35, label="Odom topic raw")
            ax.plot(t[:len(odom_topic_smooth)], odom_topic_smooth, color="C10", linewidth=1.8, label="Odom topic filtered")
        if has_odom_topic_transform_freq and len(data["odom_topic_transform_freq_hz"]) > 0:
            odom_transform_raw = data["odom_topic_transform_freq_hz"]
            odom_transform_smooth = moving_average(odom_transform_raw, window=15)
            ax.plot(t[:len(odom_transform_raw)], odom_transform_raw, color="C11", linewidth=0.8, alpha=0.35, label="Pose/transform topic raw")
            ax.plot(t[:len(odom_transform_smooth)], odom_transform_smooth, color="C11", linewidth=1.8, label="Pose/transform topic filtered")
        ax.set_ylabel("Freq (Hz)")
        ax.set_ylim(0, 100)
        ax.legend(loc="upper right")
        ax.grid(True, linestyle="--", alpha=0.5)
        ax.set_title("Odometry Frequency (TF vs Topics)")

    # Set xlabel on last axis
    axes[-1].set_xlabel("Time (s)")

    plt.tight_layout(rect=[0, 0, 1, 0.96])

    if save_path:
        plt.savefig(save_path, dpi=300)
        print(f"Figure saved to: {save_path}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Plot HeightScan statistics from CSV log.")
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
        help="Save figure to file instead of displaying",
    )
    args = parser.parse_args()

    # 确定要读取的 CSV 文件
    csv_path = args.csv
    if csv_path is None:
        csv_path = find_latest_log()
        if csv_path is None:
            print(f"Error: no CSV files found in {DEFAULT_LOG_DIR}", file=sys.stderr)
            print("Specify a file path explicitly: "
                  "python3 plot_heightscan_stats.py <path>", file=sys.stderr)
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
