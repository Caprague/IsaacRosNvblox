#!/usr/bin/env python3
"""
可视化 nvblox_heightscan_stats.csv 日志文件。

用法:
    python3 plot_heightscan_stats.py /path/to/nvblox_heightscan_stats.csv

默认读取路径: /tmp/nvblox_heightscan_stats.csv
"""

import argparse
import sys

import matplotlib.pyplot as plt
import numpy as np


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
    fig, axes = plt.subplots(4, 1, figsize=(12, 14), sharex=True)
    fig.suptitle("Nvblox LocomotionHeightScan Statistics", fontsize=14)

    t = data["timestamp_sec"]

    # 1. 高程均值与最大偏差
    ax = axes[0]
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
    ax = axes[1]
    freq_raw = data["publish_freq_hz"]
    freq_smooth = moving_average(freq_raw, window=15)
    ax.plot(t, freq_raw, color="C1", linewidth=0.8, alpha=0.4, label="raw")
    ax.plot(t, freq_smooth, color="C1", linewidth=1.8, label="filtered")
    ax.set_ylabel("Freq (Hz)")
    ax.set_ylim(bottom=0)
    ax.legend(loc="upper right")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_title("Publish Frequency")

    # 3. CPU / GPU 负载
    ax = axes[2]
    valid_cpu = data["cpu_load_percent"] >= 0
    valid_gpu = data["gpu_load_percent"] >= 0
    if np.any(valid_cpu):
        cpu_raw = data["cpu_load_percent"][valid_cpu]
        t_cpu = t[valid_cpu]
        cpu_smooth = moving_average(cpu_raw, window=15)
        ax.plot(t_cpu, cpu_raw, color="C2", linewidth=0.8, alpha=0.4)
        ax.plot(t_cpu, cpu_smooth, color="C2", linewidth=1.8, label="CPU")
    if np.any(valid_gpu):
        gpu_raw = data["gpu_load_percent"][valid_gpu]
        t_gpu = t[valid_gpu]
        gpu_smooth = moving_average(gpu_raw, window=15)
        ax.plot(t_gpu, gpu_raw, color="C3", linewidth=0.8, alpha=0.4)
        ax.plot(t_gpu, gpu_smooth, color="C3", linewidth=1.8, label="GPU")
    ax.set_ylabel("Load (%)")
    ax.set_ylim(0, 105)
    ax.legend(loc="upper right")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_title("CPU & GPU Load")

    # 4. 内存使用
    ax = axes[3]
    valid_mem = data["mem_used_mb"] >= 0
    if np.any(valid_mem):
        ax.plot(t[valid_mem], data["mem_used_mb"][valid_mem], label="Used", color="C4")
        if len(data["mem_total_mb"]) > 0 and data["mem_total_mb"][0] > 0:
            ax.axhline(
                data["mem_total_mb"][0],
                color="C4",
                linestyle="--",
                alpha=0.6,
                label="Total",
            )
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Memory (MB)")
    ax.legend(loc="upper right")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_title("Memory Usage")

    plt.tight_layout(rect=[0, 0, 1, 0.96])

    if save_path:
        plt.savefig(save_path, dpi=300)
        print(f"Figure saved to: {save_path}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Plot HeightScan statistics from CSV log.")
    parser.add_argument(
        "csv",
        nargs="?",
        default="/tmp/nvblox_heightscan_stats.csv",
        help="Path to the CSV log file (default: /tmp/nvblox_heightscan_stats.csv)",
    )
    parser.add_argument(
        "--save",
        "-s",
        metavar="PATH",
        help="Save figure to file instead of displaying",
    )
    args = parser.parse_args()

    try:
        data = load_data(args.csv)
    except FileNotFoundError:
        print(f"Error: file not found: {args.csv}")
        sys.exit(1)

    if len(data["timestamp_sec"]) == 0:
        print("Error: no valid data rows found in CSV.")
        sys.exit(1)

    print(f"Loaded {len(data['timestamp_sec'])} records from {args.csv}")
    plot(data, save_path=args.save)


if __name__ == "__main__":
    main()
