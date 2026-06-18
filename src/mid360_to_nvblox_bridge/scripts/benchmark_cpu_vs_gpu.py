#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Comprehensive benchmark comparing CPU vs GPU performance
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import time
from collections import defaultdict
import matplotlib.pyplot as plt
import json

class PerformanceBenchmark(Node):
    def __init__(self):
        super().__init__('performance_benchmark')
        
        # Subscribe to both CPU and GPU outputs
        self.sub_input = self.create_subscription(
            PointCloud2, '/livox/lidar', self.input_callback, 10)
        self.sub_cpu = self.create_subscription(
            PointCloud2, '/bridge_cpu/output', self.cpu_output_callback, 10)
        self.sub_gpu = self.create_subscription(
            PointCloud2, '/bridge_gpu/output', self.gpu_output_callback, 10)
        
        # Data storage
        self.data = defaultdict(list)
        self.start_time = time.time()
        self.benchmark_duration = 60.0  # seconds
        
        # Timer for periodic reporting
        self.timer = self.create_timer(5.0, self.report_progress)
        
        self.get_logger().info('Benchmark started - collecting data for 60s...')
    
    def input_callback(self, msg):
        """Record input pointcloud statistics"""
        points = list(pc2.read_points(msg, skip_nans=True))
        self.data['input_count'].append(len(points))
        self.data['input_time'].append(time.time())
    
    def cpu_output_callback(self, msg):
        """Record CPU processing results"""
        self.data['cpu_time'].append(time.time())
        points = list(pc2.read_points(msg, skip_nans=True))
        self.data['cpu_valid'].append(len(points))
        self.data['cpu_total'].append(msg.width * msg.height)
    
    def gpu_output_callback(self, msg):
        """Record GPU processing results"""
        self.data['gpu_time'].append(time.time())
        points = list(pc2.read_points(msg, skip_nans=True))
        self.data['gpu_valid'].append(len(points))
        self.data['gpu_total'].append(msg.width * msg.height)
    
    def report_progress(self):
        """Print progress report"""
        elapsed = time.time() - self.start_time
        
        if elapsed >= self.benchmark_duration:
            self.finalize_benchmark()
            rclpy.shutdown()
            return
        
        cpu_count = len(self.data['cpu_time'])
        gpu_count = len(self.data['gpu_time'])
        
        self.get_logger().info(
            f'Progress: {elapsed:.0f}s / {self.benchmark_duration:.0f}s  '
            f'Samples: CPU={cpu_count}, GPU={gpu_count}'
        )
    
    def finalize_benchmark(self):
        """Compute final statistics and generate report"""
        self.get_logger().info('Benchmark complete - analyzing results...')
        
        # Compute statistics
        results = {
            'cpu': self.compute_stats('cpu'),
            'gpu': self.compute_stats('gpu'),
            'input': {
                'avg_points': np.mean(self.data['input_count']),
                'total_frames': len(self.data['input_count'])
            }
        }
        
        # Compute speedup
        if results['cpu']['avg_latency_ms'] > 0:
            speedup = results['cpu']['avg_latency_ms'] / results['gpu']['avg_latency_ms']
        else:
            speedup = 0.0
        
        # Print report
        self.print_report(results, speedup)
        
        # Save results
        self.save_results(results, speedup)
        
        # Generate plots
        self.generate_plots(results)
    
    def compute_stats(self, mode):
        """Compute statistics for CPU or GPU"""
        if len(self.data[f'{mode}_time']) < 2:
            return {}
        
        # Compute frame-to-frame latency
        times = np.array(self.data[f'{mode}_time'])
        latencies = np.diff(times) * 1000  # ms
        
        valid_ratio = np.array(self.data[f'{mode}_valid']) / np.array(self.data[f'{mode}_total'])
        
        return {
            'total_frames': len(times),
            'avg_latency_ms': np.mean(latencies),
            'std_latency_ms': np.std(latencies),
            'min_latency_ms': np.min(latencies),
            'max_latency_ms': np.max(latencies),
            'p50_latency_ms': np.percentile(latencies, 50),
            'p95_latency_ms': np.percentile(latencies, 95),
            'p99_latency_ms': np.percentile(latencies, 99),
            'avg_valid_ratio': np.mean(valid_ratio),
            'avg_frequency_hz': 1000.0 / np.mean(latencies)
        }
    
    def print_report(self, results, speedup):
        """Print formatted benchmark report"""
        report = f"""
╔═══════════════════════════════════════════════════════════════╗
║           Mid360 Bridge Performance Benchmark Report          ║
╠═══════════════════════════════════════════════════════════════╣
║ Input Statistics                                              ║
║   Average points per frame: {results['input']['avg_points']:>10.0f}     ║
║   Total frames processed:   {results['input']['total_frames']:>10d}     ║
╠═══════════════════════════════════════════════════════════════╣
║ CPU Performance                                               ║
║   Average latency:          {results['cpu']['avg_latency_ms']:>10.2f} ms  ║
║   Std deviation:            {results['cpu']['std_latency_ms']:>10.2f} ms  ║
║   P50 latency:              {results['cpu']['p50_latency_ms']:>10.2f} ms  ║
║   P95 latency:              {results['cpu']['p95_latency_ms']:>10.2f} ms  ║
║   P99 latency:              {results['cpu']['p99_latency_ms']:>10.2f} ms  ║
║   Frequency:                {results['cpu']['avg_frequency_hz']:>10.1f} Hz ║
║   Valid ratio:              {results['cpu']['avg_valid_ratio']*100:>10.1f} %  ║
╠═══════════════════════════════════════════════════════════════╣
║ GPU Performance                                               ║
║   Average latency:          {results['gpu']['avg_latency_ms']:>10.2f} ms  ║
║   Std deviation:            {results['gpu']['std_latency_ms']:>10.2f} ms  ║
║   P50 latency:              {results['gpu']['p50_latency_ms']:>10.2f} ms  ║
║   P95 latency:              {results['gpu']['p95_latency_ms']:>10.2f} ms  ║
║   P99 latency:              {results['gpu']['p99_latency_ms']:>10.2f} ms  ║
║   Frequency:                {results['gpu']['avg_frequency_hz']:>10.1f} Hz ║
║   Valid ratio:              {results['gpu']['avg_valid_ratio']*100:>10.1f} %  ║
╠═══════════════════════════════════════════════════════════════╣
║ Comparison                                                    ║
║   Speedup (CPU/GPU):        {speedup:>10.2f}x     ║
║   Latency reduction:        {(1 - 1/speedup)*100:>10.1f} %  ║
╚═══════════════════════════════════════════════════════════════╝
"""
        print(report)
    
    def save_results(self, results, speedup):
        """Save results to JSON file"""
        output = {
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'duration_s': self.benchmark_duration,
            'results': results,
            'speedup': float(speedup)
        }
        
        filename = f'benchmark_results_{int(time.time())}.json'
        with open(filename, 'w') as f:
            json.dump(output, f, indent=2)
        
        self.get_logger().info(f'Results saved to {filename}')
    
    def generate_plots(self, results):
        """Generate performance visualization plots"""
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        
        # Plot 1: Latency distribution
        ax = axes[0, 0]
        cpu_times = np.diff(np.array(self.data['cpu_time'])) * 1000
        gpu_times = np.diff(np.array(self.data['gpu_time'])) * 1000
        ax.hist([cpu_times, gpu_times], bins=50, label=['CPU', 'GPU'], alpha=0.7)
        ax.set_xlabel('Latency (ms)')
        ax.set_ylabel('Frequency')
        ax.set_title('Latency Distribution')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Plot 2: Latency over time
        ax = axes[0, 1]
        ax.plot(cpu_times, 'b-', alpha=0.5, label='CPU')
        ax.plot(gpu_times, 'r-', alpha=0.5, label='GPU')
        ax.set_xlabel('Frame number')
        ax.set_ylabel('Latency (ms)')
        ax.set_title('Latency Over Time')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Plot 3: CDF comparison
        ax = axes[1, 0]
        cpu_sorted = np.sort(cpu_times)
        gpu_sorted = np.sort(gpu_times)
        cpu_cdf = np.arange(1, len(cpu_sorted)+1) / len(cpu_sorted)
        gpu_cdf = np.arange(1, len(gpu_sorted)+1) / len(gpu_sorted)
        ax.plot(cpu_sorted, cpu_cdf, 'b-', label='CPU')
        ax.plot(gpu_sorted, gpu_cdf, 'r-', label='GPU')
        ax.set_xlabel('Latency (ms)')
        ax.set_ylabel('CDF')
        ax.set_title('Cumulative Distribution Function')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Plot 4: Bar chart comparison
        ax = axes[1, 1]
        metrics = ['Mean', 'P50', 'P95', 'P99']
        cpu_vals = [results['cpu']['avg_latency_ms'], 
                   results['cpu']['p50_latency_ms'],
                   results['cpu']['p95_latency_ms'],
                   results['cpu']['p99_latency_ms']]
        gpu_vals = [results['gpu']['avg_latency_ms'],
                   results['gpu']['p50_latency_ms'],
                   results['gpu']['p95_latency_ms'],
                   results['gpu']['p99_latency_ms']]
        x = np.arange(len(metrics))
        width = 0.35
        ax.bar(x - width/2, cpu_vals, width, label='CPU', alpha=0.8)
        ax.bar(x + width/2, gpu_vals, width, label='GPU', alpha=0.8)
        ax.set_ylabel('Latency (ms)')
        ax.set_title('Latency Metrics Comparison')
        ax.set_xticks(x)
        ax.set_xticklabels(metrics)
        ax.legend()
        ax.grid(True, alpha=0.3, axis='y')
        
        plt.tight_layout()
        filename = f'benchmark_plots_{int(time.time())}.png'
        plt.savefig(filename, dpi=150)
        self.get_logger().info(f'Plots saved to {filename}')
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = PerformanceBenchmark()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
