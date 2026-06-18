#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Diagnostic tool for Mid360 to nvblox bridge
Analyzes conversion quality and performance
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from collections import deque
import time


class BridgeDiagnostics(Node):
    def __init__(self):
        super().__init__('bridge_diagnostics')
        
        # Subscribers
        self.sub_input = self.create_subscription(
            PointCloud2, '/livox/lidar', self.input_callback, 10)
        self.sub_output = self.create_subscription(
            PointCloud2, '/lidar/pointcloud_structured', self.output_callback, 10)
        
        # Statistics
        self.input_stats = {'points': [], 'timestamps': deque(maxlen=100)}
        self.output_stats = {'points': [], 'valid_ratio': [], 'timestamps': deque(maxlen=100)}
        
        # Timer for reporting
        self.timer = self.create_timer(5.0, self.report_statistics)
        
        self.get_logger().info('Bridge diagnostics started')
    
    def input_callback(self, msg):
        """Analyze input Mid360 pointcloud"""
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        point_count = len(points)
        
        self.input_stats['points'].append(point_count)
        self.input_stats['timestamps'].append(time.time())
        
        if len(points) > 0:
            # Analyze spatial distribution
            points_np = np.array(points)
            if points_np.dtype.names:
                points_np = np.vstack([points_np['x'], points_np['y'], points_np['z']]).T
            ranges = np.linalg.norm(points_np, axis=1)
            
            self.get_logger().debug(
                f'Input: {point_count} points, '
                f'range: [{ranges.min():.2f}, {ranges.max():.2f}]m'
            )
    
    def output_callback(self, msg):
        """Analyze output structured pointcloud"""
        # Check structure
        if msg.height * msg.width != len(msg.data) // msg.point_step:
            self.get_logger().error(
                f'Structure mismatch! height={msg.height}, width={msg.width}, '
                f'but data length suggests {len(msg.data) // msg.point_step} points'
            )
            return
        
        # Count valid points (non-NaN)
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        valid_count = len(points)
        total_count = msg.height * msg.width
        valid_ratio = valid_count / total_count if total_count > 0 else 0
        
        self.output_stats['points'].append(valid_count)
        self.output_stats['valid_ratio'].append(valid_ratio)
        self.output_stats['timestamps'].append(time.time())
        
        self.get_logger().debug(
            f'Output: {valid_count}/{total_count} valid points ({valid_ratio*100:.1f}%)'
        )
    
    def report_statistics(self):
        """Report periodic statistics"""
        if not self.input_stats['points'] or not self.output_stats['points']:
            self.get_logger().info('Waiting for data...')
            return
        
        # Input statistics
        input_points_avg = np.mean(self.input_stats['points'][-20:])
        input_freq = self.calculate_frequency(self.input_stats['timestamps'])
        
        # Output statistics
        output_points_avg = np.mean(self.output_stats['points'][-20:])
        output_freq = self.calculate_frequency(self.output_stats['timestamps'])
        valid_ratio_avg = np.mean(self.output_stats['valid_ratio'][-20:])
        
        # Conversion efficiency
        efficiency = (output_points_avg / input_points_avg * 100) if input_points_avg > 0 else 0
        
        self.get_logger().info(
            f'\n'
            f'=== Bridge Diagnostics (Last 20 frames) ===\n'
            f'Input:  {input_points_avg:.0f} pts/frame @ {input_freq:.1f} Hz\n'
            f'Output: {output_points_avg:.0f} pts/frame @ {output_freq:.1f} Hz\n'
            f'Valid ratio: {valid_ratio_avg*100:.1f}%\n'
            f'Efficiency: {efficiency:.1f}% (output/input)\n'
            f'=========================================='
        )
    
    def calculate_frequency(self, timestamps):
        """Calculate message frequency from timestamps"""
        if len(timestamps) < 2:
            return 0.0
        
        time_diffs = [timestamps[i] - timestamps[i-1] for i in range(1, len(timestamps))]
        avg_period = np.mean(time_diffs)
        return 1.0 / avg_period if avg_period > 0 else 0.0


def main(args=None):
    rclpy.init(args=args)
    node = BridgeDiagnostics()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
