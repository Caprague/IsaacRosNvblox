#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Diagnostic tool for Mid360 to nvblox bridge
Analyzes conversion quality and performance

Supports two output modes (controlled by bridge parameters):
  enable_structured_output=true  -> monitors /lidar/pointcloud_structured
  enable_depth_image_output=true -> monitors /lidar/depth_image
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from collections import deque
import time
import struct


class BridgeDiagnostics(Node):
    def __init__(self):
        super().__init__('bridge_diagnostics')

        # Parameters to match bridge output mode
        self.declare_parameter('enable_structured_output', False)
        self.declare_parameter('enable_depth_image_output', True)
        self.declare_parameter('input_pointcloud_topic', '/livox/lidar')
        self.declare_parameter('output_pointcloud_topic', '/lidar/pointcloud_structured')
        self.declare_parameter('output_depth_image_topic', '/lidar/depth_image')

        self.enable_structured = self.get_parameter('enable_structured_output').value
        self.enable_depth_image = self.get_parameter('enable_depth_image_output').value
        input_topic = self.get_parameter('input_pointcloud_topic').value
        output_pc_topic = self.get_parameter('output_pointcloud_topic').value
        output_img_topic = self.get_parameter('output_depth_image_topic').value

        # Subscribers
        self.sub_input = self.create_subscription(
            PointCloud2, input_topic, self.input_callback, 10)

        self.sub_output_pc = None
        self.sub_output_img = None

        if self.enable_structured:
            self.sub_output_pc = self.create_subscription(
                PointCloud2, output_pc_topic, self.output_pointcloud_callback, 10)
            self.get_logger().info(f'Monitoring structured pointcloud: {output_pc_topic}')

        if self.enable_depth_image:
            self.sub_output_img = self.create_subscription(
                Image, output_img_topic, self.output_depth_image_callback, 10)
            self.get_logger().info(f'Monitoring depth image: {output_img_topic}')

        if not self.enable_structured and not self.enable_depth_image:
            self.get_logger().warn(
                'Both enable_structured_output and enable_depth_image_output are false. '
                'No output will be monitored.')

        # Statistics
        self.input_stats = {'points': [], 'timestamps': deque(maxlen=100)}
        self.output_pc_stats = {
            'points': [], 'valid_ratio': [], 'timestamps': deque(maxlen=100)}
        self.output_img_stats = {
            'valid_ratio': [], 'min_depth': [], 'max_depth': [],
            'timestamps': deque(maxlen=100)}

        self.timer = self.create_timer(5.0, self.report_statistics)
        self.get_logger().info('Bridge diagnostics started')

    def input_callback(self, msg):
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        point_count = len(points)

        self.input_stats['points'].append(point_count)
        self.input_stats['timestamps'].append(time.time())

        if point_count > 0:
            points_np = np.array(points)
            if points_np.dtype.names:
                points_np = np.vstack([points_np['x'], points_np['y'], points_np['z']]).T
            ranges = np.linalg.norm(points_np, axis=1)
            self.get_logger().debug(
                f'Input: {point_count} points, '
                f'range: [{ranges.min():.2f}, {ranges.max():.2f}]m')

    def output_pointcloud_callback(self, msg):
        total_count = msg.height * msg.width
        actual_count = len(msg.data) // msg.point_step if msg.point_step > 0 else 0

        if total_count != actual_count:
            self.get_logger().error(
                f'Structure mismatch! height={msg.height}, width={msg.width} '
                f'(expected {total_count}), but data has {actual_count} points')
            return

        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        valid_count = len(points)
        valid_ratio = valid_count / total_count if total_count > 0 else 0.0

        self.output_pc_stats['points'].append(valid_count)
        self.output_pc_stats['valid_ratio'].append(valid_ratio)
        self.output_pc_stats['timestamps'].append(time.time())

        self.get_logger().debug(
            f'Structured PC: {valid_count}/{total_count} valid ({valid_ratio*100:.1f}%)')

    def output_depth_image_callback(self, msg):
        if msg.encoding != '32FC1':
            self.get_logger().warn(f'Unexpected depth image encoding: {msg.encoding}')
            return

        total_pixels = msg.height * msg.width
        if total_pixels == 0:
            return

        # Decode float32 depth image
        depth_data = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)

        valid_mask = np.isfinite(depth_data) & (depth_data > 0.0)
        valid_count = int(np.sum(valid_mask))
        valid_ratio = valid_count / total_pixels

        self.output_img_stats['valid_ratio'].append(valid_ratio)
        self.output_img_stats['timestamps'].append(time.time())

        if valid_count > 0:
            valid_depths = depth_data[valid_mask]
            self.output_img_stats['min_depth'].append(float(valid_depths.min()))
            self.output_img_stats['max_depth'].append(float(valid_depths.max()))
        else:
            self.output_img_stats['min_depth'].append(0.0)
            self.output_img_stats['max_depth'].append(0.0)

        self.get_logger().debug(
            f'Depth image {msg.width}x{msg.height}: '
            f'{valid_count}/{total_pixels} valid ({valid_ratio*100:.1f}%)')

    def report_statistics(self):
        if not self.input_stats['points']:
            self.get_logger().info('Waiting for input data...')
            return

        input_points_avg = np.mean(self.input_stats['points'][-20:])
        input_freq = self._calc_freq(self.input_stats['timestamps'])

        lines = [
            '=== Bridge Diagnostics (Last 20 frames) ===',
            f'Input:  {input_points_avg:.0f} pts/frame @ {input_freq:.1f} Hz',
        ]

        if self.enable_structured and self.output_pc_stats['points']:
            output_pts_avg = np.mean(self.output_pc_stats['points'][-20:])
            output_freq = self._calc_freq(self.output_pc_stats['timestamps'])
            valid_ratio_avg = np.mean(self.output_pc_stats['valid_ratio'][-20:])
            efficiency = (output_pts_avg / input_points_avg * 100) if input_points_avg > 0 else 0.0
            lines += [
                f'[Structured PC]  {output_pts_avg:.0f} pts/frame @ {output_freq:.1f} Hz',
                f'  Valid ratio: {valid_ratio_avg*100:.1f}%  |  Efficiency: {efficiency:.1f}%',
            ]
        elif self.enable_structured:
            lines.append('[Structured PC]  waiting for data...')

        if self.enable_depth_image and self.output_img_stats['valid_ratio']:
            img_freq = self._calc_freq(self.output_img_stats['timestamps'])
            valid_ratio_avg = np.mean(self.output_img_stats['valid_ratio'][-20:])
            min_d = np.mean(self.output_img_stats['min_depth'][-20:])
            max_d = np.mean(self.output_img_stats['max_depth'][-20:])
            lines += [
                f'[Depth Image]    @ {img_freq:.1f} Hz',
                f'  Valid ratio: {valid_ratio_avg*100:.1f}%  |  Depth range: [{min_d:.2f}, {max_d:.2f}]m',
            ]
        elif self.enable_depth_image:
            lines.append('[Depth Image]    waiting for data...')

        lines.append('==========================================')
        self.get_logger().info('\n' + '\n'.join(lines))

    def _calc_freq(self, timestamps):
        if len(timestamps) < 2:
            return 0.0
        diffs = [timestamps[i] - timestamps[i - 1] for i in range(1, len(timestamps))]
        avg_period = np.mean(diffs)
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
