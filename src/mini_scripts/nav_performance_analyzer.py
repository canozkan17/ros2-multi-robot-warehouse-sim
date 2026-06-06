#!/usr/bin/env python3
"""
nav_performance_analyzer.py - Real-Time Control Loop Oscillation and Wobble Analyzer.

This node subscribes to cmd_vel and calculates:
1. Root Mean Square (RMS) of Angular Velocity (measures overall steering energy).
2. Zero-Crossing Rate (ZCR) (measures oscillation frequency).
3. Peak Angular Velocity (measures maximum steering correction).
"""

import math
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class NavPerformanceAnalyzer(Node):
    def __init__(self):
        super().__init__('nav_performance_analyzer')
        
        self.declare_parameter('robot_name', 'robot1')
        self.robot_name = self.get_parameter('robot_name').value
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            f'/{self.robot_name}/cmd_vel',
            self._cmd_vel_callback,
            10
        )
        
        self.angular_velocities = []
        self.timestamps = []
        
        self.max_samples = 300  # 30 seconds of data at 10Hz
        self.get_logger().info(f"[{self.robot_name}] Performance Analyzer active. Collecting {self.max_samples} samples...")

    def _cmd_vel_callback(self, msg: Twist):
        # Ignore zero velocity (idle state) at start
        if len(self.angular_velocities) == 0 and abs(msg.linear.x) < 0.05:
            return
            
        if len(self.angular_velocities) >= self.max_samples:
            return
            
        self.angular_velocities.append(msg.angular.z)
        self.timestamps.append(self.get_clock().now().nanoseconds / 1e9)
        
        # Log progress every 50 samples
        if len(self.angular_velocities) % 50 == 0:
            self.get_logger().info(f"Collected {len(self.angular_velocities)}/{self.max_samples} samples...")
            
        if len(self.angular_velocities) == self.max_samples:
            self._calculate_metrics()

    def _calculate_metrics(self):
        n = len(self.angular_velocities)
        if n < 2:
            self.get_logger().error("Not enough data collected.")
            return
            
        # 1. Root Mean Square (RMS) of steering velocity
        sum_sq = sum(w ** 2 for w in self.angular_velocities)
        rms_w = math.sqrt(sum_sq / n)
        
        # 2. Peak (Max Absolute) steering command
        peak_w = max(abs(w) for w in self.angular_velocities)
        
        # 3. Zero-Crossing Rate (ZCR) - how many times steering crossed 0 line
        zero_crossings = 0
        for i in range(1, n):
            if (self.angular_velocities[i] >= 0 and self.angular_velocities[i-1] < 0) or \
               (self.angular_velocities[i] < 0 and self.angular_velocities[i-1] >= 0):
                zero_crossings += 1
                
        duration = self.timestamps[-1] - self.timestamps[0]
        zcr = zero_crossings / duration if duration > 0 else 0.0
        
        # Render clean performance report
        print("\n" + "="*50)
        print(f" NAVIGATION QUALITY REPORT FOR [{self.robot_name}]")
        print("="*50)
        print(f" Sample Count           : {n}")
        print(f" Test Duration (sec)    : {duration:.2f}")
        print(f" RMS Angular Velocity   : {rms_w:.4f} rad/s  (Lower = smoother tracking)")
        print(f" Peak Angular Velocity  : {peak_w:.4f} rad/s  (Lower = less aggressive overshoot)")
        print(f" Zero-Crossing Rate     : {zcr:.4f} Hz     (Lower = lower frequency oscillation)")
        print("="*50)
        print(" EVALUATION:")
        if rms_w > 0.12 or zcr > 0.4:
            print(" --> STATUS: HIGH OSCILLATION (Wobbling / Drunk-driving detected)")
            print(" --> SUGGESTION: Adjust lookahead boundary limits or reduce velocity.")
        else:
            print(" --> STATUS: STABLE AND SMOOTH TRACKING")
            print(" --> SUGGESTION: Lookahead and linear velocity are well-balanced.")
        print("="*50 + "\n")
        
        # Reset registers for the next sweep run
        self.angular_velocities.clear()
        self.timestamps.clear()
        self.get_logger().info("Ready for next collection run...")


def main(args=None):
    rclpy.init(args=args)
    node = NavPerformanceAnalyzer()
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