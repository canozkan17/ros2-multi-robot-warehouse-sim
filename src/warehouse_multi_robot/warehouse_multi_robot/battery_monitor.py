#!/usr/bin/env python3
"""
battery_monitor.py - Simulated Battery and Hardware Status Monitor.

This node simulates battery consumption over time. When the battery charge
reaches the failure threshold, it publishes a FAILED status to trigger
autonomous reallocation among surviving fleet members.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__("battery_monitor")
        
        # Declare Parameters
        self.declare_parameter("robot_name", "robot1")
        self.declare_parameter("drain_rate", 0.01)  # Amount of charge lost per tick
        self.declare_parameter("fail_at", 0.0)      # Charge percentage that triggers failure
        self.declare_parameter("start_at", 100.0)   # Initial battery percentage
        
        self.robot_name = self.get_parameter("robot_name").value
        self.drain_rate = self.get_parameter("drain_rate").value
        self.fail_at = self.get_parameter("fail_at").value
        self.current_charge = self.get_parameter("start_at").value
        
        # State Indicators
        self.is_failed = False
        
        # Publisher
        self.status_pub = self.create_publisher(String, "status", 10)
        
        # Timer (10 Hz rate to drain battery smoothly)
        self.drain_timer = self.create_timer(0.1, self._drain_battery_tick)
        
        self.get_logger().info(
            f"[{self.robot_name}] Battery Monitor active. Start charge: {self.current_charge}%"
        )

    def _drain_battery_tick(self):
        """Simulates battery depletion. Triggers failure state at the threshold."""
        if self.is_failed:
            return
            
        # Smoothly drain the battery
        self.current_charge -= self.drain_rate
        
        if self.current_charge <= self.fail_at:
            self.current_charge = self.fail_at
            self.is_failed = True
            self._trigger_failure()
            
    def _trigger_failure(self):
        """Publishes FAILED status to inform fleet coordination systems."""
        self.get_logger().error(
            f"[{self.robot_name}] BATTERY DEPLETED! Triggering simulated hardware failure."
        )
        msg = String()
        msg.data = "FAILED"
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
           rclpy.shutdown()

if __name__ == "__main__":
    main()