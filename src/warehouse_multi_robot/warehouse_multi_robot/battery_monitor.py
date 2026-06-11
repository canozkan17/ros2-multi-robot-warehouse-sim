#!/usr/bin/env python3
"""
battery_monitor.py - Simulated Battery and Hardware Status Monitor.

This node simulates battery consumption over time. When the battery charge
reaches the failure threshold, it publishes a FAILED status to trigger
autonomous reallocation among surviving fleet members.
"""

import rclpy
import math
import json
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__("battery_monitor")
        
        # Declare Parameters
        self.declare_parameter("robot_name", "robot1")
        self.declare_parameter("drain_rate", 0.003)  # Amount of charge lost per tick
        self.declare_parameter("fail_at", 0.0)      # Charge percentage that triggers failure
        self.declare_parameter("start_at", 100.0)   # Initial battery percentage
        
        self.robot_name = self.get_parameter("robot_name").value
        self.drain_rate = self.get_parameter("drain_rate").value
        self.fail_at = self.get_parameter("fail_at").value
        self.current_charge = self.get_parameter("start_at").value
        
        # State Indicators
        self.is_failed = False
        
        # Publisher
        qos_status = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        # Force explicit absolute status topic pathing matching the core agent node
        absolute_status_topic = f"/{self.robot_name}/status"
        self.status_pub = self.create_publisher(String, absolute_status_topic, qos_status)
        
        # Tracking variables for physical distance-based drain
        self.last_x = None
        self.last_y = None
        self.dynamic_drain_coefficient = 0.15  # Battery depleted by 0.15% per meter moved
        
        # Subscribe to local namespaced AMCL pose to calculate step distance
        qos_transient = QoSProfile(
            depth=10, 
            durability=DurabilityPolicy.TRANSIENT_LOCAL, 
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.create_subscription(
            PoseWithCovarianceStamped, 
            "amcl_pose", 
            self._pose_callback, 
            qos_transient
        )

        # Timer (10 Hz rate to drain battery smoothly)
        self.drain_timer = self.create_timer(0.1, self._drain_battery_tick)
        
        self.get_logger().info(
            f"[{self.robot_name}] Battery Monitor active. Start charge: {self.current_charge}%"
        )

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        """Monitors step-by-step physical coordinate movements to apply dynamic battery drain."""
        if self.is_failed:
            return
            
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        if self.last_x is not None and self.last_y is not None:
            step_distance = math.hypot(x - self.last_x, y - self.last_y)
            # Apply dynamic consumption based on distance moved between ticks
            self.current_charge -= (step_distance * self.dynamic_drain_coefficient)
            
        self.last_x = x
        self.last_y = y

    def _drain_battery_tick(self):
        """Simulates battery depletion. Triggers failure state at the threshold."""
        if self.is_failed:
            return
            
        # Smoothly apply static idling drain (LiDAR, CPU, Electronics consumption)
        self.current_charge -= self.drain_rate
        
        if self.current_charge <= self.fail_at:
            self.current_charge = self.fail_at
            self.is_failed = True
            self._trigger_failure()
        else:
            self._publish_consolidated_status()
            
    def _publish_consolidated_status(self):
        """Publishes 10Hz periodic JSON heartbeat with dynamic battery telemetries."""
        payload = {
            "status": "ACTIVE",
            "battery": round(self.current_charge, 2)
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)
            
    def _trigger_failure(self):
        """Publishes FAILED status JSON to inform fleet coordination systems."""
        self.get_logger().error(
            f"[{self.robot_name}] BATTERY DEPLETED! Triggering simulated hardware failure."
        )
        payload = {
            "status": "FAILED",
            "battery": 0.0
        }
        msg = String()
        msg.data = json.dumps(payload)
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