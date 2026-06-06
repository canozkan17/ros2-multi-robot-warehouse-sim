#!/usr/bin/env python3
"""
mission_gate.py — Global mission start latch for multi-robot runs.

Operators publish a volatile trigger on /mission_start; this node republishes
/mission_armed with TRANSIENT_LOCAL so waypoint_sender nodes that start later
still receive the armed state.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool as BoolMsg

MISSION_ARMED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class MissionGate(Node):
    def __init__(self):
        super().__init__('mission_gate')
        self.declare_parameter('auto_start', False)
        self.declare_parameter('republish_hz', 1.0)
        self._armed = bool(self.get_parameter('auto_start').value)
        republish_hz = float(self.get_parameter('republish_hz').value)

        self._armed_pub = self.create_publisher(BoolMsg, '/mission_armed', MISSION_ARMED_QOS)
        self.create_subscription(BoolMsg, '/mission_start', self._on_mission_start, 10)

        period = 1.0 / max(0.2, republish_hz)
        self.create_timer(period, self._republish_armed)

        if self._armed:
            self._publish_armed(True)
            self.get_logger().info('MISSION-ARMED (auto_start:=true, latched on /mission_armed)')
        else:
            self.get_logger().info(
                'WAIT-ARMED — publish /mission_start (std_msgs/Bool data: true) to begin')

    def _publish_armed(self, value: bool):
        msg = BoolMsg()
        msg.data = value
        self._armed_pub.publish(msg)

    def _republish_armed(self):
        if self._armed:
            self._publish_armed(True)

    def _on_mission_start(self, msg: BoolMsg):
        if not msg.data:
            return
        if self._armed:
            self.get_logger().info('MISSION-ARMED already latched (ignored duplicate /mission_start)')
            return
        self._armed = True
        self._publish_armed(True)
        self.get_logger().info('MISSION-ARMED latched on /mission_armed (all robots)')


def main(args=None):
    rclpy.init(args=args)
    node = MissionGate()
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
