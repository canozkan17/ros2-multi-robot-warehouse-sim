#!/usr/bin/env python3
"""

Runs independently for each robot.
Parameters:
  - robot_name  : robot1 | robot2 | robot3
  - drain_rate  : battery percentage per second. (default: 1.0)
  - fail_at     : failure threshold (default: 0.0 -> total finish)
  - start_at    : starting battery percentage (default: 100.0) 

Topics:
  - /robotN/status       -> String  (ACTIVE | FAILED)
  - /robotN/battery_pct  -> Float32 (0.0 - 100.0)

Services:
  - /robotN/inject_failure -> Trigger (external failure trigger)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from std_srvs.srv import Trigger


class BatteryMonitor(Node):

    def __init__(self):
        super().__init__('battery_monitor')

        # Parameters
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('drain_rate', 1.0)   # % / seconds
        self.declare_parameter('fail_at', 0.0)       # FAILED in this threshold
        self.declare_parameter('start_at', 100.0)    # starting point. 

        self.robot_name = self.get_parameter('robot_name').value
        self.drain_rate = self.get_parameter('drain_rate').value
        self.fail_at    = self.get_parameter('fail_at').value
        self.battery    = self.get_parameter('start_at').value

        # status
        self.status = 'ACTIVE'
        self.failed = False

        # Publishers
        self.status_pub  = self.create_publisher(
            String, f'/{self.robot_name}/status', 10)
        self.battery_pub = self.create_publisher(
            Float32, f'/{self.robot_name}/battery_pct', 10)

        # External failure trigger service
        self.inject_srv = self.create_service(
            Trigger,
            f'/{self.robot_name}/inject_failure',
            self.inject_failure_callback)

        # Main loop: works once for every second
        self.timer = self.create_timer(1.0, self.tick)

        self.get_logger().info(
            f'[{self.robot_name}] Battery_Monitor started | '
            f'Start={self.battery:.1f}% | '
            f'drain={self.drain_rate:.2f}%/s | '
            f'fail_at={self.fail_at:.1f}%'
        )

    
    # Main Loop
    def tick(self):
        if self.failed:
            self._publish_status('FAILED')
            return

        # Drop battery
        self.battery = max(0.0, self.battery - self.drain_rate)

        # Battery pub
        batt_msg = Float32()
        batt_msg.data = self.battery
        self.battery_pub.publish(batt_msg)

        # control
        if self.battery <= self.fail_at:
            self._trigger_failure(reason='Battery Drained')
            return

        self._publish_status('ACTIVE')

        self.get_logger().debug(
            f'[{self.robot_name}] battery={self.battery:.1f}%')

    # Fauilure Trigger (internal + external)
    def _trigger_failure(self, reason: str = 'unknown'):
        if self.failed:
            return
        self.failed = True
        self.status = 'FAILED'
        self.get_logger().warn(
            f'[{self.robot_name}] FAILURE DETECTED — reason: {reason} | '
            f'Battery={self.battery:.1f}%'
        )
        self._publish_status('FAILED')

    def inject_failure_callback(self, request, response):
        """
        External Trigger: ros2 service call /robot1/inject_failure std_srvs/srv/Trigger {}
        """
        if self.failed:
            response.success = False
            response.message = f'{self.robot_name} already in FAILED status.'
        else:
            self._trigger_failure(reason='external inject_failure service')
            response.success = True
            response.message = f'{self.robot_name} went into FAILED state.'
        return response

    
    # Status publisher
    
    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()