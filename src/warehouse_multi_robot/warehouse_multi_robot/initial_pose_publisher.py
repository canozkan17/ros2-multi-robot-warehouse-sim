#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        
        robot_name = self.get_parameter('robot_name').value
        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        
        pub = self.create_publisher(
            PoseWithCovarianceStamped,
            f'/{robot_name}/initialpose',
            10
        )
        
        # wait 2 seconds and then send
        self.timer = self.create_timer(2.0, lambda: self.publish(pub, x, y, robot_name))
        self.sent = False

    def publish(self, pub, x, y, robot_name):
        if self.sent:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        pub.publish(msg)
        self.get_logger().info(f'{robot_name} initial pose sent: x={x}, y={y}')
        self.sent = True

def main():
    rclpy.init()
    node = InitialPosePublisher()
    rclpy.spin_once(node, timeout_sec=5.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()