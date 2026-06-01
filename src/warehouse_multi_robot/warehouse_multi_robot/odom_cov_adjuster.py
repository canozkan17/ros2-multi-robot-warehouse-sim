#!/usr/bin/env python3
"""
Republishes odometry with adjusted covariance to reduce odom overconfidence.
Usage: run per robot namespace, e.g.:
  ros2 run mini_scripts odom_cov_adjuster.py --robot robot1
This will subscribe to '/{robot}/odom' and publish '/{robot}/odom_fixed'.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import argparse

DEFAULT_COV = [0.0004,0,0,0,0,0,
               0,0.0004,0,0,0,0,
               0,0,0.0004,0,0,0,
               0,0,0,0.0001,0,0,
               0,0,0,0,0.0001,0,
               0,0,0,0,0,0.01]

class OdomAdjuster(Node):
    def __init__(self, robot):
        super().__init__('odom_cov_adjuster_' + robot)
        self.robot = robot
        sub_topic = f'/{robot}/odom'
        pub_topic = f'/{robot}/odom_fixed'
        qos = rclpy.qos.QoSProfile(depth=10)
        self.pub = self.create_publisher(Odometry, pub_topic, qos)
        self.sub = self.create_subscription(Odometry, sub_topic, self.cb_odom, qos)
        self.get_logger().info(f'Subscribed: {sub_topic} -> Publishing: {pub_topic}')

    def cb_odom(self, msg: Odometry):
        # clone and replace covariance
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist
        out.pose.covariance = DEFAULT_COV
        # optionally set twist covariance too (keep original)
        try:
            out.twist.covariance = msg.twist.covariance
        except Exception:
            pass
        self.pub.publish(out)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = OdomAdjuster(args.robot)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
