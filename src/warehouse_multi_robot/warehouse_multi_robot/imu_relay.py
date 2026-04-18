#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuRelay(Node):
    def __init__(self):
        super().__init__('imu_relay')
        self.pub = self.create_publisher(Imu, '/imu_fixed', 10)
        self.sub = self.create_subscription(Imu, '/imu', self.cb, 10)

    def cb(self, msg):
        msg.header.frame_id = 'imu_link' 
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(ImuRelay())

if __name__ == '__main__':
    main()
