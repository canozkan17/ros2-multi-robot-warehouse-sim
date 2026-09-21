#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelStates
import csv
import os
import argparse
import time

class Recorder(Node):
    def __init__(self, outdir, timeout=5.0):
        super().__init__('bag_to_csv_recorder')
        self.outdir = outdir
        os.makedirs(self.outdir, exist_ok=True)
        self.odom_file = open(os.path.join(self.outdir, 'odom.csv'), 'w')
        self.amcl_file = open(os.path.join(self.outdir, 'amcl.csv'), 'w')
        self.model_file = open(os.path.join(self.outdir, 'model_states.csv'), 'w')
        self.odom_writer = csv.writer(self.odom_file)
        self.amcl_writer = csv.writer(self.amcl_file)
        self.model_writer = csv.writer(self.model_file)
        self.odom_writer.writerow(['stamp_sec','stamp_nsec','seq','x','y','z','qx','qy','qz','qw','vx','vy','vz','vroll','vpitch','vyaw'])
        self.amcl_writer.writerow(['stamp_sec','stamp_nsec','seq','x','y','z','qx','qy','qz','qw','cov0','cov1','cov2','cov3','cov4','cov5'])
        self.model_writer.writerow(['stamp_sec','stamp_nsec','names','poses'])
        self.last_received = time.time()
        self.timeout = timeout
        qos_profile = rclpy.qos.QoSProfile(depth=10)
        self.create_subscription(Odometry, '/robot1/odom', self.odom_cb, qos_profile)
        self.create_subscription(PoseWithCovarianceStamped, '/robot1/amcl_pose', self.amcl_cb, qos_profile)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.model_cb, qos_profile)
        self.get_logger().info(f'Listening to /robot1/odom, /robot1/amcl_pose, /gazebo/model_states. Writing to {self.outdir}')
        self.timer = self.create_timer(1.0, self.check_timeout)

    def odom_cb(self, msg: Odometry):
        ts = msg.header.stamp
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        # no roll/pitch/yaw in twist; leave zeros
        self.odom_writer.writerow([ts.sec, ts.nanosec, msg.header.frame_id, p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z, 0,0,0])
        self.odom_file.flush()
        self.last_received = time.time()

    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        ts = msg.header.stamp
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        cov = msg.pose.covariance
        cov0 = cov[0] if len(cov)>0 else ''
        cov4 = cov[4] if len(cov)>4 else ''
        cov5 = cov[35] if len(cov)>35 else ''
        self.amcl_writer.writerow([ts.sec, ts.nanosec, msg.header.frame_id, p.x, p.y, p.z, q.x, q.y, q.z, q.w, cov0, cov4, cov5, '', '', ''])
        self.amcl_file.flush()
        self.last_received = time.time()

    def model_cb(self, msg: ModelStates):
        ts = self.get_clock().now().to_msg()
        # write names count and first pose of robot1 if present
        names = '|'.join(msg.name)
        poses = '|'.join([f"{p.position.x:.3f},{p.position.y:.3f},{p.position.z:.3f}" for p in msg.pose])
        self.model_writer.writerow([ts.sec, ts.nanosec, names, poses])
        self.model_file.flush()
        self.last_received = time.time()

    def check_timeout(self):
        if time.time() - self.last_received > self.timeout:
            self.get_logger().info('No messages for timeout period; shutting down.')
            self.cleanup()
            rclpy.shutdown()

    def cleanup(self):
        try:
            self.odom_file.close()
            self.amcl_file.close()
            self.model_file.close()
        except Exception:
            pass

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--outdir', default='run_final_csv')
    parser.add_argument('--timeout', type=float, default=5.0)
    args = parser.parse_args()
    rclpy.init()
    node = Recorder(args.outdir, timeout=args.timeout)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt')
    finally:
        node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
