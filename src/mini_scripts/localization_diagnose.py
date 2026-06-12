#!/usr/bin/env python3
"""
diagnosis_localization.py - Multi-Robot Localization and TF Alignment Diagnostic Suite.

This diagnostic node acts as a passive flight recorder. It monitors TF delays, 
Euler yaw decoupling, EKF stationary velocity drifts, and AMCL covariance spikes 
for robot1, robot2, and robot3. Upon shutdown (Ctrl+C), it writes a detailed 
ASCII analysis report to ~/thesis_ws/localization_diagnose_report.txt.
"""

import os
import sys
import math
import time
from typing import Dict, List, Tuple, Optional, Any

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import tf2_ros


class LocalizationDiagnosticSuite(Node):
    def __init__(self):
        super().__init__("localization_diagnostic_suite")
        
        self.get_logger().info("==================================================")
        self.get_logger().info("[Diagnostics] Initializing Passive Telemetry Flight Recorder")
        self.get_logger().info("==================================================")

        # Force simulation time
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)

        # TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Telemetry Registers for 3 Robots
        self.robots = ["robot1", "robot2", "robot3"]
        self.logs: Dict[str, Dict[str, List[Any]]] = {
            r: {
                "timestamps": [],
                "tf_map_base_latency": [],
                "tf_map_odom_latency": [],
                "tf_odom_base_latency": [],
                "yaw_offset_deg": [],
                "amcl_covariance_sigma": [],
                "ekf_linear_vel_x": [],
                "ekf_angular_vel_z": []
            } for r in self.robots
        }

        # Active status trackers
        self.amcl_latest_yaw: Dict[str, float] = {r: 0.0 for r in self.robots}
        self.amcl_covariance_sigma: Dict[str, float] = {r: 0.0 for r in self.robots}
        self.robot_nav_states: Dict[str, str] = {r: "BOOTSTRAP_AMCL" for r in self.robots}

        # Event Discrepancy Registry (to catch peaks)
        self.discrepancy_events: List[Dict[str, Any]] = []

        # QoS Profiles matching the core agent stack
        qos_transient = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Subscriptions
        for robot_name in self.robots:
            # AMCL Pose (Estimated Pose & Covariance)
            self.create_subscription(
                PoseWithCovarianceStamped,
                f"/{robot_name}/amcl_pose",
                lambda msg, r_name=robot_name: self._amcl_pose_callback(msg, r_name),
                qos_transient
            )
            # EKF Odom (Filtered Odometry & Velocities)
            self.create_subscription(
                Odometry,
                f"/{robot_name}/odom_filtered",
                lambda msg, r_name=robot_name: self._ekf_odom_callback(msg, r_name),
                10
            )
            # Consolidated Nav Status (True state machine states)
            self.create_subscription(
                String,
                f"/{robot_name}/nav_status",
                lambda msg, r_name=robot_name: self._nav_status_callback(msg, r_name),
                10
            )

        # Recording Loop Timer (Runs at 2.0 Hz to avoid CPU overhead on WSL2)
        self.recording_timer = self.create_timer(0.5, self._record_telemetry_tick)

        self.get_logger().info("[Diagnostics] Active. Logging data at 0.5 seconds intervals. Press Ctrl+C to stop.")

    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped, robot_name: str):
        # Extract Euler Yaw
        q = msg.pose.pose.orientation
        self.amcl_latest_yaw[robot_name] = self._euler_from_quaternion(q)

        # Extract standard deviation (sigma) of position estimation
        var_x = abs(msg.pose.covariance[0])
        var_y = abs(msg.pose.covariance[7])
        self.amcl_covariance_sigma[robot_name] = math.hypot(math.sqrt(var_x), math.sqrt(var_y))

    def _ekf_odom_callback(self, msg: Odometry, robot_name: str):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        self.logs[robot_name]["ekf_linear_vel_x"].append(msg.twist.twist.linear.x)
        self.logs[robot_name]["ekf_angular_vel_z"].append(msg.twist.twist.angular.z)

    def _nav_status_callback(self, msg: String, robot_name: str):
        self.robot_nav_states[robot_name] = msg.data

    def _record_telemetry_tick(self):
        """Active 2Hz tick querying TF buffers, comparing yaws, and logging event metrics."""
        now = self.get_clock().now()
        now_sec = now.nanoseconds / 1e9

        for r_name in self.robots:
            try:
                # 1. Lookup continuous transforms from TF Tree
                t_map_base = self.tf_buffer.lookup_transform("map", f"{r_name}/base_footprint", Time())
                t_map_odom = self.tf_buffer.lookup_transform("map", f"{r_name}/odom", Time())
                t_odom_base = self.tf_buffer.lookup_transform(f"{r_name}/odom", f"{r_name}/base_footprint", Time())

                # 2. Calculate latency / age of transforms (in seconds)
                lat_map_base = (now - Time.from_msg(t_map_base.header.stamp)).nanoseconds / 1e9
                lat_map_odom = (now - Time.from_msg(t_map_odom.header.stamp)).nanoseconds / 1e9
                lat_odom_base = (now - Time.from_msg(t_odom_base.header.stamp)).nanoseconds / 1e9

                # 3. Extract physical Yaw from map -> base_footprint transform
                tf_yaw = self._euler_from_quaternion(t_map_base.transform.rotation)

                # 4. Calculate Yaw offset between AMCL topic and physical TF tree
                amcl_yaw = self.amcl_latest_yaw[r_name]
                diff = tf_yaw - amcl_yaw
                diff = math.atan2(math.sin(diff), math.cos(diff))  # Normalize between [-pi, pi]
                yaw_offset_deg = abs(math.degrees(diff))

                # 5. Retrieve AMCL Covariance (Sigma)
                sigma = self.amcl_covariance_sigma[r_name]

                # 6. Append to log registers
                self.logs[r_name]["timestamps"].append(now_sec)
                self.logs[r_name]["tf_map_base_latency"].append(lat_map_base)
                self.logs[r_name]["tf_map_odom_latency"].append(lat_map_odom)
                self.logs[r_name]["tf_odom_base_latency"].append(lat_odom_base)
                self.logs[r_name]["yaw_offset_deg"].append(yaw_offset_deg)
                self.logs[r_name]["amcl_covariance_sigma"].append(sigma)

                # 7. Check for critical discrepancy thresholds
                if lat_map_base > 0.150 or yaw_offset_deg > 5.0 or sigma > 0.35:
                    event = {
                        "timestamp": now_sec,
                        "robot_name": r_name,
                        "state": self.robot_nav_states[r_name],
                        "tf_latency_ms": round(lat_map_base * 1000.0, 1),
                        "yaw_offset_deg": round(yaw_offset_deg, 2),
                        "covariance_sigma": round(sigma, 3),
                        "linear_x": self.logs[r_name]["ekf_linear_vel_x"][-1] if self.logs[r_name]["ekf_linear_vel_x"] else 0.0,
                        "angular_z": self.logs[r_name]["ekf_angular_vel_z"][-1] if self.logs[r_name]["ekf_angular_vel_z"] else 0.0
                    }
                    self.discrepancy_events.append(event)
                    self.get_logger().warn(
                        f"[ALERT] {r_name} in {event['state']}: "
                        f"Delay {event['tf_latency_ms']}ms | "
                        f"Yaw Offset {event['yaw_offset_deg']} deg | "
                        f"Sigma {event['covariance_sigma']}"
                    )

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                # TF transforms not populated yet, skip silently
                pass

    def _euler_from_quaternion(self, q) -> float:
        """Converts quaternion (x,y,z,w) orientation representation to Euler Yaw angle."""
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def generate_report(self):
        """Compiles statistical summary registers and writes the diagnostics document to disk."""
        print("\n[Diagnostics] Compiling flight record report. Writing to disk...")
        
        output_path = os.path.expanduser("~/thesis_ws/localization_diagnose_report.txt")
        
        try:
            with open(output_path, "w") as f:
                f.write("=========================================================================\n")
                f.write(" MULTI-ROBOT LOCALIZATION & TF TRANSITION FLIGHT DIAGNOSTICS REPORT\n")
                f.write(f" Compiled on: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())}\n")
                f.write("=========================================================================\n\n")

                for r_name in self.robots:
                    f.write(f" --- STATISTICAL REPORT FOR {r_name.upper()} ---\n")
                    
                    timestamps = self.logs[r_name]["timestamps"]
                    if not timestamps:
                        f.write("  Status: No telemetry recorded (transform lookup failed or offline).\n\n")
                        continue

                    # Average and maximum computations
                    avg_tf_base_lat = sum(self.logs[r_name]["tf_map_base_latency"]) / len(timestamps)
                    max_tf_base_lat = max(self.logs[r_name]["tf_map_base_latency"])
                    
                    avg_tf_odom_lat = sum(self.logs[r_name]["tf_map_odom_latency"]) / len(timestamps)
                    max_tf_odom_lat = max(self.logs[r_name]["tf_map_odom_latency"])
                    
                    avg_yaw_offset = sum(self.logs[r_name]["yaw_offset_deg"]) / len(timestamps)
                    max_yaw_offset = max(self.logs[r_name]["yaw_offset_deg"])
                    
                    avg_sigma = sum(self.logs[r_name]["amcl_covariance_sigma"]) / len(timestamps)
                    max_sigma = max(self.logs[r_name]["amcl_covariance_sigma"])

                    r_events = [e for e in self.discrepancy_events if e["robot_name"] == r_name]

                    f.write(f"  * Total Active Recording Frames : {len(timestamps)} ticks\n")
                    f.write(f"  * TF map->base_footprint Latency: Avg {avg_tf_base_lat*1000:.2f} ms | Max {max_tf_base_lat*1000:.2f} ms\n")
                    f.write(f"  * TF map->odom (AMCL) Latency    : Avg {avg_tf_odom_lat*1000:.2f} ms | Max {max_tf_odom_lat*1000:.2f} ms\n")
                    f.write(f"  * Yaw Offset (AMCL vs TF Tree)   : Avg {avg_yaw_offset:.3f} deg  | Max {max_yaw_offset:.3f} deg\n")
                    f.write(f"  * AMCL Estimation Sigma (spread) : Avg {avg_sigma:.4f}       | Max {max_sigma:.4f}\n")
                    f.write(f"  * Total Discrepancy Incidents   : {len(r_events)} events detected\n\n")

                f.write("=========================================================================\n")
                f.write(" DETECTED CRITICAL DISCREPANCY INCIDENTS (LOG CHRONOLOGY)\n")
                f.write("=========================================================================\n")
                
                if not self.discrepancy_events:
                    f.write("  No critical localization, latency, or orientation decoupling incidents detected.\n")
                else:
                    f.write(f"  {'Time (s)':<12} | {'Robot':<8} | {'State':<20} | {'TF Lat.':<10} | {'Yaw Off.':<10} | {'Sigma':<10}\n")
                    f.write("  " + "-"*75 + "\n")
                    for e in self.discrepancy_events:
                        f.write(
                            f"  {e['timestamp']:<12.1f} | "
                            f"{e['robot_name']:<8} | "
                            f"{e['state']:<20} | "
                            f"{str(e['tf_latency_ms']) + 'ms':<10} | "
                            f"{str(e['yaw_offset_deg']) + 'd':<10} | "
                            f"{e['covariance_sigma']:<10}\n"
                        )

            print(f"[Diagnostics] SUCCESS: Report saved to {output_path}")

        except Exception as e:
            print(f"[Diagnostics] ERROR: Failed to generate report file: {e}", file=sys.stderr)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationDiagnosticSuite()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.generate_report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()