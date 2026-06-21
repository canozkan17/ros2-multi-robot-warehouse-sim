#!/usr/bin/env python3
"""
diagnosis.py - Decentralized Multi-Agent System Diagnosis Suite (v1.1 - Event Driven).

This diagnostic node acts as a high-fidelity flight data recorder. It eliminates 
10Hz logging noise, captures dynamic status transitions, claims, and completions,
tracks uncompleted task deferrals, and prints structured system summaries.
"""

import os
import sys
import math
import time
import json
import csv
from typing import Dict, Tuple, Optional, Set, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String, Bool

# Inject workspace source path to cleanly resolve local imports
sys.path.append(os.path.expanduser("~/thesis_ws/src/warehouse_multi_robot"))
try:
    from warehouse_multi_robot.sdf_parser import parse_sdf
    from warehouse_multi_robot.config import SDF_WORLD_PATH, ROBOTS_SPECIFICATION
except ImportError as e:
    print(f"[Diagnosis Init Error] Failed to resolve local imports: {e}")
    sys.exit(1)


class MultiRobotSystemDiagnosis(Node):
    def __init__(self):
        super().__init__("system_diagnosis")
        
        self.get_logger().info("==================================================")
        self.get_logger().info("[Diagnosis] Initializing Event-Driven Flight Recorder (v1.1)")
        self.get_logger().info("==================================================")

        # 1. Physical Layout Ground-Truth Configuration (Corrected Boundaries)
        self.warehouse_x_min, self.warehouse_x_max = -15.0, 15.0
        self.warehouse_y_min, self.warehouse_y_max = -28.0, 26.0  # Extended to 26.0 to clear shelf_big_1
        self.collision_proximity_threshold_m = 0.54

        # 2. Parse SDF World & Verify Ground-Truth Waypoints Spatially
        self.items_registry = parse_sdf(SDF_WORLD_PATH)
        self._verify_ground_truth_waypoints()

        # 3. Dynamic Positional Tracking Registers
        self.agents_pose: Dict[str, Tuple[float, float, float]] = {
            "robot1": (0.0, 0.0, 0.0),
            "robot2": (0.0, 0.0, 0.0),
            "robot3": (0.0, 0.0, 0.0),
        }
        self.agents_last_update: Dict[str, float] = {
            "robot1": 0.0,
            "robot2": 0.0,
            "robot3": 0.0,
        }

        # 4. State, Claims, and Task Registers
        self.active_claims: Dict[str, Dict[str, Any]] = {}  # key: section_key, val: {robot_id, expire_at_ns}
        self.completed_waypoints_global: Set[str] = set()
        self.deferred_waypoints_global: Set[str] = set()
        
        self.robot_statuses: Dict[str, str] = {
            "robot1": "BOOTSTRAP_AMCL",
            "robot2": "BOOTSTRAP_AMCL",
            "robot3": "BOOTSTRAP_AMCL",
        }
        self.robot_batteries: Dict[str, float] = {
            "robot1": 100.0,
            "robot2": 100.0,
            "robot3": 100.0,
        }
        self.robot_active_targets: Dict[str, str] = {
            "robot1": "None",
            "robot2": "None",
            "robot3": "None",
        }
        self.robot_status_timestamps: Dict[str, float] = {
            "robot1": 0.0,
            "robot2": 0.0,
            "robot3": 0.0,
        }

        # 5. Thesis Metrics Collection Registers
        self.total_target_waypoints = 142
        self.failure_time: Optional[float] = None
        self.failed_robot_name: Optional[str] = None
        self.pre_fail_completed_count: int = 0
        self.latency_recorded: bool = False
        self.coverage_recorded: bool = False

        # Chronometer registers for Total Warehouse Audit Time
        self.mission_start_time: Optional[float] = None
        self.mission_end_time: Optional[float] = None

        # Safety registers for Unique Near-Collision counting
        self.proximity_violation_count: int = 0
        self.logged_proximity_events: Set[str] = set()

        # Latency tracking dictionary for Anomaly Detection (arrival vs publication)
        self.shelf_arrival_timestamps: Dict[str, float] = {}

        self.latency_csv_path = "/home/canozkan/thesis_ws/reallocation_latency.csv"
        self.coverage_csv_path = "/home/canozkan/thesis_ws/coverage_loss.csv"
        self.anomaly_latency_csv_path = "/home/canozkan/thesis_ws/anomaly_latency.csv"
        self.total_audit_csv_path = "/home/canozkan/thesis_ws/total_audit_time.csv"

        # 6. Asynchronous Subscriptions Setup
        qos_transient = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        qos_claims = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Subscribe to robot poses, statuses, and reallocation messages
        for robot_name in self.agents_pose.keys():
            self.create_subscription(
                PoseWithCovarianceStamped,
                f"/{robot_name}/amcl_pose",
                lambda msg, r_name=robot_name: self._amcl_pose_callback(msg, r_name),
                qos_transient
            )
            # Subscribe to dedicated nav_status topic for true mission states
            self.create_subscription(
                String,
                f"/{robot_name}/nav_status",
                lambda msg, r_name=robot_name: self._robot_status_callback(msg, r_name),
                10
            )
            # Subscribe to status topic purely for battery logging
            self.create_subscription(
                String,
                f"/{robot_name}/status",
                lambda msg, r_name=robot_name: self._robot_battery_callback(msg, r_name),
                10
            )
            self.create_subscription(
                String,
                f"/{robot_name}/add_waypoints",
                lambda msg, r_name=robot_name: self._add_waypoints_callback(msg, r_name),
                10
            )

        # Global system topics
        self.create_subscription(String, "/side_claims", self._side_claims_callback, qos_claims)
        self.create_subscription(String, "/shelf_arrived", self._shelf_arrived_callback, qos_claims)

        # Subscribe to global mission start event to start chronometer
        self.create_subscription(
            Bool,
            "/mission_armed",
            self._mission_armed_callback,
            qos_transient
        )

        # Subscribe to dynamic anomaly alerts to measure classification latency
        self.create_subscription(
            String,
            "/anomaly_alert",
            self._anomaly_alert_callback,
            10
        )

        # 7. Low-Frequency Summary Trigger Timers
        self.diagnostic_timer = self.create_timer(1.0, self._diagnostic_tick)  # Run slower (1Hz) to save processing
        self.summary_timer = self.create_timer(10.0, self._print_system_state_summary)  # Print full report every 10s

        self.get_logger().info("[Diagnosis] Flight Recorder initialized. Quiet mode active.")

    def _verify_ground_truth_waypoints(self):
        """Validates parsed SDF waypoints against physical bounds on initialization."""
        total_waypoints = 0
        invalid_waypoints = 0

        for item_name, item in self.items_registry.items():
            for side_name, side_spec in item.get("sides", {}).items():
                points = side_spec.get("section_points_template", [])
                for gx, gy, label in points:
                    total_waypoints += 1
                    if not (self.warehouse_x_min <= gx <= self.warehouse_x_max) or \
                       not (self.warehouse_y_min <= gy <= self.warehouse_y_max):
                        self.get_logger().error(
                            f"[SDF Geometrical Fault] Waypoint '{side_spec['key']}_{label}' at ({gx:.3f}, {gy:.3f}) "
                            f"lies OUTSIDE warehouse limits!"
                        )
                        invalid_waypoints += 1

        self.get_logger().info(
            f"[Diagnosis] Geometrical Verification Complete. Waypoints: {total_waypoints}, Invalid: {invalid_waypoints}"
        )

    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped, robot_name: str):
        """Asynchronously captures active estimated coordinates of target robot."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        self.agents_pose[robot_name] = (x, y, yaw)
        self.agents_last_update[robot_name] = time.time()

    def _robot_status_callback(self, msg: String, robot_name: str):
        """Monitors local status states and records timestamps to measure allocation latency."""
        prev_status = self.robot_statuses.get(robot_name, "UNKNOWN")
        
        # Parse consolidated JSON status or fallback safely to legacy string
        status_val = msg.data
        try:
            data = json.loads(msg.data)
            status_val = data.get("status", msg.data)
        except json.JSONDecodeError:
            pass
            
        new_status = status_val
        
        # Start chronometer fallback if first robot transitions from BOOTSTRAP to ACTIVE
        if new_status not in ("BOOTSTRAP_AMCL", "WAITING_FOR_MISSION") and self.mission_start_time is None:
            self.mission_start_time = time.time()
            self.get_logger().info(
                f"[Metrics Engine] Active movement detected. Start chronometer initiated at {self.mission_start_time:.3f}s."
            )
        
        if prev_status != new_status:
            self.robot_statuses[robot_name] = new_status
            self.robot_status_timestamps[robot_name] = time.time()
            self.get_logger().info(f"[EVENT] State Transition: {robot_name} changed '{prev_status}' -> '{new_status}'")
            self._print_system_state_summary()

        # Catch transitions to FAILED state to flag t_fail for Sprint 2 Task 4
        if new_status == "FAILED" and prev_status != "FAILED":
            if self.failure_time is None:
                self.failure_time = time.time()
                self.failed_robot_name = robot_name
                self.pre_fail_completed_count = len(self.completed_waypoints_global)
                self.get_logger().warn(
                    f"[Metrics Engine] Detected failure of '{robot_name}' at {self.failure_time:.3f}s. "
                    f"Completed prior to failure: {self.pre_fail_completed_count} waypoints."
                )

    def _robot_battery_callback(self, msg: String, robot_name: str):
        """Monitors battery levels from the hardware monitor."""
        try:
            data = json.loads(msg.data)
            self.robot_batteries[robot_name] = float(data.get("battery", 100.0))
        except (json.JSONDecodeError, ValueError):
            pass

    def _mission_armed_callback(self, msg: Bool):
        """Triggers the global mission duration chronometer start."""
        if msg.data and self.mission_start_time is None:
            self.mission_start_time = time.time()
            self.get_logger().info(
                f"[Metrics Engine] MISSION START DETECTED! Chronometer started at {self.mission_start_time:.3f}s."
            )

    def _anomaly_alert_callback(self, msg: String):
        """Calculates exact anomaly detection latency (arrival to alert) and writes to CSV."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        shelf_id = data.get("shelf_id")
        robot_id = data.get("robot_id")
        class_name = data.get("class_name", "damaged")

        if not shelf_id:
            return

        arrival_timestamp = self.shelf_arrival_timestamps.get(shelf_id)
        if arrival_timestamp:
            alert_timestamp = time.time()
            latency_sec = alert_timestamp - arrival_timestamp
            latency_ms = latency_sec * 1000.0  # conver to milliseconds
            model_used = data.get("model_used", "SVM")  # get the model from the message

            self.get_logger().info(
                f"[Metrics Engine] SUCCESS: Anomaly Alert Latency for '{shelf_id}' by {robot_id}: {latency_ms:.2f}ms"
            )

            # anomaly_latency.csv match
            headers = ["timestamp", "robot_id", "model_used", "latency_ms"]
            row = [
                time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
                str(robot_id),
                str(model_used),
                f"{latency_ms:.4f}"
            ]
            self._write_csv_entry(self.anomaly_latency_csv_path, headers, row)

    def _side_claims_callback(self, msg: String):
        """Parses real-time side claims to catch race conditions and conflicts."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        robot_id = data.get("robot_id")
        section_key = data.get("side_key")
        action = data.get("action")
        expire_at_ns = int(data.get("expire_at_ns", 0))

        if not robot_id or not section_key or not action:
            return

        now_ns = self.get_clock().now().nanoseconds

        if action == "release":
            # Potential Deferral Check: Claim is released but no waypoints on this side were completed!
            # Since section_key is now a side-level key, we verify if at least one section
            # of this side has been successfully inspected in our global registry.
            has_completed_any = any(wp.startswith(section_key) for wp in self.completed_waypoints_global)
            if not has_completed_any and section_key in self.active_claims:
                self.deferred_waypoints_global.add(section_key)
                self.get_logger().warn(
                    f"[EVENT - Task Deferral] {robot_id} RELEASED claim on side '{section_key}' WITHOUT completing any inspection!"
                )
                self._print_system_state_summary()
            
            self.active_claims.pop(section_key, None)
            if self.robot_active_targets.get(robot_id) == section_key:
                self.robot_active_targets[robot_id] = "None"

        elif action == "claim":
            existing_claim = self.active_claims.get(section_key)
            if existing_claim:
                if existing_claim["robot_id"] != robot_id and existing_claim["expire_at_ns"] > now_ns:
                    self.get_logger().error(
                        f"[CLAIM COLLISION] Conflict detected on '{section_key}'! "
                        f"Active lease held by '{existing_claim['robot_id']}', but '{robot_id}' attempted to claim!"
                    )
            
            self.active_claims[section_key] = {
                "robot_id": robot_id,
                "expire_at_ns": expire_at_ns
            }
            self.robot_active_targets[robot_id] = section_key
            self.get_logger().info(f"[EVENT] Claim Locked: {robot_id} registered target '{section_key}'")
            self._print_system_state_summary()

        elif action == "renew":
            # Renewals are kept silent to prevent terminal pollution, just update state
            self.active_claims[section_key] = {
                "robot_id": robot_id,
                "expire_at_ns": expire_at_ns
            }
            self.robot_active_targets[robot_id] = section_key

    def _shelf_arrived_callback(self, msg: String):
        """Tracks completions, registers arrival timestamps, and monitors redundant runs."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        robot_id = data.get("robot_id")
        shelf_id = data.get("shelf_id")

        if shelf_id:
            # Register exact arrival time to measure anomaly latency upon alert publication
            self.shelf_arrival_timestamps[shelf_id] = time.time()

        if not robot_id or not shelf_id:
            return

        if shelf_id in self.completed_waypoints_global:
            self.get_logger().error(
                f"[REDUNDANT INSPECTION] Redundancy detected! Waypoint '{shelf_id}' "
                f"was completed by '{robot_id}', but is already marked as DONE globally!"
            )
        else:
            self.completed_waypoints_global.add(shelf_id)
            # Remove from deferred registry if completed later
            self.deferred_waypoints_global.discard(shelf_id)
            self.get_logger().info(f"[EVENT] Task Completed: {robot_id} reached '{shelf_id}'")
            self._print_system_state_summary()

    def _add_waypoints_callback(self, msg: String, coordinator_robot_name: str):
        """Validates reallocation packets and calculates reallocation latency (Task 4)."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        failed_robot = data.get("failed_robot")
        if not failed_robot:
            return

        local_status = self.robot_statuses.get(failed_robot, "UNKNOWN")
        last_update_age = time.time() - self.robot_status_timestamps.get(failed_robot, 0.0)

        # Catch False Positives
        if local_status in ("ACTIVE", "IDLE") and last_update_age < 10.0:
            self.get_logger().error(
                f"======================================================================\n"
                f"[FALSE POSITIVE FAILURE DETECTED]\n"
                f"Coordinator on '{coordinator_robot_name}' triggered greedy reallocation for '{failed_robot}'!\n"
                f"However, '{failed_robot}' is still alive! Status: {local_status} (Last update: {last_update_age:.2f}s ago).\n"
                f"======================================================================"
            )

        # Latency Calculation
        if self.failure_time is not None and failed_robot == self.failed_robot_name and not self.latency_recorded:
            realloc_time = time.time()
            latency_sec = realloc_time - self.failure_time
            self.latency_recorded = True

            self.get_logger().warn(
                f"[Metrics Engine] SUCCESS: Re-allocation Latency measured: {latency_sec:.4f} seconds!"
            )

            headers = ["timestamp", "failed_robot", "coordinator_robot", "latency_sec"]
            row = [
                time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
                failed_robot,
                coordinator_robot_name,
                f"{latency_sec:.4f}"
            ]
            self._write_csv_entry(self.latency_csv_path, headers, row)

    def _write_csv_entry(self, filepath: str, headers: list, row: list):
        """Safely writes structured metric rows into target CSV documents."""
        file_exists = os.path.exists(filepath)
        try:
            with open(filepath, mode="a", newline="") as f:
                writer = csv.writer(f)
                if not file_exists:
                    writer.writerow(headers)
                writer.writerow(row)
        except Exception as e:
            self.get_logger().error(f"[Metrics Engine] Failed to write CSV entry to {filepath}: {e}")

    def _diagnostic_tick(self):
        """Executes safety boundaries, inter-robot proximity, and metric completions checks at 1Hz."""
        self._check_safety_boundaries()
        self._check_inter_robot_proximity()
        self._check_expired_claims()
        self._check_coverage_loss_completion()

    def _check_safety_boundaries(self):
        """Verifies if active robots are escaping valid coordinate limits."""
        for robot_name, (x, y, _) in self.agents_pose.items():
            if x == 0.0 and y == 0.0 and self.agents_last_update[robot_name] == 0.0:
                continue

            if not (self.warehouse_x_min <= x <= self.warehouse_x_max) or \
               not (self.warehouse_y_min <= y <= self.warehouse_y_max):
                self.get_logger().warn(
                    f"[Boundary Violation] {robot_name} has escaped safe bounds! Position: ({x:.3f}, {y:.3f})"
                )

    def _check_inter_robot_proximity(self):
        """Monitors real-time inter-robot distances to count unique proximity violations."""
        robot_names = list(self.agents_pose.keys())
        for i in range(len(robot_names)):
            for j in range(i + 1, len(robot_names)):
                r1, r2 = robot_names[i], robot_names[j]
                
                pos1 = self.agents_pose[r1]
                pos2 = self.agents_pose[r2]

                if self.agents_last_update[r1] == 0.0 or self.agents_last_update[r2] == 0.0:
                    continue

                distance = math.hypot(pos1[0] - pos2[0], pos1[1] - pos2[1])
                if distance < self.collision_proximity_threshold_m:
                    event_key = f"{min(r1, r2)}_{max(r1, r2)}"
                    
                    # Only count as a new unique violation if robots were not already close in previous ticks
                    if event_key not in self.logged_proximity_events:
                        self.logged_proximity_events.add(event_key)
                        self.proximity_violation_count += 1
                        self.get_logger().warn(
                            f"[Proximity Alert #{self.proximity_violation_count}] {r1} and {r2} are dangerously close! "
                            f"Distance: {distance:.3f}m"
                        )
                else:
                    event_key = f"{min(r1, r2)}_{max(r1, r2)}"
                    self.logged_proximity_events.discard(event_key)

    def _check_expired_claims(self):
        """Prunes expired claims silently so the diagnostic registries remain clean."""
        now_ns = self.get_clock().now().nanoseconds
        expired_keys = [
            k for k, v in self.active_claims.items() if v["expire_at_ns"] <= now_ns
        ]
        for k in expired_keys:
            self.active_claims.pop(k, None)

    def _check_coverage_loss_completion(self):
        """Checks for mission completion (all active survivors IDLE) under both failure and non-failure scenarios."""
        if self.mission_start_time is None or self.coverage_recorded:
            return

        # Active survivors are those not FAILED
        active_survivors = [
            r_name for r_name in self.agents_pose.keys() 
            if self.robot_statuses.get(r_name, "UNKNOWN") != "FAILED"
        ]
        
        # Wait until mission has actually initiated
        if not active_survivors:
            return

        all_idle = True
        has_started = False
        for r_name in active_survivors:
            status = self.robot_statuses.get(r_name, "UNKNOWN")
            if status not in ("IDLE", "BOOTSTRAP_AMCL"):
                has_started = True
            if status != "IDLE":
                all_idle = False

        # Guard: Ensure they have at least started executing waypoints to prevent early trigger
        waypoint_threshold = 10 if self.failed_robot_name else 100
        if all_idle and len(self.completed_waypoints_global) >= waypoint_threshold:
            self.coverage_recorded = True
            self.mission_end_time = time.time()
            total_duration_sec = self.mission_end_time - self.mission_start_time
            
            final_completed = len(self.completed_waypoints_global)
            coverage_percent = (final_completed / self.total_target_waypoints) * 100.0
            coverage_loss_count = self.total_target_waypoints - final_completed

            self.get_logger().warn(
                f"\n" + "="*50 + "\n"
                f" [Metrics Engine] CONGRATULATIONS! MISSION DURATION COMPLETE!\n"
                f" Total Run Duration   : {total_duration_sec:.2f} seconds\n"
                f" Waypoints Completed  : {final_completed}/{self.total_target_waypoints} ({coverage_percent:.2f}%)\n"
                f" Proximity Violations : {self.proximity_violation_count}\n"
                f"" + "="*50 + "\n"
            )

            # Write overall statistics to total_audit_time.csv
            headers_audit = ["timestamp", "failed_robot", "total_completed", "duration_sec", "proximity_violations"]
            row_audit = [
                time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
                str(self.failed_robot_name or "None"),
                str(final_completed),
                f"{total_duration_sec:.2f}",
                str(self.proximity_violation_count)
            ]
            self._write_csv_entry(self.total_audit_csv_path, headers_audit, row_audit)

            # Write specific coverage loss to coverage_loss.csv only if a failure occurred
            if self.failed_robot_name:
                headers = [
                    "timestamp", "failed_robot", "pre_fail_completed", 
                    "final_completed", "total_waypoints", "coverage_percent", "coverage_loss"
                ]
                row = [
                    time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
                    self.failed_robot_name,
                    str(self.pre_fail_completed_count),
                    str(final_completed),
                    str(self.total_target_waypoints),
                    f"{coverage_percent:.2f}",
                    str(coverage_loss_count)
                ]
                self._write_csv_entry(self.coverage_csv_path, headers, row)

    def _print_system_state_summary(self):
        """Prints a high-density, low-noise ASCII summary of the complete system state."""
        total_completed = len(self.completed_waypoints_global)
        total_deferred = len(self.deferred_waypoints_global)
        progress_pct = (total_completed / self.total_target_waypoints) * 100.0

        print("\n" + "="*95)
        print(f" SYSTEM PROGRESS SUMMARY: {total_completed}/{self.total_target_waypoints} Completed ({progress_pct:.2f}%) | {total_deferred} Deferred")
        print("="*95 + "\n")
        print(f" {'Robot':<10} | {'Status':<25} | {'Battery (%)':<12} | {'Active Target':<35} | {'Pose (X, Y, Yaw)':<20}")
        print("-"*110)
        for r_name in sorted(self.agents_pose.keys()):
            status = self.robot_statuses.get(r_name, "UNKNOWN")
            battery = self.robot_batteries.get(r_name, 100.0)
            target = self.robot_active_targets.get(r_name, "None")
            x, y, yaw = self.agents_pose[r_name]
            pos_str = f"({x:6.2f}, {y:6.2f}, {yaw:5.2f})" if self.agents_last_update[r_name] > 0 else "Uninitialized"
            print(f" {r_name:<10} | {status:<25} | {battery:<12.2f} | {target:<35} | {pos_str:<20}")
        
        print("-"*95)
        if self.active_claims:
            claims_list = [f"{k} ({v['robot_id']})" for k, v in self.active_claims.items()]
            print(f" Active Locks : {', '.join(claims_list)}")
        else:
            print(" Active Locks : None")

        if self.deferred_waypoints_global:
            print(f" Deferred WPs : {', '.join(sorted(self.deferred_waypoints_global))}")
        print("="*95 + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotSystemDiagnosis()
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