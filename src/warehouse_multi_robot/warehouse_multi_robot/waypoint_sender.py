#!/usr/bin/env python3
"""
waypoint_sender.py - Unified Multi-Agent Core Navigation & Coordination Node (Part 1).

This node consolidates waypoint sender and agent coordinator functionalities 
into a single process. It uses modular helper classes to perform decentralized 
heartbeat checking, task locking (claims), and task re-allocation locally in memory 
with zero network latency.
"""

import math
import json
import time
import os
import pickle
import hashlib
import threading
import csv
from enum import Enum
from typing import Optional, Tuple, Dict, Any, List, Set

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

# --- SPRINT 3 ML CORDIAL LIBRARIES ---
import cv2
import numpy as np
from skimage.feature import local_binary_pattern, hog

# --- ROS 2 IMAGE TRANSPORT ---
from sensor_msgs.msg import Image

from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from std_msgs.msg import String, Bool
from std_srvs.srv import Empty
from nav2_msgs.action import NavigateToPose, ComputePathToPose

# Pure logic & Helper imports
from warehouse_multi_robot.config import (
    SDF_WORLD_PATH, ROBOTS_SPECIFICATION, SCAN_DURATION_SEC, 
    NAVIGATION_FINAL_GOAL_TOLERANCE_M, ITEM_SIDE_OWNERS
)
from warehouse_multi_robot.sdf_parser import parse_sdf
from warehouse_multi_robot.item_selection import select_next_target
from warehouse_multi_robot.recovery import RecoveryManager, RecoveryState


# ==============================================================================
# (MOVEMENT STATES)
# ==============================================================================
class RobotState(Enum):
    BOOTSTRAP_AMCL = "BOOTSTRAP_AMCL"
    WAITING_FOR_MISSION = "WAITING_FOR_MISSION"
    DECISION_PHASE = "DECISION_PHASE"
    NAVIGATING_TO_TARGET = "NAVIGATING_TO_TARGET"
    ALIGNING_YAW = "ALIGNING_YAW"
    SCANNING = "SCANNING"
    FAILED = "FAILED"
    IDLE = "IDLE"
    THROTTLED_RETRY = "THROTTLED_RETRY"
    AMCL_RECOVERY_SPIN = "AMCL_RECOVERY_SPIN"


# ==============================================================================
# 1. YARDIMCI BEYİN: FLEET HEALTH MONITOR (Sağlık ve Canlılık Takibi)
# ==============================================================================
class FleetHealthMonitor:
    def __init__(self, node: Node):
        """
        Monitors health signals from other fleet agents.
        """
        self.node = node
        self.robots_heartbeat_registry: Dict[str, int] = {}
        self.robots_last_known_wp: Dict[str, str] = {}
        self.failed_robots: Set[str] = set()

    def register_heartbeat(self, robot_id: str, timestamp_ns: int):
        """Registers heartbeat timestamp from a peer claim/renew action."""
        if robot_id != self.node.robot_name:
            self.robots_heartbeat_registry[robot_id] = timestamp_ns
            if robot_id in self.failed_robots:
                self.failed_robots.discard(robot_id)

    def register_last_known_wp(self, robot_id: str, shelf_id: str):
        """Updates the last known shelf position for a peer robot."""
        self.robots_last_known_wp[robot_id] = shelf_id

    def check_fleet_health(self):
        """Watchdog running inside 1Hz timer to detect lost peers."""
        if self.node.current_state == RobotState.FAILED:
            return
            
        now_ns = int(self.node.get_clock().now().nanoseconds)
        lease_timeout_ns = 5 * 1_000_000_000  # 5 seconds expiration threshold
        
        for other_robot, last_timestamp in list(self.robots_heartbeat_registry.items()):
            if other_robot == self.node.robot_name or other_robot in self.failed_robots:
                continue
                
            elapsed_ns = now_ns - last_timestamp
            if elapsed_ns > lease_timeout_ns:
                self.node.get_logger().error(
                    f"[{self.node.robot_name}] HEARTBEAT LOST for {other_robot}. "
                    f"Triggering decentralized reallocation!"
                )
                self.failed_robots.add(other_robot)
                self.node.task_coord.trigger_reallocation(other_robot)


# ==============================================================================
# 2. YARDIMCI BEYİN: CLAIM MANAGER (Kilit ve Rezerve Yönetimi)
# ==============================================================================
class ClaimManager:
    def __init__(self, node: Node):
        """
        Manages spatial mutex claims (locking/releasing waypoints).
        """
        self.node = node
        self.active_claims: Dict[str, Dict[str, Any]] = {}
        self.local_active_claim_key: Optional[str] = None
        
        # QoS profiles matching global telemetry
        qos_claims = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.claims_pub = self.node.create_publisher(String, "/side_claims", qos_claims)

    def process_claim_message(self, side_key: str, robot_id: str, action: str, expire_at_ns: int):
        """Updates internal claims registry based on global claims messages."""
        if action == "release":
            self.active_claims.pop(side_key, None)
            if robot_id == self.node.robot_name:
                self.local_active_claim_key = None
        elif action in ("claim", "renew"):
            self.active_claims[side_key] = {
                "robot_id": robot_id,
                "expire_at_ns": expire_at_ns
            }
            if robot_id == self.node.robot_name:
                self.local_active_claim_key = side_key
        elif action == "blacklist":
            # Guard: If another robot blacklisted this shelf, record it and never visit it
            if robot_id != self.node.robot_name:
                self.node.get_logger().warn(
                    f"[{self.node.robot_name}] Received BLACKLIST propagation from {robot_id} "
                    f"for {side_key}. Permanently blocking task locally."
                )
                self.node.permanently_blocked_side_keys.add(side_key)
                self.node.deferred_side_keys.discard(side_key)
                self.active_claims.pop(side_key, None)

    def publish_local_claim(self, action_string: str):
        """Publishes own claims / renewals to the global topic."""
        if not self.node.active_target_dict:
            return
            
        now_ns = int(self.node.get_clock().now().nanoseconds)
        expire_at_ns = now_ns + (6 * 1_000_000_000)
        
        # Build side-specific (corridor-segment) lock key (e.g. shelf_big_3_xminus)
        side_key = self.node.active_target_dict['side_key']
        payload = {
            "robot_id": self.node.robot_name,
            "side_key": side_key,
            "action": action_string,
            "timestamp_ns": now_ns,
            "expire_at_ns": expire_at_ns
        }
        
        msg = String()
        msg.data = json.dumps(payload)
        self.claims_pub.publish(msg)

    def publish_renewals(self):
        """Timer-driven claim renewal generator."""
        if self.node.current_state == RobotState.FAILED:
            return
            
        # Keep renewals alive for any active state as long as we hold a local claim
        if self.local_active_claim_key:
            self.publish_local_claim("renew")

    def release_local_claim(self):
        """Releases own claims."""
        if not self.local_active_claim_key:
            return
            
        now_ns = int(self.node.get_clock().now().nanoseconds)
        payload = {
            "robot_id": self.node.robot_name,
            "side_key": self.local_active_claim_key,
            "action": "release",
            "timestamp_ns": now_ns,
            "expire_at_ns": now_ns
        }
        
        msg = String()
        msg.data = json.dumps(payload)
        self.claims_pub.publish(msg)
        
        self.active_claims.pop(self.local_active_claim_key, None)
        self.local_active_claim_key = None

    def publish_blacklist(self, side_key: str):
        """Publishes a blacklisted/blocked side to the fleet."""
        now_ns = int(self.node.get_clock().now().nanoseconds)
        payload = {
            "robot_id": self.node.robot_name,
            "side_key": side_key,
            "action": "blacklist",
            "timestamp_ns": now_ns,
            "expire_at_ns": now_ns + (3600 * 1_000_000_000)  # Effectively permanent (1 hour)
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.claims_pub.publish(msg)


# ==============================================================================
# 3. YARDIMCI BEYİN: TASK COORDINATOR (Harita, İş Takibi ve Paylaşımı)
# ==============================================================================
class TaskCoordinator:
    def __init__(self, node: "WaypointSenderNode"):
        """
        Manages the static map layout and handles decentralized task reallocation.
        """
        self.node = node
        self.items_registry = parse_sdf(SDF_WORLD_PATH)
        self.global_completed_waypoints: Set[str] = set()
        
        # Publisher changed to absolute/global to resolve DDS namespace isolation bug
        self.add_waypoints_pub = self.node.create_publisher(String, "/add_waypoints", 10)

    def process_arrival(self, robot_id: str, shelf_id: str):
        """Tracks global task completions to keep registries synchronized."""
        self.global_completed_waypoints.add(shelf_id)
        self.node.health_monitor.register_last_known_wp(robot_id, shelf_id)
        
        parts = shelf_id.split("_")
        if len(parts) >= 3:
            item_name = "_".join(parts[:-2])
            if item_name in self.items_registry:
                self.items_registry[item_name].setdefault("done", set()).add(shelf_id)
                
        # If we completed our own target, release our local kilit cleanly
        if robot_id == self.node.robot_name:
            if self.node.claim_mgr.local_active_claim_key == shelf_id:
                self.node.claim_mgr.release_local_claim()

    def trigger_reallocation(self, failed_robot_name: str):
        """
        [Global Task Pool Split]
        Performs battery-proportional re-distribution of the entire remaining 
        uncompleted task pool in the warehouse among active survivors.
        """
        # 1. Clean up stale locks held by the failed peer
        expired_claims = []
        for side_key, claim in list(self.node.claim_mgr.active_claims.items()):
            if claim.get("robot_id") == failed_robot_name:
                expired_claims.append(side_key)
        for side_key in expired_claims:
            self.node.claim_mgr.active_claims.pop(side_key, None)

        # 2. Identify active survivors
        all_robots = list(ROBOTS_SPECIFICATION.keys())
        survivors = [
            r for r in all_robots 
            if r != failed_robot_name and r not in self.node.health_monitor.failed_robots
        ]
        
        if not survivors:
            self.node.get_logger().error(f"[{self.node.robot_name}] No active survivors left to take over tasks!")
            return

        # 3. Gather ALL uncompleted unique waypoints in the entire warehouse
        uncompleted_waypoints_to_reallocate = []
        for item_name, item in self.items_registry.items():
            for side_name, side_spec in item["sides"].items():
                labels = side_spec.get("section_labels_template") or side_spec.get("section_labels", ["A"])
                for label in labels:
                    section_key = f"{side_spec['key']}_{label}"
                    # Skip if already completed globally
                    if section_key not in self.global_completed_waypoints:
                        coord = self._get_section_coordinates(side_spec, label)
                        if coord:
                            uncompleted_waypoints_to_reallocate.append({
                                "item_name": item_name,
                                "side_key": side_spec["key"],
                                "side": side_spec,
                                "section_label": label,
                                "x": coord[0],
                                "y": coord[1]
                            })

        if not uncompleted_waypoints_to_reallocate:
            self.node.get_logger().info(f"[{self.node.robot_name}] Global uncompleted task pool is already empty.")
            return

        # 4. Gather batteries and calculate proportional split ratios
        batteries = {s: self.node.peer_batteries.get(s, 100.0) for s in survivors}
        # Force current node to use its own updated local battery
        if self.node.robot_name in survivors:
            batteries[self.node.robot_name] = self.node.local_battery
            
        total_battery = sum(batteries.values())
        if total_battery <= 0.0:
            ratios = {s: 1.0 / len(survivors) for s in survivors}
        else:
            ratios = {s: batteries[s] / total_battery for s in survivors}

        # 5. Deterministically sort the global pool to ensure decentralized consistency
        # Both survivors must calculate exactly the same split sequence in parallel
        uncompleted_waypoints_to_reallocate.sort(
            key=lambda wp: (wp["item_name"], wp["side_key"], wp["section_label"])
        )

        # 6. Perform proportional split calculation (Deterministic Split)
        N = len(uncompleted_waypoints_to_reallocate)
        allocated_counts = {}
        remaining_tasks = N
        for i, s in enumerate(survivors):
            if i == len(survivors) - 1:
                allocated_counts[s] = remaining_tasks
            else:
                count = int(round(N * ratios[s]))
                allocated_counts[s] = count
                remaining_tasks -= count

        # 7. Populate pre-allocated lists inside payload
        allocations = {s: [] for s in survivors}
        task_idx = 0
        for s in survivors:
            count = allocated_counts[s]
            allocations[s] = uncompleted_waypoints_to_reallocate[task_idx : task_idx + count]
            task_idx += count

        self.node.get_logger().warn(
            f"[{self.node.robot_name}] GLOBAL TASK POOL SPLIT. Survivors: {survivors}, "
            f"Batteries: {batteries}, Remaining Tasks: {N}, Split: {allocated_counts}"
        )

        # Reallocation Payload
        payload = {
            "trigger_robot": self.node.robot_name,
            "failed_robot": failed_robot_name,
            "allocations": allocations
        }
        
        msg = String()
        msg.data = json.dumps(payload)
        self.add_waypoints_pub.publish(msg)

    def _get_waypoint_coordinates_by_id(self, wp_id: str) -> Optional[Tuple[float, float]]:
        parts = wp_id.split("_")
        if len(parts) < 3:
            return None
            
        item_name = "_".join(parts[:-2])
        section_label = parts[-1]
        side_name = parts[-2]

        item = self.items_registry.get(item_name)
        if not item:
            return None
            
        side_spec = item["sides"].get(side_name)
        if not side_spec:
            return None
            
        return self._get_section_coordinates(side_spec, section_label)

    def _get_section_coordinates(self, side_spec: Dict[str, Any], label: str) -> Optional[Tuple[float, float]]:
        for gx, gy, lbl in side_spec.get("section_points_template", []):
            if lbl == label:
                return gx, gy
        return None
# ==============================================================================
# 4. ANA GÖVDE: WAYPOINT SENDER NODE (Navigasyon Motoru ve ROS Düğümü)
# ==============================================================================
class WaypointSenderNode(Node):
    def __init__(self):
        super().__init__("waypoint_sender")
        
        self.declare_parameter("robot_name", "robot1")
        self.robot_name = self.get_parameter("robot_name").value
        self.spec = ROBOTS_SPECIFICATION[self.robot_name]
        
        self.declare_parameter("max_amcl_pos_sigma", 0.15)
        self.declare_parameter("max_amcl_age_sec", 5.0)
        self.declare_parameter("require_amcl_age_gate", False)
        self.declare_parameter("goal_reject_retry_sec", 1.0)
        
        self.max_amcl_pos_sigma = self.get_parameter("max_amcl_pos_sigma").value
        self.max_amcl_age_sec = self.get_parameter("max_amcl_age_sec").value
        self.require_amcl_age_gate = self.get_parameter("require_amcl_age_gate").value
        self.goal_reject_retry_sec = self.get_parameter("goal_reject_retry_sec").value
        
        # Initialize Core Helper Modules inside single memory space
        self.health_monitor = FleetHealthMonitor(self)
        self.claim_mgr = ClaimManager(self)
        self.task_coord = TaskCoordinator(self)
        
        self.current_state = RobotState.BOOTSTRAP_AMCL
        self.mission_armed = False
        self.completed_waypoints_count = 0
        
        self.greedy_mode = False
        self.reallocation_ref_coords = None
        self.deferred_side_keys = set()
        
        # Multi-deferral registers to prevent infinite loop
        self.permanently_blocked_side_keys = set()
        self.deferred_sides_count = {}

        # Registers for battery-aware allocation split and duplicate filters
        self.peer_batteries = {}
        self.reallocated_assigned_side_keys = set()
        self.processed_failed_robots = set()

        self.side_lock_active = False
        
        # Local battery and peer coordinate cache registers
        self.local_battery = 100.0
        self.peer_poses: Dict[str, Tuple[float, float]] = {}
        
        # ROI Goal-shifting trackers
        self.current_goal_is_shifted = False
        self.shifted_focus_point: Optional[Tuple[float, float]] = None
        
        self.active_target_dict: Optional[Dict[str, Any]] = {
            "item_name": self.spec["first_item"],
            "side_key": f"{self.spec['first_item']}_{self.spec['first_side']}"
        }
        
        self.candidate_queue: List[Tuple[float, float]] = []
        self.evaluating_candidate_index = 0
        self.chosen_candidate: Optional[Tuple[float, float]] = None
        
        self.current_x = self.spec["spawn_coordinates"][0]
        self.current_y = self.spec["spawn_coordinates"][1]
        self.current_yaw = 0.0
        self.amcl_covariance_sigma = float('inf')
        self.is_localization_fresh = False
        
        # AMCL active recovery spin registers
        self.amcl_recovery_accumulated_yaw = 0.0
        self.amcl_recovery_prev_yaw = 0.0
        self.amcl_recovery_stable_count = 0
        self.amcl_recovery_stage = 1  # Track recovery stage (1: Local, 2: Global)
        
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.compute_path_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.active_nav_goal_handle = None
        
        # Service client to reinitialize global localization when kidnapped or diverged
        self.global_loc_client = self.create_client(Empty, "reinitialize_global_localization")
        
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.shelf_arrived_pub = self.create_publisher(String, "/shelf_arrived", 10)
        self.completed_pub = self.create_publisher(String, "completed_items", 10)

        qos_status = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        # Master consolidated status publisher (for peers and battery monitor)
        self.status_pub = self.create_publisher(String, f"/{self.robot_name}/status", qos_status)
        # Nav status publisher (strictly for diagnosis and telemetry UI)
        self.nav_status_pub = self.create_publisher(String, f"/{self.robot_name}/nav_status", qos_status)
        
        qos_claims = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        qos_transient = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        
        self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self._amcl_callback, qos_transient)
        self.create_subscription(Bool, "/mission_armed", self._mission_armed_callback, qos_transient)
        self.create_subscription(String, "/side_claims", self._side_claims_callback, qos_claims)
        self.create_subscription(String, "/shelf_arrived", self._shelf_arrived_callback, qos_claims)
        
        # Changed to absolute/global topic subscription to resolve namespaced isolation
        self.create_subscription(String, "/add_waypoints", self._add_waypoints_callback, 10)
        
        # Subscribe to local battery monitor telemetry
        self.create_subscription(
            String, 
            f"/{self.robot_name}/battery_level", 
            self._local_battery_callback, 
            qos_transient
        )

        # Dynamically subscribe to all peer status topics to monitor battery and status consolidated JSONs
        for peer_name in ROBOTS_SPECIFICATION.keys():
            peer_status_topic = f"/{peer_name}/status"
            self.create_subscription(
                String,
                peer_status_topic,
                lambda msg, p_name=peer_name: self._peer_status_callback(msg, p_name),
                qos_transient
            )
        
        self.recovery_mgr = RecoveryManager(
            node=self,
            cmd_vel_publisher=self.cmd_vel_pub,
            nav2_cancel_callback=self._cancel_active_nav2_goal,
            nav2_retrigger_callback=self._retrigger_navigation,
            defer_task_callback=self._defer_active_task
        )
        
        self._retry_timer = None
        self.failure_cleanup_done = False  # Gatekeeper: Tracks if failure cleanup has executed once
        self.sm_timer = self.create_timer(0.1, self._state_machine_tick)
        self.claim_renewal_timer = self.create_timer(1.5, self._publish_local_claim_renewals)
        
        # New: Decoupled Fleet Health watchdog timer running at 1Hz
        self.heartbeat_monitor_timer = self.create_timer(1.0, self.health_monitor.check_fleet_health)
        
        # New: 1Hz Master Heartbeat timer to periodically broadcast status
        self.heartbeat_timer = self.create_timer(1.0, self._publish_heartbeat_tick)
        
        self.scan_delay_timer = None
        self._target_align_yaw: float = 0.0

        self.amcl_recovery_attempts = 0

        # --- SPRINT 3 ML PIPELINE PARAMETERS & PUBLISHERS ---
        self.declare_parameter("model_path", "")
        self.declare_parameter("dataset_path", "")
        
        self.anomaly_pub = self.create_publisher(String, "/anomaly_alert", 10)
        self._inference_in_progress = False
        self._inference_finished = False
        self._async_scan_thread = None

        # Asynchronously streams processed/XAI-overlaid views to PyQt5 UI
        self.image_pub = self.create_publisher(Image, "camera_view", 10)
        
        # Pre-load frozen ML assets and map available image paths safely
        self._load_ml_assets()
        self._pool_dataset_paths()
        # -----------------------------------------------------

        self.get_logger().info(f"[{self.robot_name}] WaypointSender Ready. First target: {self.spec['first_item']}")

    def _peer_status_callback(self, msg: String, peer_name: str):
        """Asynchronously parses peer status updates to extract heartbeats, battery, and failure."""
        try:
            data = json.loads(msg.data)
            status = data.get("status")
            battery = float(data.get("battery", 100.0))
            
            self.peer_batteries[peer_name] = battery
            
            # Cache peer coordinates if available in heartbeat
            px = data.get("x")
            py = data.get("y")
            if px is not None and py is not None:
                self.peer_poses[peer_name] = (float(px), float(py))
            
            if status in ("ACTIVE", "IDLE") and peer_name != self.robot_name:
                self.health_monitor.register_heartbeat(peer_name, int(self.get_clock().now().nanoseconds))
            
            # Instant failure trigger: If status is FAILED, trigger reallocation immediately
            elif status == "FAILED" and peer_name != self.robot_name:
                if peer_name not in self.health_monitor.failed_robots:
                    self.get_logger().error(
                        f"[{self.robot_name}] Peer {peer_name} reported FAILURE. Triggering instant reallocation!"
                    )
                    self.health_monitor.failed_robots.add(peer_name)
                    self.task_coord.trigger_reallocation(peer_name)
                
        except json.JSONDecodeError:
            # Fallback for old/legacy raw status strings
            if msg.data == "FAILED" and peer_name != self.robot_name:
                self.peer_batteries[peer_name] = 0.0
                if peer_name not in self.health_monitor.failed_robots:
                    self.get_logger().error(
                        f"[{self.robot_name}] Legacy string indicating failure for {peer_name}. Triggering instant reallocation!"
                    )
                    self.health_monitor.failed_robots.add(peer_name)
                    self.task_coord.trigger_reallocation(peer_name)


    def _amcl_callback(self, msg: PoseWithCovarianceStamped):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        
        var_x = abs(msg.pose.covariance[0])
        var_y = abs(msg.pose.covariance[7])
        self.amcl_covariance_sigma = math.hypot(math.sqrt(var_x), math.sqrt(var_y))
        
        self.is_localization_fresh = self.amcl_covariance_sigma < self.max_amcl_pos_sigma
        self.recovery_mgr.update_pose(self.current_x, self.current_y)

        # Track consecutive stable localization updates
        if self.is_localization_fresh:
            self.amcl_recovery_stable_count += 1
        else:
            self.amcl_recovery_stable_count = 0

        # Prevent any processing if the node is already flagged as failed
        if self.current_state == RobotState.FAILED:
            return

        
        # Preventative active watchdog: trigger spin if localization drifts during active driving
        if not self.is_localization_fresh and self.current_state in (
            RobotState.NAVIGATING_TO_TARGET, RobotState.ALIGNING_YAW
        ):
            self._trigger_amcl_recovery_spin()

    def _mission_armed_callback(self, msg: Bool):
        if msg.data and not self.mission_armed:
            self.mission_armed = True
            self.get_logger().info(f"[{self.robot_name}] Mission Armed received. Active.")

    def _side_claims_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
            
        side_key = data.get("side_key")
        robot_id = data.get("robot_id")
        action = data.get("action")
        timestamp_ns = int(data.get("timestamp_ns", 0))
        expire_at_ns = int(data.get("expire_at_ns", 0))
        
        if not side_key or not robot_id or not action:
            return
            
        # Unified callbacks directly updates internal modules
        self.health_monitor.register_heartbeat(robot_id, timestamp_ns)
        self.claim_mgr.process_claim_message(side_key, robot_id, action, expire_at_ns)

    def _shelf_arrived_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        robot_id = data.get("robot_id")
        shelf_id = data.get("shelf_id")

        if not robot_id or not shelf_id:
            return

        # Updates TaskCoordinator
        self.task_coord.process_arrival(robot_id, shelf_id)

    def _add_waypoints_callback(self, msg: String):
        """Processes peer re-allocation split payload and registers tasks to deferred queue."""
        if self.current_state == RobotState.FAILED:
            return
            
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
            
        failed_robot = data.get("failed_robot")
        if failed_robot == self.robot_name:
            return

        # Deduplication filter: prevent duplicate processing of the same failed peer
        if failed_robot in self.processed_failed_robots:
            return
        self.processed_failed_robots.add(failed_robot)
            
        self.get_logger().error(
            f"[{self.robot_name}] Peer {failed_robot} is OFFLINE. "
            f"Re-evaluating global task pool allocation..."
        )
        
        # Extract pre-allocated tasks computed by the peer
        allocations = data.get("allocations", {})
        my_reallocated_wps = allocations.get(self.robot_name, [])
        
        if not my_reallocated_wps:
            self.get_logger().info(f"[{self.robot_name}] Global reallocation allocated 0 new tasks to me.")
            return

        self.get_logger().warn(
            f"[{self.robot_name}] Assumed {len(my_reallocated_wps)} remaining tasks globally. "
            f"Will begin execution once original heuristic tasks are complete."
        )
        
        for wp in my_reallocated_wps:
            item_name = wp["item_name"]
            if item_name not in self.task_coord.items_registry:
                self.task_coord.items_registry[item_name] = {
                    "name": item_name,
                    "type": "reallocated",
                    "x": wp["x"],
                    "y": wp["y"],
                    "region": 1,
                    "sides": {wp["side_key"].split("_")[-1]: wp["side"]},
                    "done": set()
                }
            # Cache the specific reallocated side key to allow access during deferred mode
            self.reallocated_assigned_side_keys.add(wp["side_key"])
            
        # Dynamically set the new waypoint limit mathematically to completed + newly allocated remaining tasks
        # This completely eliminates any double-counting or regional overlap quota inflation!
        self.spec["waypoint_limit"] = self.completed_waypoints_count + len(my_reallocated_wps)
        
        self.get_logger().info(
            f"[{self.robot_name}] Waypoint limit mathematically updated: {self.completed_waypoints_count} completed "
            f"+ {len(my_reallocated_wps)} newly allocated = New Limit: {self.spec['waypoint_limit']}"
        )

    def _local_status_callback(self, msg: String):
        """Checks local battery telemetry payload to trigger failure cleanup."""
        status_string = msg.data
        try:
            data = json.loads(msg.data)
            status_string = data.get("status", msg.data)
        except json.JSONDecodeError:
            pass  # Fallback to legacy raw string
            
        if status_string == "FAILED" and self.current_state != RobotState.FAILED:
            self.get_logger().error(f"[{self.robot_name}] Local Status indicates FAILURE. Transitioning state.")
            self.current_state = RobotState.FAILED
            self._execute_failure_cleanup()

    def _execute_failure_cleanup(self):
        """
        Gatekeeper: Centralized one-shot cleanup routine.
        Ensures motor stop and claim releasing are executed exactly once.
        """
        if self.failure_cleanup_done:
            return
            
        self.get_logger().error(f"[{self.robot_name}] Executing centralized hardware failure cleanup.")
        
        # 1. Immediately halt all physical motion
        self._stop_robot()

        # Publish the FAILED status cleanly to the nav_status topic
        self._publish_local_status("FAILED")
        
        # 2. Cancel active Nav2 goal trajectories asynchronously
        self._cancel_active_nav2_goal()
        
        # 3. Halt the recovery manager watchdog to prevent automatic cmd_vel outputs
        self.recovery_mgr.stop_monitoring()
        
        # 4. Safely cancel all active timers to prevent asynchronous state leaks
        if self.scan_delay_timer is not None:
            try:
                self.scan_delay_timer.cancel()
            except Exception as e:
                self.get_logger().warn(f"Failed to cancel scan delay timer: {e}")
            self.scan_delay_timer = None

        if self._retry_timer is not None:
            try:
                self._retry_timer.cancel()
            except Exception as e:
                self.get_logger().warn(f"Failed to cancel retry timer: {e}")
            self._retry_timer = None
            
        # 5. Cleanly release spatial claim mutexes
        self.claim_mgr.release_local_claim()
        
        self.failure_cleanup_done = True

    def _state_machine_tick(self):
        if self.current_state == RobotState.FAILED:
            return
            
        self.recovery_mgr.tick(self.current_x, self.current_y)
        
        # State-Independent Corridor Mutex Watchdog: If we hold an active claim
        # and we are IDLE (returning to spawn), release the corridor lock only 
        # when we are physically outside of its boundary limits.
        if self.current_state == RobotState.IDLE and self.claim_mgr.local_active_claim_key:
            if self._has_cleared_current_corridor_segment():
                self.get_logger().info(
                    f"[{self.robot_name}] Physically cleared current corridor segment. Releasing segment lock safely."
                )
                self.claim_mgr.release_local_claim()
        
        if self.current_state == RobotState.BOOTSTRAP_AMCL:
            self._handle_bootstrap_amcl()
        elif self.current_state == RobotState.WAITING_FOR_MISSION:
            if self.mission_armed:
                self.current_state = RobotState.DECISION_PHASE
        elif self.current_state == RobotState.ALIGNING_YAW:
            self._handle_yaw_alignment_tick()
        elif self.current_state == RobotState.DECISION_PHASE:
            self._execute_decision_phase()
        elif self.current_state == RobotState.AMCL_RECOVERY_SPIN:
            self._handle_amcl_recovery_tick()
        elif self.current_state == RobotState.SCANNING:
            self._handle_scanning_tick()

    def _handle_bootstrap_amcl(self):
        if self.is_localization_fresh:
            self.get_logger().info(f"[{self.robot_name}] AMCL Stable (Sigma: {self.amcl_covariance_sigma:.3f}). Fleet ACTIVE.")
            self._stop_robot()
            self._publish_local_status("ACTIVE")
            self.current_state = RobotState.WAITING_FOR_MISSION
            return
            
        cmd = Twist()
        cmd.angular.z = 0.3
        self.cmd_vel_pub.publish(cmd)

    def _execute_decision_phase(self):
        now_ns = int(self.get_clock().now().nanoseconds)
        current_item = self.active_target_dict["item_name"] if self.active_target_dict else None
        current_side = self.active_target_dict["side_key"] if self.active_target_dict else None

        # first_item registry validation guard
        if current_item and current_item not in self.task_coord.items_registry:
            self.get_logger().error(
                f"[{self.robot_name}] CRITICAL: first_item '{current_item}' not found in items_registry. "
                f"SDF model name mismatch. Available keys: {list(self.task_coord.items_registry.keys())}"
            )
            self.active_target_dict = None
            return

        # Standard sweeps should actively exclude both temporarily deferred and permanently blocked tasks
        active_exclude_keys = self.deferred_side_keys.union(self.permanently_blocked_side_keys)

        target = select_next_target(
            items=self.task_coord.items_registry,
            robot_name=self.robot_name,
            current_x=self.current_x,
            current_y=self.current_y,
            owned_regions=self.spec["owned_regions"],
            priority_dirs=self.spec["priority_directions"],
            active_claims=self.claim_mgr.active_claims,
            now_ns=now_ns,
            completed_count=self.completed_waypoints_count,
            waypoint_limit=self.spec["waypoint_limit"],
            current_item_name=current_item,
            current_side_key=current_side,
            greedy_mode=self.greedy_mode,
            reallocation_reference_coords=self.reallocation_ref_coords,
            deferred_side_keys=active_exclude_keys,
            reallocated_assigned_side_keys=self.reallocated_assigned_side_keys
        )

        # (Deferred Greedy) Gateway
        if not target and self.reallocated_assigned_side_keys and not self.greedy_mode:
            self.get_logger().error(
                f"[{self.robot_name}] Original waypoint limit reached ({self.completed_waypoints_count}). "
                f"Transitioning to GREEDY_MODE now to process battery-split reallocations!"
            )
            self.greedy_mode = True
            
            # Re-run target selection immediately with greedy_mode enabled
            target = select_next_target(
                items=self.task_coord.items_registry,
                robot_name=self.robot_name,
                current_x=self.current_x,
                current_y=self.current_y,
                owned_regions=self.spec["owned_regions"],
                priority_dirs=self.spec["priority_directions"],
                active_claims=self.claim_mgr.active_claims,
                now_ns=now_ns,
                completed_count=self.completed_waypoints_count,
                waypoint_limit=self.spec["waypoint_limit"],
                current_item_name=current_item,
                current_side_key=current_side,
                greedy_mode=True,
                reallocation_reference_coords=self.reallocation_ref_coords,
                deferred_side_keys=self.permanently_blocked_side_keys,  # Exclude only permanently blocked
                reallocated_assigned_side_keys=self.reallocated_assigned_side_keys
            )

        # LAST RESORT RETRY FALLBACK:

        # LAST RESORT RETRY FALLBACK:
        # If no clean tasks remain but we have temporarily deferred tasks, retry them once before giving up
        if not target and self.deferred_side_keys:
            self.get_logger().warn(
                f"[{self.robot_name}] No standard tasks left. "
                f"Initiating Last-Resort Retry for deferred tasks: {list(self.deferred_side_keys)}"
            )
            # Re-run selection, but ONLY exclude permanently blocked ones (re-enabling temporarily deferred)
            target = select_next_target(
                items=self.task_coord.items_registry,
                robot_name=self.robot_name,
                current_x=self.current_x,
                current_y=self.current_y,
                owned_regions=self.spec["owned_regions"],
                priority_dirs=self.spec["priority_directions"],
                active_claims=self.claim_mgr.active_claims,
                now_ns=now_ns,
                completed_count=self.completed_waypoints_count,
                waypoint_limit=self.spec["waypoint_limit"],
                current_item_name=current_item,
                current_side_key=current_side,
                greedy_mode=self.greedy_mode,
                reallocation_reference_coords=self.reallocation_ref_coords,
                deferred_side_keys=self.permanently_blocked_side_keys
            )
            if target:
                side_key = target["side_key"]
                self.get_logger().info(f"[{self.robot_name}] Retrying deferred task: {side_key}")
                # Temporarily pop it from deferred list so it doesn't block its own current execution
                self.deferred_side_keys.discard(side_key)

        if not target:
            # Case A: If we reached our actual target waypoint limit, we are truly done!
            if self.completed_waypoints_count >= self.spec["waypoint_limit"]:
                # In both healthy and failure modes, we set state to IDLE and trigger spawn return.
                # If greedy_mode is True, we release the lock instantly.
                # If greedy_mode is False, we do NOT release the lock here; the background watchdog
                # in _state_machine_tick will release it only when we physically exit the corridor.
                if self.greedy_mode:
                    self.get_logger().info(
                        f"[{self.robot_name}] Mission Quota Satisfied in Greedy Mode. Releasing lock immediately."
                    )
                    self.claim_mgr.release_local_claim()
                else:
                    self.get_logger().info(
                        f"[{self.robot_name}] Mission Quota Satisfied. Initiating return to spawn with background corridor tracking..."
                    )
                
                # Publish the IDLE status cleanly to the nav_status topic
                self._publish_local_status("IDLE")
                self.current_state = RobotState.IDLE
                self._return_to_spawn_coordinates()
                return
            
            # Case B: If quota is NOT satisfied, but no tasks are free (locked by peers):
            # Enter standby (THROTTLED_RETRY) and poll periodically every 5.0 seconds.
            self.get_logger().warn(
                f"[{self.robot_name}] No free targets available, but quota is NOT satisfied "
                f"({self.completed_waypoints_count}/{self.spec['waypoint_limit']}). "
                f"Remaining tasks are locked by peers. Entering Standby Mode..."
            )
            self._stop_robot()
            self.current_state = RobotState.THROTTLED_RETRY
            
            # Dynamic standby polling timer to check lock releases
            if self._retry_timer is not None:
                self._retry_timer.cancel()
            self._retry_timer = self.create_timer(5.0, self._on_throttled_retry_timeout)
            return

        self.active_target_dict = target
        mode_used = target["mode"]
        target_name = f"{target['item_name']}_{target['side_key']}_{target['section_label']}"
        self.get_logger().info(f"[{self.robot_name}] Target Acquired: {target_name} via {mode_used}")

        # If we transition to a different side or a completely new item, release our old side-level claim first
        new_side_key = target["side_key"]
        if self.claim_mgr.local_active_claim_key and self.claim_mgr.local_active_claim_key != new_side_key:
            self.claim_mgr.release_local_claim()

        self.claim_mgr.publish_local_claim("claim")

        # Set the target coordinates as the direct navigation goal (Ablation optimization)
        # Completely bypassing the legacy Manhattan-style candidate routing mechanism.
        self.chosen_candidate = (target["x"], target["y"])
        self.recovery_mgr.start_monitoring(self.current_x, self.current_y, is_candidate=False)
        self._dispatch_navigation()

    def _dispatch_navigation(self):
        self.current_state = RobotState.NAVIGATING_TO_TARGET
        
        # Reset shifted goal flags
        self.current_goal_is_shifted = False
        self.shifted_focus_point = None
        
        target_x = self.active_target_dict["x"]
        target_y = self.active_target_dict["y"]
        side_key = self.active_target_dict["side_key"]
        
        # Determine perpendicular orientation angle based on side
        theta_perp = 0.0
        if "yplus" in side_key:
            theta_perp = -math.pi / 2.0
        elif "yminus" in side_key:
            theta_perp = math.pi / 2.0
        elif "xminus" in side_key:
            theta_perp = 0.0
        elif "xplus" in side_key:
            theta_perp = math.pi

        # Check if any dead robot is blocking our target standing point
        blocked_by_peer = False
        block_x, block_y = 0.0, 0.0
        failed_peer_name = "None"
        
        # D_safe: Parametric safe circular distance (Robot radius + sweeping margin + costmap inflation)
        D_safe = 0.85  
        
        for failed_peer in self.health_monitor.failed_robots:
            peer_pose = self.peer_poses.get(failed_peer)
            if peer_pose:
                dist = math.hypot(target_x - peer_pose[0], target_y - peer_pose[1])
                # If a dead robot is closer than D_safe to our target, it is blocked
                if dist < D_safe:
                    blocked_by_peer = True
                    block_x, block_y = peer_pose
                    failed_peer_name = failed_peer
                    break
                    
        if blocked_by_peer:
            self.get_logger().warn(
                f"[{self.robot_name}] Target {side_key} is blocked by failed {failed_peer_name} "
                f"at ({block_x:.2f}, {block_y:.2f}). Applying Mathematical Dynamic Goal Shifting..."
            )
            
            # Fetch item specs for geometric boundaries
            item_name = self.active_target_dict["item_name"]
            item = self.task_coord.items_registry.get(item_name, {})
            item_type = item.get("type", "shelf")
            item_x = item.get("x", 0.0)
            item_y = item.get("y", 0.0)
            
            # Calculate shift based on shelf orientation (parallel axis)
            if "yplus" in side_key or "yminus" in side_key:
                # Horizontal shelf -> Shift along X axis
                dy = abs(target_y - block_y)
                if dy < D_safe:
                    # Solve Pythagorean theorem to clear circle of radius D_safe
                    delta_x = math.sqrt(D_safe**2 - dy**2)
                    x_left = block_x - delta_x
                    x_right = block_x + delta_x
                    
                    sec_x = self.active_target_dict["x"]
                    
                    # If target falls inside the blocked interval, shift it
                    if x_left < target_x < x_right:
                        # Choose the boundary closer to the section center
                        if abs(x_left - sec_x) < abs(x_right - sec_x):
                            target_x = x_left
                        else:
                            target_x = x_right
                        
                        # Apply physical shelf geometric boundary safety valves
                        if item_type == "shelf":
                            min_val = item_x - 1.8
                            max_val = item_x + 1.8
                            if not (min_val <= target_x <= max_val):
                                self.get_logger().error(
                                    f"[{self.robot_name}] Shifted target_x ({target_x:.2f}) exceeds physical shelf boundaries! Deferring task."
                                )
                                self._defer_active_task()
                                return
            else:
                # Vertical shelf/pallet -> Shift along Y axis
                dx = abs(target_x - block_x)
                if dx < D_safe:
                    delta_y = math.sqrt(D_safe**2 - dx**2)
                    y_bottom = block_y - delta_y
                    y_top = block_y + delta_y
                    
                    sec_y = self.active_target_dict["y"]
                    
                    if y_bottom < target_y < y_top:
                        if abs(y_bottom - sec_y) < abs(y_top - sec_y):
                            target_y = y_bottom
                        else:
                            target_y = y_top
                        
                        # Apply physical big shelf geometric boundary safety valves
                        if item_type == "shelf_big":
                            min_val = item_y - 9.0
                            max_val = item_y + 9.0
                            if not (min_val <= target_y <= max_val):
                                self.get_logger().error(
                                    f"[{self.robot_name}] Shifted target_y ({target_y:.2f}) exceeds physical big shelf boundaries! Deferring task."
                                )
                                self._defer_active_task()
                                return

            # Perspective safety valve check
            if target_x != self.active_target_dict["x"] or target_y != self.active_target_dict["y"]:
                # Calculate shifted approach yaw towards actual section center
                align_yaw = math.atan2(
                    self.active_target_dict["y"] - target_y,
                    self.active_target_dict["x"] - target_x
                )
                angle_diff = abs(math.atan2(math.sin(align_yaw - theta_perp), math.cos(align_yaw - theta_perp)))
                
                # Max allowed angular deviation: 35.0 degrees to prevent perspective distortion
                max_dev_rad = 35.0 * math.pi / 180.0
                if angle_diff > max_dev_rad:
                    self.get_logger().error(
                        f"[{self.robot_name}] Perspective deviation ({math.degrees(angle_diff):.1f} deg) "
                        f"exceeds safety threshold ({math.degrees(max_dev_rad):.1f} deg)! Deferring task immediately."
                    )
                    self._defer_active_task()
                    return
                
                self.current_goal_is_shifted = True
                self.shifted_focus_point = (self.active_target_dict["x"], self.active_target_dict["y"])
                self.get_logger().info(
                    f"[{self.robot_name}] Goal dynamically shifted to: ({target_x:.2f}, {target_y:.2f}) "
                    f"focusing on original ROI center: ({self.active_target_dict['x']:.2f}, {self.active_target_dict['y']:.2f})"
                )
            
        self.chosen_candidate = (target_x, target_y)
        
        # Calculate travel heading yaw for navigation phase
        travel_yaw = math.atan2(target_y - self.current_y, target_x - self.current_x)
        qx, qy, qz, qw = self._yaw_to_quaternion(travel_yaw)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._create_pose_stamped(target_x, target_y, qx, qy, qz, qw)
        
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_nav_request_accepted)

    def _on_nav_request_accepted(self, future):
        # Gatekeeper: Prevent late asynchronous callback execution after failure or during active recovery spin
        if self.current_state in (RobotState.FAILED, RobotState.AMCL_RECOVERY_SPIN):
            return
            
        goal_handle = future.result()
        if not goal_handle.accepted:
            # Prevent unthrottled spinlock / thread starvation
            self.get_logger().warn(
                f"[{self.robot_name}] Active Nav goal rejected by Server. "
                f"Throttling retry for {self.goal_reject_retry_sec} seconds to prevent Executor starvation."
            )
            self.recovery_mgr.stop_monitoring()
            self._stop_robot()
            
            self.current_state = RobotState.THROTTLED_RETRY
            self._retry_timer = self.create_timer(
                self.goal_reject_retry_sec, 
                self._on_throttled_retry_timeout
            )
            return
            
        self.active_nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result_received)
    
    def _on_throttled_retry_timeout(self):
        """Timer callback triggered when the Nav2 rejection cooldown period finishes."""
        # Gatekeeper: Prevent state restoration if the robot is already in a FAILED state
        if self.current_state == RobotState.FAILED:
            if self._retry_timer:
                self._retry_timer.cancel()
                self._retry_timer = None
            return

        if self._retry_timer:
            self._retry_timer.cancel()
            self._retry_timer = None
            
        if self.current_state == RobotState.THROTTLED_RETRY:
            self.get_logger().info(f"[{self.robot_name}] Cooldown complete. Re-entering decision state.")
            self.current_state = RobotState.DECISION_PHASE

    def _on_nav_result_received(self, future):
        # Gatekeeper: Prevent late asynchronous callback execution after failure or during active recovery spin
        if self.current_state in (RobotState.FAILED, RobotState.AMCL_RECOVERY_SPIN):
            self.active_nav_goal_handle = None
            return

        self.active_nav_goal_handle = None
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.recovery_mgr.stop_monitoring()
            self._dispatch_alignment_rotation()
        elif status == 5:  # CANCELED
            # Bypass throttling on intentional recovery manager cancellations
            self.get_logger().info(f"[{self.robot_name}] Navigation intentionally canceled. Proceeding directly.")
            if self.recovery_mgr.state not in (RecoveryState.BACKING_UP, RecoveryState.ROTATING_ESCAPE):
                self.recovery_mgr.state = RecoveryState.IDLE
                self.current_state = RobotState.DECISION_PHASE
        else:
            self.get_logger().error(f"[{self.robot_name}] Navigation Action Ended with failure status: {status}.")
            
            # If the shifted goal failed, it means the corridor is fully blocked. Skip and defer immediately!
            if self.current_goal_is_shifted:
                self.get_logger().error(
                    f"[{self.robot_name}] Shifted bypass goal failed. Passage is completely blocked. Deferring task immediately!"
                )
                self._defer_active_task()
                return
            
            if self._retry_timer is not None:
                self._retry_timer.cancel()
                self._retry_timer = None
                
            if self.recovery_mgr.state not in (RecoveryState.BACKING_UP, RecoveryState.ROTATING_ESCAPE):
                self.recovery_mgr.state = RecoveryState.IDLE
                self.current_state = RobotState.THROTTLED_RETRY
                self._retry_timer = self.create_timer(
                    self.goal_reject_retry_sec, 
                    self._on_throttled_retry_timeout
                )

    def _dispatch_alignment_rotation(self):
        """Aligns robot focused on ROI section center if shifted, or perpendicularly via cmd_vel P-controller."""
        self.current_state = RobotState.ALIGNING_YAW
        side_key = self.active_target_dict["side_key"]

        # If the approach goal was shifted parallel to the shelf face, calculate dynamic ROI focal angle
        if self.current_goal_is_shifted and self.shifted_focus_point:
            focus_x, focus_y = self.shifted_focus_point
            self._target_align_yaw = math.atan2(focus_y - self.current_y, focus_x - self.current_x)
            self.get_logger().warn(
                f"[{self.robot_name}] Goal was shifted. Aligning to target ROI section center at "
                f"({focus_x:.2f}, {focus_y:.2f}) -> Dynamic Focus Yaw: {self._target_align_yaw:.3f} rad"
            )
        else:
            # Standard perpendicular yaw logic
            if "yplus" in side_key:
                self._target_align_yaw = -math.pi / 2.0
            elif "yminus" in side_key:
                self._target_align_yaw = math.pi / 2.0
            elif "xminus" in side_key:
                self._target_align_yaw = 0.0
            elif "xplus" in side_key:
                self._target_align_yaw = math.pi
            else:
                self._target_align_yaw = self.current_yaw

        self.get_logger().info(
            f"[{self.robot_name}] Aligning yaw to {self._target_align_yaw:.3f} rad via cmd_vel P-controller"
        )

    def _handle_yaw_alignment_tick(self):
        """10Hz P-controller tick for in-place yaw alignment."""
        YAW_KP = 1.5
        YAW_TOLERANCE = 0.05  # ~3 degrees
        MAX_ANG_VEL = 0.6

        error = self._target_align_yaw - self.current_yaw
        error = math.atan2(math.sin(error), math.cos(error))

        if abs(error) < YAW_TOLERANCE:
            self._stop_robot()
            self._start_scanning_process()
            return

        cmd = Twist()
        cmd.angular.z = max(-MAX_ANG_VEL, min(MAX_ANG_VEL, YAW_KP * error))
        self.cmd_vel_pub.publish(cmd)

    def _start_scanning_process(self):
        """Initiates the non-blocking asynchronous multi-level scanning process."""
        self.current_state = RobotState.SCANNING
        self.side_lock_active = True  # Target reached and aligned; lock on this side for subsequent sweeps
        self._stop_robot()
        
        self._inference_in_progress = True
        self._inference_finished = False
        
        # Spawn asynchronous thread to handle image injection, perturbations, and ML predictions
        self._async_scan_thread = threading.Thread(target=self._run_async_scanning_pipeline)
        self._async_scan_thread.daemon = True
        self._async_scan_thread.start()

    def _handle_scanning_tick(self):
        """Monitors the state of the asynchronous ML scanning background thread on the main ROS thread."""
        if self._inference_finished:
            # Complete the scan on the main thread safely
            self._inference_in_progress = False
            self._inference_finished = False
            if self._async_scan_thread:
                self._async_scan_thread.join()
                self._async_scan_thread = None
            self._on_scan_completed()

    def _on_scan_completed(self):
        # Reset recovery attempts counter on successful task completion
        self.amcl_recovery_attempts = 0
            
        self._publish_arrival_event()
        
        item_name = self.active_target_dict["item_name"]
        side_key = self.active_target_dict["side_key"]
        section_label = self.active_target_dict["section_label"]
        done_string = f"{side_key}_{section_label}"
        
        self.task_coord.items_registry[item_name].setdefault("done", set()).add(done_string)
        self.completed_waypoints_count += 1
        
        self.get_logger().info(f"[{self.robot_name}] Task Completed: {done_string} ({self.completed_waypoints_count}/{self.spec['waypoint_limit']})")
        self._publish_completed_summary()
        self.current_state = RobotState.DECISION_PHASE

    def _cancel_active_nav2_goal(self):
        if self.active_nav_goal_handle:
            try:
                self.get_logger().info(f"[{self.robot_name}] Aborting Nav2 path.")
                self.active_nav_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().error(
                    f"[{self.robot_name}] Exception during active Nav2 goal cancel request: {e}"
                )
            self.active_nav_goal_handle = None

    def _retrigger_navigation(self, go_direct: bool = False):
        if go_direct and self.active_target_dict:
            self.chosen_candidate = (self.active_target_dict["x"], self.active_target_dict["y"])
            self._dispatch_navigation()
        else:
            self.current_state = RobotState.DECISION_PHASE

    def _defer_active_task(self):
        self._cancel_active_nav2_goal()
        if self.active_target_dict:
            side_key = self.active_target_dict["side_key"]
            self.get_logger().error(f"[{self.robot_name}] Path persistently blocked. Deferring {side_key} task.")
            self.claim_mgr.release_local_claim()
            
            # Increment and track deferral counts
            count = self.deferred_sides_count.get(side_key, 0) + 1
            self.deferred_sides_count[side_key] = count
            
            if count >= 2:
                # If a task is deferred twice, block it permanently to prevent infinite loops
                self.get_logger().error(f"[{self.robot_name}] {side_key} permanently blocked after {count} deferrals.")
                self.permanently_blocked_side_keys.add(side_key)
                self.deferred_side_keys.discard(side_key)
                # Broadcast the blacklist action to the fleet!
                self.claim_mgr.publish_blacklist(side_key)
                
                # Dynamically reduce our waypoint limit by the number of sections on this blocked side
                # to prevent infinite standby loops when tasks become physically unreachable.
                item_name = self.active_target_dict["item_name"]
                item = self.task_coord.items_registry.get(item_name, {})
                side_spec = item.get("sides", {}).get(side_key.split("_")[-1], {})
                labels = side_spec.get("section_labels_template") or side_spec.get("section_labels", ["A"])

                uncompleted_assigned_sections = 0
                for label in labels:
                    section_key = f"{side_key}_{label}"
                    if section_key not in self.task_coord.global_completed_waypoints:
                        uncompleted_assigned_sections += 1
                
                self.spec["waypoint_limit"] -= uncompleted_assigned_sections
                self.get_logger().warn(
                    f"[{self.robot_name}] Waypoint limit adjusted accurately. Reduced by {uncompleted_assigned_sections} "
                    f"instead of full side sections ({len(labels)}). New limit: {self.spec['waypoint_limit']}"
                )
            else:
                self.deferred_side_keys.add(side_key)
                
            self.active_target_dict = None
            
        self.side_lock_active = False  # Reset side lock completely upon task deferral
        self.current_state = RobotState.DECISION_PHASE

    def _publish_local_claim_renewals(self):
        self.claim_mgr.publish_renewals()

    def _publish_local_status(self, status_string: str):
        """Publishes JSON consolidated status representation with pose coordinates to prevent type conflicts."""
        payload = {
            "status": status_string,
            "battery": round(self.local_battery, 2),
            "x": round(self.current_x, 2),
            "y": round(self.current_y, 2)
        }
        msg = String()
        msg.data = json.dumps(payload)
        
        # Publish to both topics to satisfy peers, battery monitor, and diagnosis
        self.status_pub.publish(msg)
        self.nav_status_pub.publish(msg)

    def _publish_heartbeat_tick(self):
        """Periodically broadcasts the consolidated master heartbeat to the fleet."""
        if self.current_state == RobotState.FAILED:
            self._publish_local_status("FAILED")
        elif self.current_state == RobotState.IDLE:
            self._publish_local_status("IDLE")
        else:
            self._publish_local_status("ACTIVE")

    def _local_battery_callback(self, msg: String):
        """Asynchronously updates local battery cache and handles failure triggers from the hardware monitor."""
        try:
            data = json.loads(msg.data)
            self.local_battery = float(data.get("battery", 100.0))
            is_failed = bool(data.get("is_failed", False))
            
            if is_failed and self.current_state != RobotState.FAILED:
                self.get_logger().error(f"[{self.robot_name}] Battery monitor reported critical hardware FAILURE!")
                self.current_state = RobotState.FAILED
                self._execute_failure_cleanup()
                
        except (json.JSONDecodeError, ValueError):
            pass

    def _publish_completed_summary(self):
        completed_keys = []
        for item in self.task_coord.items_registry.values():
            completed_keys.extend(list(item.get("done", set())))
            
        payload = {
            "robot_id": self.robot_name,
            "completed": completed_keys
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.completed_pub.publish(msg)

    def _return_to_spawn_coordinates(self):
        self.get_logger().info(f"[{self.robot_name}] Navigating back to initial spawn coordinates.")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._create_pose_stamped(
            self.spec["spawn_coordinates"][0],
            self.spec["spawn_coordinates"][1],
            0.0, 0.0, 0.0, 1.0
        )
        self.nav_to_pose_client.send_goal_async(goal_msg)

    def _stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def _yaw_to_quaternion(self, yaw: float) -> Tuple[float, float, float, float]:
        """Converts Euler Yaw to full 4-component Quaternion (x, y, z, w)."""
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return 0.0, 0.0, qz, qw

    def _create_pose_stamped(self, x: float, y: float, qx: float, qy: float, qz: float, qw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        return pose

    def _publish_arrival_event(self):
        payload = {
            "robot_id": self.robot_name,
            "shelf_id": f"{self.active_target_dict['side_key']}_{self.active_target_dict['section_label']}",
            "side_id": self.active_target_dict["side_key"],
            "section": self.active_target_dict["section_label"],
            "x": self.current_x,
            "y": self.current_y,
            "timestamp": int(self.get_clock().now().nanoseconds)
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.shelf_arrived_pub.publish(msg)
    
    def _trigger_amcl_recovery_spin(self):
        """Suspends active Nav2 goals and initiates Hierarchical Two-Stage Recovery."""
        self.get_logger().error(
            f"[{self.robot_name}] AMCL DIVERGENCE DETECTED (Sigma: {self.amcl_covariance_sigma:.3f} > {self.max_amcl_pos_sigma:.3f}). "
            f"Suspending navigation to prevent collision. Initiating Hierarchical Recovery Stage 1 (Local Spin)!"
        )
        self._cancel_active_nav2_goal()
        self._stop_robot()
        
        self.recovery_mgr.stop_monitoring()
        
        # Initialize Hierarchical State Parameters
        self.amcl_recovery_stage = 1  # Start with Stage 1 (Local Spin without resetting particles)
        self.amcl_recovery_start_time = self.get_clock().now().nanoseconds / 1e9
        self.amcl_recovery_stable_count = 0
        self.current_state = RobotState.AMCL_RECOVERY_SPIN
        
        # Increment recovery attempts for this target
        self.amcl_recovery_attempts += 1

    def _handle_amcl_recovery_tick(self):
        """10Hz tick processing relative rotation duration and safe exit conditions for hierarchical stages."""
        now_sec = self.get_clock().now().nanoseconds / 1e9
        elapsed_sec = now_sec - self.amcl_recovery_start_time
        
        has_turned_enough = elapsed_sec >= 8.0
        is_localization_stable = self.amcl_recovery_stable_count >= 3
        
        # Success check: If local or global AMCL has successfully stabilized
        if has_turned_enough and is_localization_stable:
            self.get_logger().info(
                f"[{self.robot_name}] AMCL RECOVERY SUCCESSFUL in Stage {self.amcl_recovery_stage}! "
                f"Elapsed Spin: {elapsed_sec:.1f}s. "
                f"Sigma: {self.amcl_covariance_sigma:.3f}. Resuming navigation via DECISION_PHASE."
            )
            self._stop_robot()
            self.current_state = RobotState.DECISION_PHASE
            return
            
        # Timeout/Transition Check at 22.0 seconds
        if elapsed_sec >= 22.0:
            self._stop_robot()
            
            # Stage 1 Timeout: If Stage 1 (Local Spin) failed, initiate Stage 2 (Global Reset Fallback)
            if self.amcl_recovery_stage == 1:
                self.get_logger().warn(
                    f"[{self.robot_name}] Stage 1 (Local Spin) Failed to converge in 22 seconds. "
                    f"Initiating Stage 2 (Global Reset Fallback)!"
                )
                self.amcl_recovery_stage = 2
                self.amcl_recovery_start_time = self.get_clock().now().nanoseconds / 1e9
                self.amcl_recovery_stable_count = 0
                
                # Request AMCL global localization reset asynchronously
                if self.global_loc_client.service_is_ready():
                    self.get_logger().info(f"[{self.robot_name}] Requesting AMCL global localization reset asynchronously.")
                    self.global_loc_client.call_async(Empty.Request())
                else:
                    self.get_logger().warn(f"[{self.robot_name}] AMCL reinitialize_global_localization service is not ready!")
                return
            
            # Stage 2 Timeout: If even Stage 2 (Global Reset) failed to converge
            else:
                self.get_logger().error(
                    f"[{self.robot_name}] Stage 2 (Global Reset) Failed to converge. Entire recovery sequence exhausted."
                )
                if self.amcl_recovery_attempts >= 2:
                    self.get_logger().error(
                        f"[{self.robot_name}] Recovery attempts exhausted twice. Deferring task to break deadlock."
                    )
                    self.amcl_recovery_attempts = 0
                    self._defer_active_task()
                else:
                    self.get_logger().error(
                        f"[{self.robot_name}] Retrying target via DECISION_PHASE."
                    )
                    self.current_state = RobotState.DECISION_PHASE
                return
                
        # Publish slow, controlled, safe rotation on cmd_vel
        cmd = Twist()
        cmd.angular.z = 0.3  # Safe speed (approx 17 degrees per second)
        self.cmd_vel_pub.publish(cmd)

    def _has_cleared_current_corridor_segment(self) -> bool:
        """Determines if the robot has physically cleared the boundary limits of its last inspected corridor."""
        if not self.active_target_dict:
            return True
            
        side_spec = self.active_target_dict.get("side_spec")
        if not side_spec:
            return True
            
        points = side_spec.get("section_points_template", [])
        if not points:
            return True
            
        # Get global endpoints of the corridor approach line
        pt_a = points[0]
        pt_b = points[-1]
        
        ax, ay = pt_a[0], pt_a[1]
        bx, by = pt_b[0], pt_b[1]
        
        # Corridor vector (v) and length
        vx = bx - ax
        vy = by - ay
        v_length = math.hypot(vx, vy)
        
        if v_length < 1e-4:
            return True
            
        # Unit direction vector (u) of the corridor
        ux = vx / v_length
        uy = vy / v_length
        
        # Vector from A to robot position (AP)
        apx = self.current_x - ax
        apy = self.current_y - ay
        
        # Project AP onto unit vector u to get linear progress t
        t = apx * ux + apy * uy
        
        # Retrieve physical offset specs to extend corridor bounds past the outer section points
        item_name = self.active_target_dict.get("item_name", "")
        item = self.task_coord.items_registry.get(item_name, {})
        item_type = item.get("type", "shelf")
        
        # Half section step allows the envelope to cover the full physical length of the shelf
        half_step = 2.25 if item_type == "shelf_big" else 0.45
        safety_margin = 0.5  # Extra 0.5 meters to ensure the robot has fully cleared the corner
        
        # Check if the robot has exited the extended longitudinal limits of the corridor
        exit_past_a = t < -(half_step + safety_margin)
        exit_past_b = t > (v_length + half_step + safety_margin)
        
        return exit_past_a or exit_past_b
    
    # ==============================================================================
    # SPRINT 3: DETAILED ML PIPELINE & SENSOR EMULATION CORE
    # ==============================================================================

    def _load_ml_assets(self):
        """Loads StandardScaler, PCA, SVM, and XGBoost models with fail-safe fallback logic."""
        self.svm_active = False
        self.xgb_active = False
        self.scaler = None
        self.pca = None
        self.svm_model = None
        self.xgb_model = None

        model_dir = self.get_parameter("model_path").value
        if not model_dir:
            model_dir = os.path.expanduser("~/thesis_ws/src/cardbox_dataset")

        self.get_logger().info(f"[{self.robot_name}] Loading ML pipeline assets from: {model_dir}")

        try:
            # Load Scaler
            scaler_path = os.path.join(model_dir, "cardbox_scaler.pkl")
            with open(scaler_path, "rb") as f:
                self.scaler = pickle.load(f)

            # Load PCA
            pca_path = os.path.join(model_dir, "cardbox_pca.pkl")
            with open(pca_path, "rb") as f:
                self.pca = pickle.load(f)

            # Load SVM (Primary)
            svm_path = os.path.join(model_dir, "cardbox_svm_model.pkl")
            with open(svm_path, "rb") as f:
                self.svm_model = pickle.load(f)
            self.svm_active = True
            self.get_logger().info(f"[{self.robot_name}] StandardScaler, PCA, and SVM loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"[{self.robot_name}] Failed to load primary SVM pipeline assets: {e}")

        try:
            # Load XGBoost (Fallback)
            from xgboost import XGBClassifier
            xgb_path = os.path.join(model_dir, "cardbox_xgb_model.json")
            self.xgb_model = XGBClassifier()
            self.xgb_model.load_model(xgb_path)
            self.xgb_active = True
            self.get_logger().info(f"[{self.robot_name}] Fallback XGBoost model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"[{self.robot_name}] Failed to load fallback XGBoost model: {e}")

        if not self.svm_active and not self.xgb_active:
            self.get_logger().error(f"[{self.robot_name}] CRITICAL: Both SVM and XGBoost models failed to load! Proceeding with mocked classification.")

    def _pool_dataset_paths(self):
        """Traverses the test and valid directories to register high-fidelity image paths into separate lists."""
        self.intact_paths = []
        self.damaged_paths = []

        dataset_dir = self.get_parameter("dataset_path").value
        if not dataset_dir:
            dataset_dir = os.path.expanduser("~/thesis_ws/src/cardbox_dataset")

        active_defect_classes = {1, 2, 3, 6}
        excluded_class = 4
        supported_extensions = (".jpg", ".jpeg", ".png", ".bmp")

        for split in ["test", "valid"]:
            images_dir = os.path.join(dataset_dir, split, "images")
            labels_dir = os.path.join(dataset_dir, split, "labels")

            if not os.path.exists(images_dir):
                continue

            for img_name in os.listdir(images_dir):
                if not img_name.lower().endswith(supported_extensions):
                    continue

                img_path = os.path.join(images_dir, img_name)
                base_name = os.path.splitext(img_name)[0]
                label_name = f"{base_name}.txt"
                label_path = os.path.join(labels_dir, label_name)

                contains_out_of_domain = False
                is_damaged = False

                if os.path.exists(label_path) and os.path.getsize(label_path) > 0:
                    with open(label_path, 'r') as f:
                        for line in f:
                            parts = line.strip().split()
                            if parts:
                                class_id = int(parts[0])
                                if class_id == excluded_class:
                                    contains_out_of_domain = True
                                    break
                                elif class_id in active_defect_classes:
                                    is_damaged = True

                if contains_out_of_domain:
                    continue

                if is_damaged:
                    self.damaged_paths.append(img_path)
                else:
                    self.intact_paths.append(img_path)

        self.get_logger().info(
            f"[{self.robot_name}] Dataset pooling complete. "
            f"Intact: {len(self.intact_paths)} paths, Damaged: {len(self.damaged_paths)} paths pooled."
        )

    def _extract_spatial_lbp(self, image, P=8, R=1, grid_rows=4, grid_cols=4):
        """Computes Local Binary Patterns (LBP) uniform histogram on a spatial grid."""
        lbp = local_binary_pattern(image, P, R, method='uniform')
        h, w = image.shape
        block_h = h // grid_rows
        block_w = w // grid_cols
        spatial_features = []
        
        for i in range(grid_rows):
            for j in range(grid_cols):
                block = lbp[i*block_h : (i+1)*block_h, j*block_w : (j+1)*block_w]
                hist, _ = np.histogram(
                    block.ravel(), 
                    bins=np.arange(0, P + 3), 
                    range=(0, P + 2), 
                    density=True
                )
                spatial_features.extend(hist)
        return np.array(spatial_features)

    def _extract_hog_features(self, image):
        """Extracts Histogram of Oriented Gradients (HOG) features."""
        features = hog(
            image,
            orientations=9,
            pixels_per_cell=(16, 16),
            cells_per_block=(2, 2),
            block_norm='L2-Hys',
            visualize=False
        )
        return features

    def _apply_physical_perturbations(self, image):
        """Simulates physical sensor noise, camera jitter, and lighting shifts on-the-fly."""
        h, w = image.shape
        
        # 1. Illumination Shift (Grayscale pixel values scaled by a factor of 0.95 to 1.05)
        scale_factor = np.random.uniform(0.95, 1.05)
        perturbed = np.clip(image * scale_factor, 0, 255).astype(np.uint8)
        
        # 2. Camera Jitter (Random micro rotation of +/- 2 degrees to simulate robotic stopping offsets)
        angle = np.random.uniform(-2.0, 2.0)
        M = cv2.getRotationMatrix2D((w / 2.0, h / 2.0), angle, 1.0)
        perturbed = cv2.warpAffine(perturbed, M, (w, h), borderMode=cv2.BORDER_REFLECT)
        
        # 3. Additive Gaussian Sensor Noise
        sigma = np.random.uniform(0.0, 5.0)
        if sigma > 0.0:
            noise = np.random.normal(0, sigma, image.shape)
            perturbed = np.clip(perturbed + noise, 0, 255).astype(np.uint8)
            
        return perturbed

    def _run_async_scanning_pipeline(self):
        """Asynchronously executes the multi-level Z-axis physical sweep and model inferences."""
        try:
            item_name = self.active_target_dict["item_name"]
            is_pallet = "pallet" in item_name.lower() or "mobile_cluster" in item_name.lower()
            max_levels = 1 if is_pallet else 3

            shelf_id = self.active_target_dict["side_key"]

            # STEP 2 EVALUATION: Physically scale climbing delay based on actual shelf geometry
            if is_pallet:
                level_climb_delay = 0.0
            elif "shelf_big" in item_name.lower():
                level_climb_delay = 4.0  # Safe indoor vertical climb for 6.0m warehouse shelving
            else:
                level_climb_delay = 1.5  # Standard vertical climb for 1.8m retail racks

            self.get_logger().info(
                f"[{self.robot_name}] Initiating async Z-axis scanning. "
                f"Item: {item_name}, Total Levels: {max_levels}, Climb Delay: {level_climb_delay}s"
            )

            latency_csv_path = os.path.expanduser("~/thesis_ws/anomaly_latency.csv")

            for level in range(1, max_levels + 1):
                # Simulated delay to represent drone physical vertical climbing/positioning
                if level_climb_delay > 0.0:
                    time.sleep(level_climb_delay)

                if self.current_state == RobotState.FAILED:
                    return

                # A. Deterministic MD5 Hashing (Option 3)
                scan_key = f"{shelf_id}_L{level}"
                hash_hex = hashlib.md5(scan_key.encode()).hexdigest()
                hash_int = int(hash_hex, 16)
                
                is_damaged = (hash_int % 100) < 30  # 30% anomaly rate

                # B. Pick corresponding image path safely from pool
                img_path = None
                if is_damaged and self.damaged_paths:
                    img_path = self.damaged_paths[hash_int % len(self.damaged_paths)]
                elif self.intact_paths:
                    img_path = self.intact_paths[hash_int % len(self.intact_paths)]

                if not img_path or not os.path.exists(img_path):
                    self.get_logger().warn(f"[{self.robot_name}] Image path not found. Using mock inference.")
                    continue

                # C. Read & Apply physical perturbations (Sensor Emulation)
                t_start = time.perf_counter()
                raw_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
                if raw_img is None:
                    continue
                resized_img = cv2.resize(raw_img, (128, 128))
                perturbed_img = self._apply_physical_perturbations(resized_img)

                # D. Extract Features and Transform (Inference Pipeline)
                lbp_feat = self._extract_spatial_lbp(perturbed_img)
                hog_feat = self._extract_hog_features(perturbed_img)
                fused_feat = np.hstack((lbp_feat, hog_feat)).reshape(1, -1)

                prediction = 0
                confidence = 1.0
                model_used = "Mocked"

                if self.scaler and self.pca:
                    scaled_feat = self.scaler.transform(fused_feat)
                    pca_feat = self.pca.transform(scaled_feat)

                    # E. Fail-safe Fallback Engine (Analytical Redundancy)
                    if self.svm_active and self.svm_model:
                        try:
                            prediction = int(self.svm_model.predict(pca_feat)[0])
                            try:
                                confidence = float(self.svm_model.predict_proba(pca_feat)[0][prediction])
                            except Exception:
                                confidence = 1.0
                            model_used = "SVM"
                        except Exception as e:
                            self.get_logger().error(f"[{self.robot_name}] Primary SVM inference failed: {e}. Falling back to XGBoost.")
                            model_used = "XGBoost Fallback"
                            if self.xgb_active and self.xgb_model:
                                prediction = int(self.xgb_model.predict(pca_feat)[0])
                                confidence = float(self.xgb_model.predict_proba(pca_feat)[0][prediction])
                    elif self.xgb_active and self.xgb_model:
                        prediction = int(self.xgb_model.predict(pca_feat)[0])
                        confidence = float(self.xgb_model.predict_proba(pca_feat)[0][prediction])
                        model_used = "XGBoost"

                t_end = time.perf_counter()
                latency_ms = (t_end - t_start) * 1000.0

                # STEP 3 GÖRSEL ENJEKSİYON: Reconstruct mathematically honest XAI Heatmaps on-the-fly
                output_image = perturbed_img
                if prediction == 1: # Only map spatial attention if defect is detected (anomalous decision)
                    output_image = self._generate_xai_overlay(perturbed_img)

                # Publish processed view asynchronously to the ROS 2 Image topic
                self._publish_ros_image(output_image)

                # F. Latency Profiling
                self._write_latency_to_csv(latency_csv_path, latency_ms, model_used)

                # G. Global Anomali Alarm Yayını (Publishing)
                self._publish_anomaly_alert(shelf_id, level, prediction, confidence, latency_ms, model_used)

                self.get_logger().info(
                    f"[{self.robot_name}] SCAN LEVEL {level}: Predict={prediction} (Conf={confidence:.2f}), "
                    f"Model={model_used}, Latency={latency_ms:.3f}ms"
                )

        except Exception as e:
            self.get_logger().error(f"[{self.robot_name}] Exception inside async scan thread: {e}")
        finally:
            self._inference_finished = True

    def _write_latency_to_csv(self, filepath, latency_ms, model_used):
        """Safely writes structured metric rows into target CSV documents."""
        file_exists = os.path.exists(filepath)
        try:
            with open(filepath, mode="a", newline="") as f:
                writer = csv.writer(f)
                if not file_exists:
                    writer.writerow(["timestamp", "robot_id", "model_used", "latency_ms"])
                writer.writerow([
                    time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
                    self.robot_name,
                    model_used,
                    f"{latency_ms:.4f}"
                ])
        except Exception as e:
            self.get_logger().error(f"[{self.robot_name}] Failed to write latency to CSV: {e}")

    def _publish_anomaly_alert(self, shelf_id, level, prediction, confidence, latency_ms, model_used):
        """Publishes the ML prediction outcome to the global fleet network."""
        payload = {
            "robot_id": self.robot_name,
            "shelf_id": shelf_id,
            "level": level,
            "class_name": "damaged" if prediction == 1 else "intact",
            "prediction": prediction,
            "confidence": confidence,
            "latency_ms": latency_ms,
            "model_used": model_used,
            "timestamp": int(self.get_clock().now().nanoseconds)
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.anomaly_pub.publish(msg)

    def _generate_xai_overlay(self, perturbed_img):
        """Generates mathematically honest Explainable AI (XAI) overlays based on SVM coefficients."""
        # Ensure perturbed_img is a 3-channel color image for overlay blending
        if len(perturbed_img.shape) == 2:
            perturbed_color = cv2.cvtColor(perturbed_img, cv2.COLOR_GRAY2BGR)
        else:
            perturbed_color = perturbed_img.copy()

        # Guard: Check if we have Scaler, PCA, Linear SVM, and valid coefficients
        if not self.scaler or not self.pca or not self.svm_active or not hasattr(self.svm_model, "coef_"):
            # If we cannot compute honest XAI (e.g. XGBoost is active), fallback to a clean sensor view
            return perturbed_color

        try:
            # 1. Project Linear SVM decision boundary weights back to original 1924-dim space (W_raw = W_pca * V^T)
            pca_weights = self.svm_model.coef_[0]
            raw_weights = np.dot(pca_weights, self.pca.components_)
            
            # 2. Rescale using StandardScaler's scaling array to undo scaling influence
            raw_weights_scaled = raw_weights / (self.scaler.scale_ + 1e-8)
            
            # 3. Extract the first 160 dimensions (Uniform Spatial LBP)
            lbp_weights_magnitude = np.abs(raw_weights_scaled[:160])
            
            # 4. Sum the 10 bins of each of the 16 blocks to calculate spatial grid importance
            block_importances = lbp_weights_magnitude.reshape(16, 10).sum(axis=1)
            grid_importance = block_importances.reshape(4, 4)
            
            # 5. Normalize and upscale 4x4 grid to 128x128 via bilinear interpolation
            grid_min, grid_max = grid_importance.min(), grid_importance.max()
            grid_norm = ((grid_importance - grid_min) / (grid_max - grid_min + 1e-8) * 255).astype(np.uint8)
            heatmap = cv2.resize(grid_norm, (128, 128), interpolation=cv2.INTER_LINEAR)
            
            # 6. Apply JET Colormap for visual overlay
            heatmap_color = cv2.applyColorMap(heatmap, cv2.COLORMAP_JET)
            
            # 7. Blend 65% original camera view with 35% XAI decision boundary heatmap
            overlay = cv2.addWeighted(perturbed_color, 0.65, heatmap_color, 0.35, 0)
            return overlay

        except Exception as e:
            self.get_logger().error(f"[{self.robot_name}] Failed to generate XAI heatmap: {e}")
            return perturbed_color

    def _publish_ros_image(self, cv_img):
        """Manually packs a CV2 image (BGR or Grayscale) into a ROS 2 Image message without CV Bridge dependency."""
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"{self.robot_name}/camera_optical_frame"
        
        if len(cv_img.shape) == 3: # BGR Color Image (XAI Overlay View)
            msg.height = cv_img.shape[0]
            msg.width = cv_img.shape[1]
            msg.encoding = "bgr8"
            msg.is_bigendian = 0
            msg.step = cv_img.shape[1] * 3
            msg.data = cv_img.tobytes()
        else: # Single-Channel Grayscale Image (Sensor View)
            msg.height = cv_img.shape[0]
            msg.width = cv_img.shape[1]
            msg.encoding = "mono8"
            msg.is_bigendian = 0
            msg.step = cv_img.shape[1]
            msg.data = cv_img.tobytes()
        
        self.image_pub.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = WaypointSenderNode()
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