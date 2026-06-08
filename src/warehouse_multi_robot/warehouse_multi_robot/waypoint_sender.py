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
from enum import Enum
from typing import Optional, Tuple, Dict, Any, List, Set

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

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
    EVALUATING_PATH = "EVALUATING_PATH"
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
        
        # Build section-specific lock key (e.g. shelf_2_yplus_D)
        side_key = f"{self.node.active_target_dict['side_key']}_{self.node.active_target_dict['section_label']}"
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
        
        # Publisher to trigger local and peer reallocations
        self.add_waypoints_pub = self.node.create_publisher(String, "add_waypoints", 10)

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
        """Performs task re-distribution for a failed peer robot."""
        # Clean up stale locks held by the failed peer
        expired_claims = []
        for side_key, claim in list(self.node.claim_mgr.active_claims.items()):
            if claim.get("robot_id") == failed_robot_name:
                expired_claims.append(side_key)
        for side_key in expired_claims:
            self.node.claim_mgr.active_claims.pop(side_key, None)

        failed_robot_spec = ROBOTS_SPECIFICATION.get(failed_robot_name)
        if not failed_robot_spec:
            return
            
        last_wp_id = self.node.health_monitor.robots_last_known_wp.get(failed_robot_name)
        last_wp_coordinates = None
        if last_wp_id:
            last_wp_coordinates = self._get_waypoint_coordinates_by_id(last_wp_id)

        uncompleted_waypoints_to_reallocate = []
        failed_robot_regions = failed_robot_spec["owned_regions"]
        
        for item_name, item in self.items_registry.items():
            if item["region"] not in failed_robot_regions:
                continue
                
            for side_name, side_spec in item["sides"].items():
                static_owner = ITEM_SIDE_OWNERS.get(side_spec["key"])
                if static_owner and static_owner != failed_robot_name and static_owner not in self.node.health_monitor.failed_robots:
                    continue
                
                labels = side_spec.get("section_labels_template") or side_spec.get("section_labels", ["A"])
                for label in labels:
                    section_key = f"{side_spec['key']}_{label}"
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
            return

        # Reallocation Payload
        payload = {
            "trigger_robot": self.node.robot_name,
            "failed_robot": failed_robot_name,
            "override_regions": True,
            "reference_coordinates": last_wp_coordinates,
            "waypoints": uncompleted_waypoints_to_reallocate
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

        self.side_lock_active = False
        
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
        # Using explicit absolute topic path to bypass any namespace relative resolution quirks
        absolute_status_topic = f"/{self.robot_name}/status"
        self.status_pub = self.create_publisher(String, absolute_status_topic, qos_status)
        
        qos_claims = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        qos_transient = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        
        self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self._amcl_callback, qos_transient)
        self.create_subscription(Bool, "/mission_armed", self._mission_armed_callback, qos_transient)
        self.create_subscription(String, "/side_claims", self._side_claims_callback, qos_claims)
        self.create_subscription(String, "/shelf_arrived", self._shelf_arrived_callback, qos_claims)
        self.create_subscription(String, "add_waypoints", self._add_waypoints_callback, 10)
        
        # Using explicit absolute status topic matching the battery monitor publisher exactly
        self.create_subscription(String, absolute_status_topic, self._local_status_callback, qos_transient)
        
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
        
        self.scan_delay_timer = None
        self._target_align_yaw: float = 0.0

        self.get_logger().info(f"[{self.robot_name}] WaypointSender Ready. First target: {self.spec['first_item']}")

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

        # Handle active tracking depending on the current state machine loop
        if self.current_state == RobotState.AMCL_RECOVERY_SPIN:
            self._accumulate_recovery_yaw()
        else:
            # Preventative active watchdog: trigger spin if localization drifts during active driving
            if not self.is_localization_fresh and self.current_state in (
                RobotState.NAVIGATING_TO_TARGET, RobotState.ALIGNING_YAW, RobotState.EVALUATING_PATH
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
        if self.current_state == RobotState.FAILED:
            return
            
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
            
        failed_robot = data.get("failed_robot")
        if failed_robot == self.robot_name:
            return
            
        self.get_logger().error(f"[{self.robot_name}] Peer {failed_robot} offline. Transitioning to GREEDY Mode!")
        self.greedy_mode = True
        
        ref = data.get("reference_coordinates")
        if ref:
            self.reallocation_ref_coords = (float(ref[0]), float(ref[1]))
            
        reallocated_wps = data.get("waypoints", [])
        for wp in reallocated_wps:
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
                
        if self.current_state in (RobotState.WAITING_FOR_MISSION, RobotState.IDLE):
            self.current_state = RobotState.DECISION_PHASE

    def _local_status_callback(self, msg: String):
        if msg.data == "FAILED" and self.current_state != RobotState.FAILED:
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
            deferred_side_keys=active_exclude_keys
        )

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
            self.get_logger().info(f"[{self.robot_name}] Mission Quota Satisfied ({self.completed_waypoints_count}/{self.spec['waypoint_limit']}). Returning to Spawn.")
            self.current_state = RobotState.IDLE
            self._return_to_spawn_coordinates()
            return

        self.active_target_dict = target
        mode_used = target["mode"]
        target_name = f"{target['item_name']}_{target['side_key']}_{target['section_label']}"
        self.get_logger().info(f"[{self.robot_name}] Target Acquired: {target_name} via {mode_used}")

        self.claim_mgr.publish_local_claim("claim")

        # Unlock side sweep tracking if we transition to a different side or a completely new item
        if mode_used != "SAME_SIDE_LOCK":
            self.side_lock_active = False

        # Only bypass candidate generation if actively locked on the side AND performing same-side sweeps
        if self.side_lock_active and mode_used in ("SAME_SIDE_LOCK", "SAME_ITEM_SIDE_TRANSITION"):
            self.chosen_candidate = (target["x"], target["y"])
            self.recovery_mgr.start_monitoring(self.current_x, self.current_y, is_candidate=False)
            self._dispatch_navigation()
        else:
            self._prepare_candidates(target["x"], target["y"])

    def _prepare_candidates(self, target_x: float, target_y: float):
        cand_a = (self.current_x, target_y)  # Moves strictly vertically (Y changes, X constant)
        cand_b = (target_x, self.current_y)  # Moves strictly horizontally (X changes, Y constant)
        
        if self.robot_name == "robot2":
            self.candidate_queue = [cand_b, cand_a]
        else:
            self.candidate_queue = [cand_a, cand_b]
            
        self.evaluating_candidate_index = 0
        self.current_state = RobotState.EVALUATING_PATH
        self._evaluate_next_candidate()

    def _evaluate_next_candidate(self):
        if self.evaluating_candidate_index >= len(self.candidate_queue):
            self.get_logger().warn(f"[{self.robot_name}] Both candidates blocked. Heading direct.")
            self.chosen_candidate = (self.active_target_dict["x"], self.active_target_dict["y"])
            self.recovery_mgr.start_monitoring(self.current_x, self.current_y, is_candidate=False)
            self._dispatch_navigation()
            return

        cand_x, cand_y = self.candidate_queue[self.evaluating_candidate_index]
        travel_yaw = math.atan2(cand_y - self.current_y, cand_x - self.current_x)
        qx, qy, qz, qw = self._yaw_to_quaternion(travel_yaw)

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = self._create_pose_stamped(cand_x, cand_y, qx, qy, qz, qw)
        goal_msg.use_start = False
        
        future = self.compute_path_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_path_request_accepted)

    def _on_path_request_accepted(self, future):
        # Gatekeeper: Prevent late asynchronous callback execution after failure or during active recovery spin
        if self.current_state in (RobotState.FAILED, RobotState.AMCL_RECOVERY_SPIN):
            return
            
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._next_candidate_fail()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_path_result_received)

    def _on_path_result_received(self, future):
        # Gatekeeper: Prevent late asynchronous callback execution after failure or during active recovery spin
        if self.current_state in (RobotState.FAILED, RobotState.AMCL_RECOVERY_SPIN):
            return
            
        try:
            result = future.result().result
            if result and len(result.path.poses) > 0:
                self.chosen_candidate = self.candidate_queue[self.evaluating_candidate_index]
                self.get_logger().info(f"[{self.robot_name}] Candidate index {self.evaluating_candidate_index} is VALID and REACHABLE.")
                self.recovery_mgr.start_monitoring(self.current_x, self.current_y, is_candidate=True)
                self._dispatch_navigation()
                return
        except Exception as e:
            self.get_logger().error(f"[{self.robot_name}] Candidate Evaluation Exception: {e}")
            
        self._next_candidate_fail()

    def _next_candidate_fail(self):
        self.evaluating_candidate_index += 1
        self._evaluate_next_candidate()

    def _dispatch_navigation(self):
        self.current_state = RobotState.NAVIGATING_TO_TARGET
        target_x, target_y = self.chosen_candidate
        
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
            target_x = self.active_target_dict["x"]
            target_y = self.active_target_dict["y"]
            
            if self.chosen_candidate == (target_x, target_y):
                self.recovery_mgr.stop_monitoring()
                self._dispatch_alignment_rotation()
            else:
                self.chosen_candidate = (target_x, target_y)
                self.recovery_mgr.start_monitoring(self.current_x, self.current_y, is_candidate=False)
                self._dispatch_navigation()
        elif status == 5:  # CANCELED
            # Bypass throttling on intentional recovery manager cancellations
            self.get_logger().info(f"[{self.robot_name}] Navigation intentionally canceled. Proceeding directly.")
            if self.recovery_mgr.state not in (RecoveryState.BACKING_UP, RecoveryState.ROTATING_ESCAPE):
                self.recovery_mgr.state = RecoveryState.IDLE
                self.current_state = RobotState.DECISION_PHASE
        else:
            self.get_logger().error(f"[{self.robot_name}] Navigation Action Ended with failure status: {status}. Throttling retry.")
            
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
        """Aligns robot perpendicularly to the shelf via cmd_vel P-controller."""
        self.current_state = RobotState.ALIGNING_YAW

        side_key = self.active_target_dict["side_key"]

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
        self.current_state = RobotState.SCANNING
        self.side_lock_active = True  # Target reached and aligned; lock on this side for subsequent sweeps
        self._stop_robot()
        
        if self.scan_delay_timer is not None:
            self.scan_delay_timer.cancel()
        self.scan_delay_timer = self.create_timer(SCAN_DURATION_SEC, self._on_scan_completed)

    def _on_scan_completed(self):
        # Gatekeeper: Prevent execution and state changes if the robot has already failed
        if self.current_state == RobotState.FAILED:
            if self.scan_delay_timer is not None:
                self.scan_delay_timer.cancel()
                self.scan_delay_timer = None
            return

        if self.scan_delay_timer is not None:
            self.scan_delay_timer.cancel()
            self.scan_delay_timer = None
            
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
            else:
                self.deferred_side_keys.add(side_key)
                
            self.active_target_dict = None
            
        self.side_lock_active = False  # Reset side lock completely upon task deferral
        self.current_state = RobotState.DECISION_PHASE

    def _publish_local_claim_renewals(self):
        self.claim_mgr.publish_renewals()

    def _publish_local_status(self, status_string: str):
        msg = String()
        msg.data = status_string
        self.status_pub.publish(msg)

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
        """Suspends active Nav2 goals, triggers AMCL global reset, and starts controlled recovery spin."""
        self.get_logger().error(
            f"[{self.robot_name}] AMCL DIVERGENCE DETECTED (Sigma: {self.amcl_covariance_sigma:.3f} > {self.max_amcl_pos_sigma:.3f}). "
            f"Suspending navigation to prevent collision. Initiating Global Particle Reset!"
        )
        # 1. Gracefully abort current Nav2 active path
        self._cancel_active_nav2_goal()
        self._stop_robot()
        
        # 2. Call AMCL global localization reinitialization service to scatter particles across the map
        if self.global_loc_client.service_is_ready():
            self.get_logger().info(f"[{self.robot_name}] Requesting AMCL global localization reset asynchronously.")
            self.global_loc_client.call_async(Empty.Request())
        else:
            self.get_logger().warn(f"[{self.robot_name}] AMCL reinitialize_global_localization service is not ready!")
        
        # 3. Prevent the recovery manager from issuing conflicting stall-recoveries
        self.recovery_mgr.stop_monitoring()
        
        # 4. Reset all tracking registers
        self.amcl_recovery_accumulated_yaw = 0.0
        self.amcl_recovery_prev_yaw = self.current_yaw
        self.amcl_recovery_stable_count = 0
        
        # 5. Jump to the recovery spin state
        self.current_state = RobotState.AMCL_RECOVERY_SPIN

    def _accumulate_recovery_yaw(self):
        """Accumulates relative rotation delta safely handling the wrapping between -PI and PI."""
        diff = self.current_yaw - self.amcl_recovery_prev_yaw
        diff = math.atan2(math.sin(diff), math.cos(diff))
        self.amcl_recovery_accumulated_yaw += abs(diff)
        self.amcl_recovery_prev_yaw = self.current_yaw

    def _handle_amcl_recovery_tick(self):
        """10Hz tick processing relative rotation accumulation and safe exit conditions."""
        # Condition 1: Verify if the robot has safely rotated at least 180 degrees (PI radians)
        has_turned_180_deg = self.amcl_recovery_accumulated_yaw >= math.pi
        
        # Condition 2: Check if AMCL covariance has remained stable below threshold for 3 ticks
        is_localization_stable = self.amcl_recovery_stable_count >= 3
        
        # Safe Early Exit Sweet Spot: Exit only if turned 180 deg AND stable
        if has_turned_180_deg and is_localization_stable:
            self.get_logger().info(
                f"[{self.robot_name}] AMCL RECOVERY SUCCESSFUL! "
                f"Accumulated Spin: {math.degrees(self.amcl_recovery_accumulated_yaw):.1f} deg. "
                f"Sigma: {self.amcl_covariance_sigma:.3f}. Resuming navigation via DECISION_PHASE."
            )
            self._stop_robot()
            self.current_state = RobotState.DECISION_PHASE
            return
            
        # Hard Timeout Gate: If robot spun full 360 degrees plus margin (6.45 rad) without convergence
        if self.amcl_recovery_accumulated_yaw >= 6.45:
            self.get_logger().error(
                f"[{self.robot_name}] AMCL RECOVERY EXHAUSTED! Turned 360 degrees without convergence. "
                f"Transitioning to DECISION_PHASE to replan/retry target."
            )
            self._stop_robot()
            self.current_state = RobotState.DECISION_PHASE
            return
            
        # Publish slow, controlled, safe rotation on cmd_vel
        cmd = Twist()
        cmd.angular.z = 0.3  # Safe speed (approx 17 degrees per second)
        self.cmd_vel_pub.publish(cmd)


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