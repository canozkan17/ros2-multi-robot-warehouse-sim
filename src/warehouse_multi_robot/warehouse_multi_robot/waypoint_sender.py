#!/usr/bin/env python3
"""
waypoint_sender.py - Final Integrated Core Navigation and State Machine Node.
"""

import math
import json
from enum import Enum
from typing import Optional, Tuple, Dict, Any, List, Set

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose, ComputePathToPose

# Pure logic & Helper imports
from warehouse_multi_robot.config import (
    SDF_WORLD_PATH, ROBOTS_SPECIFICATION, SCAN_DURATION_SEC, 
    NAVIGATION_FINAL_GOAL_TOLERANCE_M
)
from warehouse_multi_robot.sdf_parser import parse_sdf
from warehouse_multi_robot.item_selection import select_next_target
from warehouse_multi_robot.recovery import RecoveryManager, RecoveryState


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
        
        self.items_registry = parse_sdf(SDF_WORLD_PATH)
        
        self.current_state = RobotState.BOOTSTRAP_AMCL
        self.mission_armed = False
        self.completed_waypoints_count = 0
        
        self.greedy_mode = False
        self.reallocation_ref_coords: Optional[Tuple[float, float]] = None
        self.deferred_side_keys: Set[str] = set()

        self.side_lock_active = False
        
        # ======================================================================
        # FIX 2: BOOTSTRAP INITIAL TARGET LATCH (Exclusively Shelf_2 Lock)
        # ======================================================================
        self.active_target_dict: Optional[Dict[str, Any]] = {
            "item_name": self.spec["first_item"],
            "side_key": f"{self.spec['first_item']}_{self.spec['first_side']}"
        }
        self.active_claims: Dict[str, Dict[str, Any]] = {}
        
        self.candidate_queue: List[Tuple[float, float]] = []
        self.evaluating_candidate_index = 0
        self.chosen_candidate: Optional[Tuple[float, float]] = None
        
        self.current_x = self.spec["spawn_coordinates"][0]
        self.current_y = self.spec["spawn_coordinates"][1]
        self.current_yaw = 0.0
        self.amcl_covariance_sigma = float('inf')
        self.is_localization_fresh = False
        
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.compute_path_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.active_nav_goal_handle = None
        
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.shelf_arrived_pub = self.create_publisher(String, "/shelf_arrived", 10)
        self.completed_pub = self.create_publisher(String, "completed_items", 10)
        self.claims_pub = self.create_publisher(String, "/side_claims", 100)
        self.status_pub = self.create_publisher(String, "status", 10)
        
        qos_claims = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        qos_transient = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        
        self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self._amcl_callback, qos_transient)
        self.create_subscription(Bool, "/mission_armed", self._mission_armed_callback, qos_transient)
        self.create_subscription(String, "/side_claims", self._side_claims_callback, qos_claims)
        self.create_subscription(String, "/shelf_arrived", self._shelf_arrived_callback, qos_claims)
        self.create_subscription(String, "add_waypoints", self._add_waypoints_callback, 10)
        self.create_subscription(String, "status", self._local_status_callback, 10)
        
        self.recovery_mgr = RecoveryManager(
            node=self,
            cmd_vel_publisher=self.cmd_vel_pub,
            nav2_cancel_callback=self._cancel_active_nav2_goal,
            nav2_retrigger_callback=self._retrigger_navigation,
            defer_task_callback=self._defer_active_task
        )
        
        self.sm_timer = self.create_timer(0.1, self._state_machine_tick)
        self.claim_renewal_timer = self.create_timer(1.5, self._publish_local_claim_renewals)
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
        expire_at_ns = int(data.get("expire_at_ns", 0))
        
        if not side_key or not robot_id or not action:
            return
            
        if action == "release":
            self.active_claims.pop(side_key, None)
        elif action in ("claim", "renew"):
            self.active_claims[side_key] = {
                "robot_id": robot_id,
                "expire_at_ns": expire_at_ns
            }

    def _shelf_arrived_callback(self, msg: String):
        """Updates completion state for arrivals from OTHER robots only.
        Own completions are written directly in _on_scan_completed.
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        robot_id = data.get("robot_id")
        shelf_id = data.get("shelf_id")

        # Skip self-published events — already handled in _on_scan_completed
        if robot_id == self.robot_name or not shelf_id:
            return

        parts = shelf_id.split("_")
        if len(parts) >= 3:
            item_name = "_".join(parts[:-2])

            if item_name in self.items_registry:
                self.items_registry[item_name].setdefault("done", set()).add(shelf_id)

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
            if item_name not in self.items_registry:
                self.items_registry[item_name] = {
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
            self.get_logger().error(f"[{self.robot_name}] Local Status indicates FAILURE. Shutting down.")
            self.current_state = RobotState.FAILED
            self._stop_robot()
            self._cancel_active_nav2_goal()
            self._release_local_claim_publicly()

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
        if current_item and current_item not in self.items_registry:
            self.get_logger().error(
                f"[{self.robot_name}] CRITICAL: first_item '{current_item}' not found in items_registry. "
                f"SDF model name mismatch. Available keys: {list(self.items_registry.keys())}"
            )
            self.active_target_dict = None
            return

        target = select_next_target(
            items=self.items_registry,
            robot_name=self.robot_name,
            current_x=self.current_x,
            current_y=self.current_y,
            owned_regions=self.spec["owned_regions"],
            priority_dirs=self.spec["priority_directions"],
            active_claims=self.active_claims,
            now_ns=now_ns,
            completed_count=self.completed_waypoints_count,
            waypoint_limit=self.spec["waypoint_limit"],
            current_item_name=current_item,
            current_side_key=current_side,
            greedy_mode=self.greedy_mode,
            reallocation_reference_coords=self.reallocation_ref_coords
        )

        if not target:
            self.get_logger().info(f"[{self.robot_name}] Mission Quota Satisfied ({self.completed_waypoints_count}/{self.spec['waypoint_limit']}). Returning to Spawn.")
            self.current_state = RobotState.IDLE
            self._return_to_spawn_coordinates()
            return

        self.active_target_dict = target
        mode_used = target["mode"]
        target_name = f"{target['item_name']}_{target['side_key']}_{target['section_label']}"
        self.get_logger().info(f"[{self.robot_name}] Target Acquired: {target_name} via {mode_used}")

        self._publish_local_claim("claim")

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
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._next_candidate_fail()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_path_result_received)

    def _on_path_result_received(self, future):
        try:
            result = future.result().result
            # ==================================================================
            # REACHABILITY CHECK
            # Using raw path length validation rather than strict error code mapping.
            # ==================================================================
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
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"[{self.robot_name}] Active Nav goal rejected by Server. Retrying Decision.")
            self.recovery_mgr.stop_monitoring()
            self.current_state = RobotState.DECISION_PHASE
            return
            
        self.active_nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result_received)

    def _on_nav_result_received(self, future):
        self.active_nav_goal_handle = None
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            target_x = self.active_target_dict["x"]
            target_y = self.active_target_dict["y"]
            
            # If we were already navigating to the final target, trust Nav2's success and align yaw
            if self.chosen_candidate == (target_x, target_y):
                self.recovery_mgr.stop_monitoring()
                self._dispatch_alignment_rotation()
            else:
                # We successfully reached the candidate waypoint, now navigate to the final target
                self.chosen_candidate = (target_x, target_y)
                self.recovery_mgr.start_monitoring(self.current_x, self.current_y, is_candidate=False)
                self._dispatch_navigation()
        else:
            self.get_logger().error(f"[{self.robot_name}] Navigation Action Ended (Failed or Canceled).")
            # Switch to IDLE if not currently performing an active reversing or turning maneuver:
            if self.recovery_mgr.state not in (RecoveryState.BACKING_UP, RecoveryState.ROTATING_ESCAPE):
                self.recovery_mgr.state = RecoveryState.IDLE
                self.current_state = RobotState.DECISION_PHASE

    
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
        # Wrap to [-pi, pi]
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
        if self.scan_delay_timer is not None:
            self.scan_delay_timer.cancel()
            self.scan_delay_timer = None
            
        self._publish_arrival_event()
        
        item_name = self.active_target_dict["item_name"]
        side_key = self.active_target_dict["side_key"]
        section_label = self.active_target_dict["section_label"]
        done_string = f"{side_key}_{section_label}"
        
        self.items_registry[item_name].setdefault("done", set()).add(done_string)
        self.completed_waypoints_count += 1
        
        self.get_logger().info(f"[{self.robot_name}] Task Completed: {done_string} ({self.completed_waypoints_count}/{self.spec['waypoint_limit']})")
        self._publish_completed_summary()
        self.current_state = RobotState.DECISION_PHASE

    def _cancel_active_nav2_goal(self):
        if self.active_nav_goal_handle:
            self.get_logger().info(f"[{self.robot_name}] Aborting Nav2 path.")
            self.active_nav_goal_handle.cancel_goal_async()
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
            self._release_local_claim_publicly()
            self.deferred_side_keys.add(side_key)
            self.active_target_dict = None
            
        self.side_lock_active = False  # Reset side lock completely upon task deferral
        self.current_state = RobotState.DECISION_PHASE

    def _publish_local_claim(self, action_string: str):
        if not self.active_target_dict:
            return
            
        now_ns = int(self.get_clock().now().nanoseconds)
        expire_at_ns = now_ns + (6 * 1_000_000_000)
        
        payload = {
            "robot_id": self.robot_name,
            "side_key": f"{self.active_target_dict['side_key']}_{self.active_target_dict['section_label']}",
            "action": action_string,
            "timestamp_ns": now_ns,
            "expire_at_ns": expire_at_ns
        }
        
        msg = String()
        msg.data = json.dumps(payload)
        self.claims_pub.publish(msg)

    def _publish_local_claim_renewals(self):
        if self.current_state in (RobotState.FAILED, RobotState.IDLE):
            return
        if self.active_target_dict and self.active_target_dict.get("side_key") and \
                self.current_state in (RobotState.NAVIGATING_TO_TARGET, RobotState.ALIGNING_YAW, RobotState.SCANNING):
            self._publish_local_claim("renew")

    def _release_local_claim_publicly(self):
        if not self.active_target_dict:
            return
            
        now_ns = int(self.get_clock().now().nanoseconds)
        payload = {
            "robot_id": self.robot_name,
            "side_key": f"{self.active_target_dict['side_key']}_{self.active_target_dict['section_label']}",
            "action": "release",
            "timestamp_ns": now_ns,
            "expire_at_ns": now_ns
        }
        
        msg = String()
        msg.data = json.dumps(payload)
        self.claims_pub.publish(msg)

    def _publish_local_status(self, status_string: str):
        msg = String()
        msg.data = status_string
        self.status_pub.publish(msg)

    def _publish_completed_summary(self):
        completed_keys = []
        for item in self.items_registry.values():
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