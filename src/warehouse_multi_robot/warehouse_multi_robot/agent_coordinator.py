#!/usr/bin/env python3
"""
agent_coordinator.py - Decentralized Multi-Agent Fleet Coordinator Node.
"""

import json
import math
from typing import Dict, List, Set, Any, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String, Bool

from warehouse_multi_robot.config import (
    SDF_WORLD_PATH, ROBOTS_SPECIFICATION, MOBILE_CLUSTER_NAME, ITEM_SIDE_OWNERS
)
from warehouse_multi_robot.sdf_parser import parse_sdf


class AgentCoordinatorNode(Node):
    def __init__(self):
        super().__init__("agent_coordinator")
        
        self.declare_parameter("robot_name", "robot1")
        self.robot_name = self.get_parameter("robot_name").value
        self.spec = ROBOTS_SPECIFICATION[self.robot_name]
        
        self.items_registry = parse_sdf(SDF_WORLD_PATH)
        
        self.global_completed_waypoints: Set[str] = set()
        self.active_claims_registry: Dict[str, Dict[str, Any]] = {}
        self.robots_heartbeat_registry: Dict[str, int] = {}
        self.robots_last_known_wp: Dict[str, str] = {}
        
        self.failed_robots: Set[str] = set()
        
        qos_claims = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        qos_transient = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        
        self.claims_pub = self.create_publisher(String, "/side_claims", qos_claims)
        self.add_waypoints_pub = self.create_publisher(String, "add_waypoints", 10)
        self.waypoint_sender_status_pub = self.create_publisher(String, "status", 10)
        
        self.create_subscription(String, "/side_claims", self._side_claims_callback, qos_claims)
        self.create_subscription(String, "/shelf_arrived", self._shelf_arrived_callback, qos_claims)
        self.create_subscription(String, "status", self._local_status_callback, 10)
        
        self.heartbeat_monitor_timer = self.create_timer(1.0, self._monitor_fleet_heartbeats)
        self.local_claim_timer = self.create_timer(1.5, self._publish_local_claim_renewals)
        
        self.local_active_claim_key: Optional[str] = None
        self.local_is_failed = False

        self.get_logger().info(f"[{self.robot_name}] AgentCoordinator active.")

    def _monitor_fleet_heartbeats(self):
        if self.local_is_failed:
            return
            
        now_ns = int(self.get_clock().now().nanoseconds)
        lease_timeout_ns = 5 * 1_000_000_000
        
        for other_robot, last_timestamp in list(self.robots_heartbeat_registry.items()):
            if other_robot == self.robot_name or other_robot in self.failed_robots:
                continue
                
            elapsed_ns = now_ns - last_timestamp
            if elapsed_ns > lease_timeout_ns:
                self.get_logger().error(f"[{self.robot_name}] HEARTBEAT LOST for {other_robot}. Reallocating tasks!")
                self.failed_robots.add(other_robot)
                self._trigger_greedy_reallocation_for_failed_robot(other_robot)

    def _trigger_greedy_reallocation_for_failed_robot(self, failed_robot_name: str):
        expired_claims = []
        for side_key, claim in self.active_claims_registry.items():
            if claim.get("robot_id") == failed_robot_name:
                expired_claims.append(side_key)
                
        for side_key in expired_claims:
            self.active_claims_registry.pop(side_key, None)

        failed_robot_spec = ROBOTS_SPECIFICATION.get(failed_robot_name)
        if not failed_robot_spec:
            return
            
        last_wp_id = self.robots_last_known_wp.get(failed_robot_name)
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
                if static_owner and static_owner != failed_robot_name and static_owner not in self.failed_robots:
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

        payload = {
            "trigger_robot": self.robot_name,
            "failed_robot": failed_robot_name,
            "override_regions": True,
            "reference_coordinates": last_wp_coordinates,
            "waypoints": uncompleted_waypoints_to_reallocate
        }
        
        msg = String()
        msg.data = json.dumps(payload)
        self.add_waypoints_pub.publish(msg)

    def _publish_local_claim_renewals(self):
        if self.local_is_failed or not self.local_active_claim_key:
            return
            
        now_ns = int(self.get_clock().now().nanoseconds)
        expire_at_ns = now_ns + (6 * 1_000_000_000)
        
        payload = {
            "robot_id": self.robot_name,
            "side_key": self.local_active_claim_key,
            "action": "renew",
            "timestamp_ns": now_ns,
            "expire_at_ns": expire_at_ns
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.claims_pub.publish(msg)

    def _side_claims_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        robot_id = data.get("robot_id")
        side_key = data.get("side_key")
        action = data.get("action")
        timestamp_ns = int(data.get("timestamp_ns", 0))
        expire_at_ns = int(data.get("expire_at_ns", 0))

        if not robot_id or not side_key or not action:
            return

        if robot_id != self.robot_name:
            self.robots_heartbeat_registry[robot_id] = timestamp_ns
            if robot_id in self.failed_robots:
                self.failed_robots.discard(robot_id)

        if action == "release":
            self.active_claims_registry.pop(side_key, None)
        elif action in ("claim", "renew"):
            self.active_claims_registry[side_key] = {
                "robot_id": robot_id,
                "expire_at_ns": expire_at_ns
            }

    def _shelf_arrived_callback(self, msg: String):
        """Updates fleet-wide task completions with zero-bug parser."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        robot_id = data.get("robot_id")
        shelf_id = data.get("shelf_id")  # Format: shelf_name_side_section
        
        if robot_id and shelf_id:
            self.global_completed_waypoints.add(shelf_id)
            self.robots_last_known_wp[robot_id] = shelf_id
            
            # ==================================================================
            # FIX 2: GENERIC COMPLETED PARSER (ZERO BUG)
            # ==================================================================
            parts = shelf_id.split("_")
            if len(parts) >= 3:
                item_name = "_".join(parts[:-2])
                
                if item_name in self.items_registry:
                    self.items_registry[item_name].setdefault("done", set()).add(shelf_id)
            
            side_id = data.get("side_id")
            if side_id and robot_id == self.robot_name:
                if self.local_active_claim_key == side_id:
                    self._release_local_claim()

    def _local_status_callback(self, msg: String):
        if msg.data == "FAILED" and not self.local_is_failed:
            self.local_is_failed = True
            self._release_local_claim()

    def _release_local_claim(self):
        if not self.local_active_claim_key:
            return
            
        now_ns = int(self.get_clock().now().nanoseconds)
        payload = {
            "robot_id": self.robot_name,
            "side_key": self.local_active_claim_key,
            "action": "release",
            "timestamp_ns": now_ns,
            "expire_at_ns": now_ns
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.claims_pub.publish(msg)
        
        self.active_claims_registry.pop(self.local_active_claim_key, None)
        self.local_active_claim_key = None

    def _get_waypoint_coordinates_by_id(self, wp_id: str) -> Optional[Tuple[float, float]]:
        parts = wp_id.split("_")
        if len(parts) < 3:
            return None
            
        item_name = "_".join(parts[:-2])
        section_label = parts[-1]
        side_name = parts[-2]
        side_key = f"{item_name}_{side_name}"

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


def main(args=None):
    rclpy.init(args=args)
    node = AgentCoordinatorNode()
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