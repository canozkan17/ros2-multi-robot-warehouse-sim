#!/usr/bin/env python3
"""
recovery.py - Stall Detection and Obstacle Escape Recovery Engine.

This module provides the RecoveryManager class, which monitors the robot's 
navigation progress. If the robot freezes/stalls for more than 5 seconds, 
it handles:
1. Candidate bypass (direct transition to final target).
2. Physical escape maneuvers (0.2m backward drive and angled escape rotation via cmd_vel).
3. Retry limitation (max 5 times before task deferral).
"""

import time
import math
from enum import Enum
from typing import Optional, Tuple, Any

from geometry_msgs.msg import Twist
from rclpy.node import Node


# ==============================================================================
# 1. RECOVERY SYSTEM STATES
# ==============================================================================
class RecoveryState(Enum):
    IDLE = "IDLE"                   # System is inactive (not navigating)
    MONITORING = "MONITORING"       # Actively tracking coordinates for stalls
    BACKING_UP = "BACKING_UP"       # Driving backward slowly via cmd_vel
    ROTATING_ESCAPE = "ROTATING_ESCAPE" # Rotating to clear costmap constraints
    REPLANNING = "REPLANNING"       # Handing control back to Nav2 for retry
    EXHAUSTED = "EXHAUSTED"         # Max retries reached, need to defer task


# ==============================================================================
# 2. CORE RECOVERY MANAGER CLASS
# ==============================================================================
class RecoveryManager:
    def __init__(
        self, 
        node: Node, 
        cmd_vel_publisher, 
        nav2_cancel_callback,
        nav2_retrigger_callback,
        defer_task_callback
    ):
        """
        Initializes the Recovery Manager.

        Args:
            node: Parent ROS2 Node reference (for clock access).
            cmd_vel_publisher: ROS2 publisher handle for geometry_msgs/Twist (cmd_vel).
            nav2_cancel_callback: Function to cancel current active Nav2 goal.
            nav2_retrigger_callback: Function to retrigger Nav2 navigation.
            defer_task_callback: Function to defer current task to end of queue.
        """
        self.node = node
        self.cmd_vel_pub = cmd_vel_publisher
        self.cancel_nav2_goal = nav2_cancel_callback
        self.retrigger_nav2_goal = nav2_retrigger_callback
        self.defer_active_task = defer_task_callback
        
        # State Indicators
        self.state = RecoveryState.IDLE
        self.retry_count = 0
        self.is_heading_to_candidate = False
        
        # Tracking Registers (for Stall Detection)
        self.last_known_x = 0.0
        self.last_known_y = 0.0
        self.last_movement_timestamp_ns = 0
        
        # Maneuver Timing Registers
        self.maneuver_end_timestamp_ns = 0
        
        # Constants
        self.STALL_THRESHOLD_S = 12.0
        self.DIRECT_STALL_THRESHOLD_S = 10.0
        self.PROGRESS_EPSILON_M = 0.03  # Robot must move at least 3cm within threshold
        
        # Physical Escape Configuration (0.2m backward, 0.12m/s speed)
        self.BACKWARD_SPEED = -0.12
        self.BACKWARD_DURATION_S = 0.2 / 0.12  # Exactly 1.667 seconds to cover 0.2m
        
        # Angular Escape Configuration
        self.ESCAPE_ROTATION_SPEED = 0.35
        self.ESCAPE_ROTATION_DURATION_S = 0.9  # Turn slightly to clear costmap edges

    # ==========================================================================
    # STALL TRACKING API
    # ==========================================================================
    def start_monitoring(self, current_x: float, current_y: float, is_candidate: bool):
        """Signals the manager that a navigation sweep has started."""
        now_ns = self.node.get_clock().now().nanoseconds
        self.last_known_x = current_x
        self.last_known_y = current_y
        self.last_movement_timestamp_ns = now_ns
        self.is_heading_to_candidate = is_candidate
        self.state = RecoveryState.MONITORING
        self.node.get_logger().info(
            f"[{self.node.get_name()}] Recovery Monitor ACTIVE. Candidate target: {is_candidate}"
        )

    def stop_monitoring(self):
        """Stops the tracking watchdog (called upon reaching target/section)."""
        self.state = RecoveryState.IDLE
        self.retry_count = 0
        self.is_heading_to_candidate = False
        self._stop_cmd_vel_output()

    def update_pose(self, current_x: float, current_y: float):
        """Updates robot pose and resets the stall clock if progress is made."""
        if self.state != RecoveryState.MONITORING:
            return
            
        distance_moved = math.hypot(current_x - self.last_known_x, current_y - self.last_known_y)
        
        # If progress exceeds epsilon, reset the stall clock
        if distance_moved >= self.PROGRESS_EPSILON_M:
            self.last_known_x = current_x
            self.last_known_y = current_y
            self.last_movement_timestamp_ns = self.node.get_clock().now().nanoseconds

    # ==========================================================================
    # MASTER RECOVERY TICKER (Driven by 10Hz node tick)
    # ==========================================================================
    def tick(self, current_x: float, current_y: float):
        """Master 10Hz clock tick of the recovery manager."""
        if self.state == RecoveryState.IDLE:
            return
            
        now_ns = self.node.get_clock().now().nanoseconds
        
        # 1. MONITORING STATE: Actively watching for freeze/stall
        if self.state == RecoveryState.MONITORING:
            elapsed_seconds = (now_ns - self.last_movement_timestamp_ns) / 1_000_000_000.0

            threshold = self.STALL_THRESHOLD_S if self.is_heading_to_candidate else self.DIRECT_STALL_THRESHOLD_S
            if elapsed_seconds > threshold:
                self._handle_stall_detection()
                
        # 2. BACKING UP STATE: Executing active backward cmd_vel drive
        elif self.state == RecoveryState.BACKING_UP:
            if now_ns >= self.maneuver_end_timestamp_ns:
                self._stop_cmd_vel_output()
                self._start_escape_rotation()
            else:
                self._publish_backward_drive()
                
        # 3. ROTATING ESCAPE STATE: Rotating away from obstacle
        elif self.state == RecoveryState.ROTATING_ESCAPE:
            if now_ns >= self.maneuver_end_timestamp_ns:
                self._stop_cmd_vel_output()
                self._trigger_nav2_replan()
            else:
                self._publish_escape_rotation()

    # ==========================================================================
    # RECOVERY BEHAVIORS & ESCAPES
    # ==========================================================================
    def _handle_stall_detection(self):
        """Fires when the 5-second stall timer expires."""
        self.node.get_logger().error(
            f"[{self.node.get_name()}] STALL DETECTED! Robot has been stagnant for {self.STALL_THRESHOLD_S}s."
        )
        
        # Cancel current active Nav2 trajectory
        self.cancel_nav2_goal()
        
        # Rule A: If stuck on Candidate Approach -> Bypass Candidate and go Direct!
        if self.is_heading_to_candidate:
            self.node.get_logger().warn(
                f"[{self.node.get_name()}] Stuck during Candidate Approach. Bypassing Candidate -> Heading Direct."
            )
            self.is_heading_to_candidate = False
            self.state = RecoveryState.MONITORING
            # Reset stall clock to allow 5s for the direct approach before physical recovery triggers
            self.last_movement_timestamp_ns = self.node.get_clock().now().nanoseconds
            self.retrigger_nav2_goal(go_direct=True)
            return
            
        # Rule B: Stuck during Direct Approach -> Initiate physical cmd_vel escape maneuvers
        self._start_physical_escape()

    def _start_physical_escape(self):
        """Initiates physical escape if direct target is blocked."""
        if self.retry_count >= 5:
            self.node.get_logger().error(
                f"[{self.node.get_name()}] PHYSICAL ESCAPE EXHAUSTED ({self.retry_count}/5 retries). Deferring task!"
            )
            self.state = RecoveryState.EXHAUSTED
            self.defer_active_task()
            return

        self.retry_count += 1
        self.node.get_logger().warn(
            f"[{self.node.get_name()}] Initiating physical escape maneuver (Attempt {self.retry_count}/5)..."
        )
        
        self.state = RecoveryState.BACKING_UP
        now_ns = self.node.get_clock().now().nanoseconds
        self.maneuver_end_timestamp_ns = now_ns + int(self.BACKWARD_DURATION_S * 1_000_000_000)

    def _start_escape_rotation(self):
        """Starts slight rotation to clear inflation costmap boundaries."""
        self.state = RecoveryState.ROTATING_ESCAPE
        now_ns = self.node.get_clock().now().nanoseconds
        self.maneuver_end_timestamp_ns = now_ns + int(self.ESCAPE_ROTATION_DURATION_S * 1_000_000_000)

    def _trigger_nav2_replan(self):
        """Finished physical escape. Instructs Nav2 to replan and try again."""
        self.state = RecoveryState.MONITORING
        self.last_movement_timestamp_ns = self.node.get_clock().now().nanoseconds
        self.node.get_logger().info(
            f"[{self.node.get_name()}] Physical escape finished. Retriggering direct navigation target."
        )
        self.retrigger_nav2_goal(go_direct=True)

    # ==========================================================================
    # VELOCITY CMD_VEL OUTPUTS
    # ==========================================================================
    def _publish_backward_drive(self):
        """Publishes negative linear velocity on cmd_vel."""
        cmd = Twist()
        cmd.linear.x = self.BACKWARD_SPEED
        self.cmd_vel_pub.publish(cmd)

    def _publish_escape_rotation(self):
        """Publishes safe angular rotation to clear obstacle edges."""
        cmd = Twist()
        # Alternate rotation direction based on attempt number to find free space
        rotation_sign = 1.0 if (self.retry_count % 2 == 1) else -1.0
        cmd.angular.z = rotation_sign * self.ESCAPE_ROTATION_SPEED
        self.cmd_vel_pub.publish(cmd)

    def _stop_cmd_vel_output(self):
        """Publishes zero velocity to cleanly stop the robot."""
        self.cmd_vel_pub.publish(Twist())