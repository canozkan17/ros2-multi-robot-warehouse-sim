#!/usr/bin/env python3
import json
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav2_msgs.action import Spin
import math
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class WaypointSender(Node):
    def __init__(self):
        super().__init__('waypoint_sender')
        
        # parameter
        self.declare_parameter('robot_name', 'robot1')
        self.robot_name = self.get_parameter('robot_name').value
        
        # State
        self.remaining = [] 
        self.completed = []
        self.navigating = False
        self.all_done = False
        self.is_failed = False          # stops navigation when FAILED 
        self._current_goal_handle = None  # active goal - for cancelling
        self._delayed_send_timer = None

        self.action_clients = {}
        self.action_results = {}
        
        self.shelf_arrived_pub = self.create_publisher(String, '/shelf_arrived', 10)

        self._alignment_phase = False
        self._current_wp = None

        self._retry_count = 0
        self._max_retries = 5
        self._alignment_retry_count = 0
        self._max_alignment_retries = 3
        self._alignment_timer = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.spin_client = ActionClient(self, Spin, f'/{self.robot_name}/spin')

        # Action Client
        self.action_client = ActionClient(
            self, NavigateToPose, 
            f'/{self.robot_name}/navigate_to_pose'
            )
        
        # Publishers
        self.remaining_pub = self.create_publisher (
            String, 
            f'/{self.robot_name}/remaining_waypoints',10
            )
        
        self.shelf_pub = self.create_publisher(
            String, '/shelf_arrived',10)
        
        # Service: AgentCoordinator triggers navigation
        self.add_wp_srv = self.create_service(
            Trigger,
            f'/{self.robot_name}/trigger_reallocation',
            self._trigger_reallocation_callback
        )

        self._pending_waypoints = [] 

        # Subscriber: get new waypoints from AgentCoordinator 
        self.create_subscription(
            String,
            f'/{self.robot_name}/add_waypoints',
            self._add_waypoints_cb,
            10
        )

        # Status subscriber: FAILED olunca navigasyonu durdur
        self.create_subscription(
            String,
            f'/{self.robot_name}/status',
            self._status_cb,
            10
        )

        # Load waypoints
        self._load_assignments()

        # Periodic remaining broadcast (2 seconds)
        self.create_timer(2.0, self._publish_remaining)

        # Start when nav2 ready

        self.create_timer (1.0, self._try_start)
        self._started = False

        self.get_logger().info(
            f'[{self.robot_name}] WaypointSender initialized. | '
            f'{len(self.remaining)} waypoints loaded.'
        )

    # LOADING
    def _load_assignments(self):
        path = os.path.expanduser(
            '~/thesis_ws/src/warehouse_multi_robot/config/robot_assignments.json')
        with open(path) as f:
            assignments = json.load(f)

        self.remaining = list(assignments.get(self.robot_name, []))

    # STARTING 
    def _try_start(self):
        if self._started:
            return
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(
                f'[{self.robot_name} Nav2 action server is not yet ready...]')
            return
        
        self._started = True
        self.get_logger().info(f'[{self.robot_name}] Nav2 Ready, Navigation Initializing')
        self._send_next()

    # NAVIGATION LOOP
    def _send_next(self):
        if self.is_failed:
            return
        if self.navigating:
            return
        
        # add pending new waypoints
        if self._pending_waypoints:
            added = self._pending_waypoints[:]
            self._pending_waypoints.clear()
            self.remaining.extend(added)
            self.get_logger().info(
                f'[{self.robot_name}] Re-allocation: {len(added)} new waypoints added.'
            )
        
        if not self.remaining:
            if not self.all_done:
                self.get_logger().info(
                    f'[{self.robot_name}] All waypoints COMPLETED |'
                    f'Total: {len(self.completed)}' 
                )
            return
        
        wp = self.remaining[0]
        self.navigating = True

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        # Phase 1: go to the position, orientation relaxed
        goal.pose.pose.position.x = float(wp['x'])
        goal.pose.pose.position.y = float(wp['y'])
        goal.pose.pose.orientation.z = 0.0
        goal.pose.pose.orientation.w = 1.0
        current_yaw = self._get_current_yaw()
        if current_yaw is not None:
            self._set_pose_yaw(goal.pose.pose, current_yaw)
        else:
            goal.pose.pose.orientation.z = 0.0
            goal.pose.pose.orientation.w = 1.0
        self._alignment_phase = False
        self._current_wp = wp

        self.get_logger().info(
            f"[{self.robot_name}] -> waypoint: {wp.get('shelf_id', '?')} x={wp['x']}, y={wp['y']} | "
            f"yaw={goal.pose.pose.orientation.z},{goal.pose.pose.orientation.w} "
            f"(remaining: {len(self.remaining)})"
        )

        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'[{self.robot_name}] Goal Rejected')
            self.navigating = False
            self._send_next()
            return
        self._current_goal_handle = goal_handle  # save for cancellation
        goal_handle.get_result_async().add_done_callback(self._result_cb)
    
    def _result_cb(self, future):
        self.navigating = False
        result = future.result()

        if result.status == 4:  # SUCCEEDED
            self._retry_count = 0
            if not self._alignment_phase:
                # Phase 1 completed - phase 2: send rotation goal
                self._alignment_phase = True
                self._alignment_retry_count = 0
                wp = self._current_wp
                self.get_logger().info(
                    f'[{self.robot_name}] Phase1 OK -> alignment starting: {wp.get("shelf_id")}')
                self._send_alignment_goal(wp)
            else:
                # Phase 2 completed - waypoint really finished 
                self._alignment_phase = False
                wp = self.remaining.pop(0)
                self._current_wp = None
                self.completed.append(wp)
                self._publish_shelf_arrived(wp)
                self.get_logger().info(
                    f'[{self.robot_name}] Waypoint COMPLETED | '
                    f'completed:{len(self.completed)} remaining:{len(self.remaining)}')
                self._schedule_next_send()
        else:
            if self._alignment_phase:
                # Rotation Failed - retry phase 1
                self._alignment_phase = False
                self.get_logger().warn(
                    f'[{self.robot_name}] Alignment FAILED - retrying from phase1: '
                    f'{self._current_wp.get("shelf_id")}')
                self._send_next()
            else:
                # Phase 1 Failed - waypoint skip
                self._retry_count += 1
                if self._retry_count >= self._max_retries:
                    self._retry_count = 0
                    self.remaining.pop(0)
                    self._current_wp = None
                    self.get_logger().warn(
                        f'[{self.robot_name}] Phase 1 FAILED {self._max_retries}x, skipping: '
                        f'{self._current_wp.get("shelf_id") if self._current_wp else "?"}')
                else:
                    self.get_logger().warn(
                        f'[{self.robot_name}] Phase 1 FAILED, retry {self._retry_count}/{self._max_retries}')
                self._send_next()

    def _send_alignment_goal(self, wp):
        self.navigating = True
        # The difference between the current orientation and the target orientation
        # qz, qw -> yaw conversion (2D, z-axis rotation)
        target_yaw = 2.0 * math.atan2(float(wp.get('qz', 0.0)), float(wp.get('qw', 1.0)))

        current_yaw = self._get_current_yaw()
        if current_yaw is None:
            self.get_logger().warn(
                f'[{self.robot_name}] Alignment: current yaw unavailable, retrying...')
            self.navigating = False
            self._schedule_alignment_retry()
            return

        target_delta = self._normalize_angle(target_yaw - current_yaw)
        
        goal = Spin.Goal()
        goal.target_yaw = target_delta
        
        if not self.spin_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn(
                f'[{self.robot_name}] Spin server unavailable, retrying alignment')
            self.navigating = False
            self._schedule_alignment_retry()
            return

        future = self.spin_client.send_goal_async(goal)
        future.add_done_callback(self._spin_response_cb)

    def _spin_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(
                f'[{self.robot_name}] Spin rejected, retrying alignment')
            self.navigating = False
            self._schedule_alignment_retry()
            return
        goal_handle.get_result_async().add_done_callback(self._spin_result_cb)

    def _spin_result_cb(self, future):
        self.navigating = False
        result = future.result()
        if result.status != 4:
            self._alignment_retry_count += 1
            self.get_logger().warn(
                f'[{self.robot_name}] Alignment failed ({self._alignment_retry_count}/'
                f'{self._max_alignment_retries}), retrying')
            if self._alignment_retry_count >= self._max_alignment_retries:
                self._alignment_retry_count = 0
                self._alignment_phase = False
                self._send_next()
            else:
                self._schedule_alignment_retry()
            return

        self._alignment_phase = False
        wp = self.remaining.pop(0)
        self._current_wp = None
        self.completed.append(wp)
        self._publish_shelf_arrived(wp)
        self.get_logger().info(
            f'[{self.robot_name}] Waypoint COMPLETED (with alignment) | '
            f'completed:{len(self.completed)} remaining:{len(self.remaining)}')
        self._schedule_next_send()

    def _schedule_alignment_retry(self, delay=2.0):
        if self._alignment_timer is not None:
            self._alignment_timer.cancel()
            self._alignment_timer = None
        self._alignment_timer = self.create_timer(
            delay,
            self._alignment_retry_callback
        )

    def _alignment_retry_callback(self):
        if self._alignment_timer is not None:
            self._alignment_timer.cancel()
            self._alignment_timer = None
        if self._current_wp is None:
            return
        self._send_alignment_goal(self._current_wp)

    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _get_current_yaw(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', f'{self.robot_name}/base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().warn(
                f'[{self.robot_name}] TF lookup failed: {ex}')
            return None

        q = tf.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _set_pose_yaw(self, pose, yaw: float) -> None:
        pose.orientation.z = math.sin(yaw * 0.5)
        pose.orientation.w = math.cos(yaw * 0.5)

    def _schedule_next_send(self, delay=2.0):
        if self._delayed_send_timer is not None:
            self._delayed_send_timer.cancel()
            self._delayed_send_timer = None
        self._delayed_send_timer = self.create_timer(
            delay,
            self._delayed_send_callback
        )

    def _delayed_send_callback(self):
        if self._delayed_send_timer is not None:
            self._delayed_send_timer.cancel()
            self._delayed_send_timer = None
        self._send_next()

    #/shelf_arrived broadcast
    def _publish_shelf_arrived(self, wp):
        msg = String()
        msg.data = json.dumps({
            'robot_id':  self.robot_name,
            'shelf_id':  wp.get('shelf_id', f'shelf_{len(self.completed)}'),
            'x':         wp['x'],
            'y':         wp['y'],
            'timestamp': self.get_clock().now().nanoseconds
        })
        self.shelf_pub.publish(msg)

    # /robotN/remaining_waypoints periodic broadcast
    def _publish_remaining(self):
        msg = String()
        msg.data = json.dumps({
            'robot_id':  self.robot_name,
            'remaining': self.remaining,
            'count':     len(self.remaining)
        })
        self.remaining_pub.publish(msg)

    # Reallocation service: AgentCoordinator writes into this variable and calls this service to trigger navigation
    def _trigger_reallocation_callback(self, request, response):
        if self._pending_waypoints: 
            self.all_done = False
            self._send_next()
            response.success = True
            response.message = (
                f'{len(self._pending_waypoints)} waypoints received into the queue)'
            )
        else: 
            response.success = False
            response.message = 'No pending waypoints to process'
        return response
    
    # Add to queue
    def _add_waypoints_cb(self, msg:String):
        try: 
            data = json.loads(msg.data)
            waypoints = data.get('waypoints', [])
            if waypoints:
                self._pending_waypoints.extend(waypoints)
                self.get_logger().info(
                    f'[{self.robot_name}] add_waypoits:'
                    f'{len(waypoints)} added to the queue'
                )
        except json.JSONDecodeError:
            self.get_logger().error('add_waypoints: Invalid JSON format')

    # Status callback: stop when FAILED 
    def _status_cb(self, msg: String):
        if msg.data == 'FAILED' and not self.is_failed:
            self.is_failed = True
            self.navigating = False
            self.get_logger().warn(
                f'[{self.robot_name}] FAILED status received - navigation stopped. '
                f'Remaining Waypoints : {len(self.remaining)}'
            )
            # Cancel active points if any
            if self._current_goal_handle is not None:
                self._current_goal_handle.cancel_goal_async()
                self._current_goal_handle = None

    def add_pending_waypoints(self, waypoints: list):
        self._pending_waypoints.extend(waypoints)
        self.all_done = False

def main(args=None):
    rclpy.init(args=args)
    node = WaypointSender()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()