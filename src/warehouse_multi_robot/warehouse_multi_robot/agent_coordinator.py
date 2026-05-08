#!/usr/bin/env python3
"""
Parameters:
  robot_name  : robot1 | robot2 | robot3

Subscribed topics:
  /robot1/status, /robot2/status, /robot3/status         -> String
  /robot1/battery_pct, /robot2/battery_pct, ...          -> Float32
  /robot1/remaining_waypoints, /robot2/remaining_..., .. -> String (JSON)

Published topic:
  /robotN/coordinator_status  -> String (log)

Re-allocation Logic:
- When a robot FAILS, the surviving robots share the remaining shelves proportionally to their battery percentage.
- Consensus: The surviving robot with the lowest ID calculates the re-allocation and calls the /robotN/trigger_reallocation service.
The other robot performs the same calculation and takes its share.
Thus, there is no central decision-maker; both robots reach the same result deterministically.

Failure latency measurement:
- The FAILED event timestamp is recorded.
- The difference is calculated when the trigger_reallocation call is made.
- It is published in milliseconds to the /robotN/reallocation_latency topic.
"""

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from std_srvs.srv import Trigger


ALL_ROBOTS = ['robot1', 'robot2', 'robot3']


class AgentCoordinator(Node):

    def __init__(self):
        super().__init__('agent_coordinator')

        #  Parameters
        self.declare_parameter('robot_name', 'robot1')
        self.robot_name = self.get_parameter('robot_name').value

        # others
        self.peers = [r for r in ALL_ROBOTS if r != self.robot_name]

        #  State 
        # For each robot: status, battery, remaining waypoints
        self.peer_status    = {r: 'ACTIVE' for r in ALL_ROBOTS}
        self.peer_battery   = {r: 100.0    for r in ALL_ROBOTS}
        self.peer_remaining = {r: []       for r in ALL_ROBOTS}

        self.handled_failures = set()   
        self.failure_timestamps = {}    # for latency 

        #  Subscribers: listen to all robots
        for robot in ALL_ROBOTS:
            self.create_subscription(
                String, f'/{robot}/status',
                lambda msg, r=robot: self._status_cb(msg, r), 10)

            self.create_subscription(
                Float32, f'/{robot}/battery_pct',
                lambda msg, r=robot: self._battery_cb(msg, r), 10)

            self.create_subscription(
                String, f'/{robot}/remaining_waypoints',
                lambda msg, r=robot: self._remaining_cb(msg, r), 10)

        #  Publisher: coordinator status log 
        self.coord_pub = self.create_publisher(
            String, f'/{self.robot_name}/coordinator_status', 10)

        #  Publisher: re-allocation latency (thesis metrics) 
        self.latency_pub = self.create_publisher(
            Float32, f'/{self.robot_name}/reallocation_latency', 10)

        # Publisher: add_waypoints 
        self.add_wp_pub = self.create_publisher(
            String, f'/{self.robot_name}/add_waypoints', 10)

         
        # WatpointSender service clients
        # Trigger service for remaining robots
        self.wp_clients = {}
        for robot in ALL_ROBOTS:
            self.wp_clients[robot] = self.create_client(
                Trigger, f'/{robot}/trigger_reallocation')

        self.get_logger().info(
            f'[{self.robot_name}] AgentCoordinator initiated | '
            f'peers: {self.peers}'
        )

    # Callbacks
    def _status_cb(self, msg: String, robot: str):
        new_status = msg.data
        old_status = self.peer_status.get(robot, 'ACTIVE')

        self.peer_status[robot] = new_status

        # FAILED
        if new_status == 'FAILED' and old_status == 'ACTIVE':
            if robot not in self.handled_failures:
                self.failure_timestamps[robot] = self.get_clock().now().nanoseconds
                self.get_logger().warn(
                    f'[{self.robot_name}] FAILURE DETECTED: {robot} -> FAILED')
                self._handle_failure(robot)

    def _battery_cb(self, msg: Float32, robot: str):
        self.peer_battery[robot] = msg.data

    def _remaining_cb(self, msg: String, robot: str):
        try:
            data = json.loads(msg.data)
            self.peer_remaining[robot] = data.get('remaining', [])
        except json.JSONDecodeError:
            pass

    # Failure handling
    def _handle_failure(self, failed_robot: str):
        """
        It is triggered when FAILED is detected. Consensus: only the surviving robot with the lowest ID
        triggers re-allocation. However, each robot calculates its own share
        independently - no central decision.
        """
        self.handled_failures.add(failed_robot)

        # survivors
        survivors = [
            r for r in ALL_ROBOTS
            if self.peer_status.get(r, 'ACTIVE') == 'ACTIVE'
        ]

        if self.robot_name not in survivors:
            self.get_logger().info(
                f'[{self.robot_name}] Also FAILED, '
                f'CANNOT re-allocate.')
            return

        if not survivors:
            self.get_logger().error('No surviving agents!')
            return

        # Remaining Waypoints (of failed robot)
        orphaned = self.peer_remaining.get(failed_robot, [])

        if not orphaned:
            self.get_logger().warn(
                f'[{self.robot_name}] {failed_robot}\' remaining waypoints '
                f'is not recived. Will try again in 3 seconds.')
            timer = self.create_timer(
                3.0,
                lambda: None # place holder
            )
            timer.cancel()
            timer = self.create_timer(
                3.0,
                lambda fr = failed_robot, sv = survivors: self._retry_reallocation(fr,sv)
            )
            return

        self._do_reallocation(failed_robot, orphaned, survivors)

    def _retry_reallocation(self, failed_robot: str, survivors: list, timer=None):
            if timer:
                timer.cancel()
            orphaned = self.peer_remaining.get(failed_robot, [])
            if orphaned:
                self._do_reallocation(failed_robot, orphaned, survivors)
            else:
                self.get_logger().error(
                    f'[{self.robot_name}] {failed_robot} remaining_waypoints '
                    f'still did not recieved, re-allocation CANCELLED.')

    def _do_reallocation(self, failed_robot: str,
                         orphaned: list, survivors: list):
        """
        Waypoint distribution proportional to battery percentage. 
        Each surviving robot performs this calculation independently 
        -> same deterministic result -> consensus.        """
        total_battery = sum(self.peer_battery[r] for r in survivors)

        if total_battery == 0:
            # Edge case: all zero battery percentage = divide equally
            shares = {r: 1.0 / len(survivors) for r in survivors}
        else:
            shares = {
                r: self.peer_battery[r] / total_battery
                for r in survivors
            }

        total = len(orphaned)

        #  Calculate the number of waypoints each robot should get
        allocations = {}
        assigned = 0
        sorted_survivors = sorted(survivors)  # deterministic order

        for i, robot in enumerate(sorted_survivors):
            if i == len(sorted_survivors) - 1:
                # Give all to the last remaining (round error correction)
                count = total - assigned
            else:
                count = math.floor(shares[robot] * total)
            allocations[robot] = count
            assigned += count

        # Log
        self.get_logger().info(
            f'[{self.robot_name}] Re-allocation math:\n'
            f'  Failed: {failed_robot} ({total} shelves)\n'
            f'  Battery: { {r: f"{self.peer_battery[r]:.1f}%" for r in survivors} }\n'
            f'  Allocation: {allocations}'
        )

        # Allocate Waypoints
        cursor = 0
        for robot in sorted_survivors:
            count = allocations[robot]
            if count == 0:
                continue

            chunk = orphaned[cursor: cursor + count]
            cursor += count

            if robot == self.robot_name:
                # Send to self waypointsender
                self._inject_waypoints_to_self(chunk)
            
        # Latency Measurement
        if failed_robot in self.failure_timestamps:
            now = self.get_clock().now().nanoseconds
            latency_ms = (now - self.failure_timestamps[failed_robot]) / 1e6
            lat_msg = Float32()
            lat_msg.data = float(latency_ms)
            self.latency_pub.publish(lat_msg)
            self.get_logger().info(
                f'[{self.robot_name}] Re-allocation latency: {latency_ms:.1f} ms')

        # Coordinator status log
        status_msg = String()
        status_msg.data = json.dumps({
            'event':       'reallocation_complete',
            'failed':      failed_robot,
            'survivors':   survivors,
            'allocations': allocations,
            'reporter':    self.robot_name
        })
        self.coord_pub.publish(status_msg)

    def _inject_waypoints_to_self(self, waypoints: list):
        """
        Sends the new waypoints to its own WaypointSender
        """
        if not waypoints:
            return

        # Send Waypoints to add_waypoints topic
        msg = String()
        msg.data = json.dumps({'waypoints': waypoints})
        self.add_wp_pub.publish(msg)

        self.get_logger().info(
            f'[{self.robot_name}] {len(waypoints)} waypoints has been sent to WaypointSender')

        # call trigger_reallocation service after short wait
        client = self.wp_clients.get(self.robot_name)
        if client is not None:
            self.create_timer(2.0, lambda: self._call_trigger(client))

    def _call_trigger(self, client):
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'[{self.robot_name}] trigger_reallocation response: '
                f'{f.result().message if f.result() else "no response"}'
            )
        )

def main(args=None):
    rclpy.init(args=args)
    node = AgentCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()