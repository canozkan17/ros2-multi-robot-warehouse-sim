#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json
import os

class WaypointSender(Node):
    def __init__(self):
        super().__init__('waypoint_sender')
        
        self.robots = ['robot1', 'robot2', 'robot3']
        self.action_clients = {}
        self.action_results = {}
        
        self.shelf_arrived_pub = self.create_publisher(String, '/shelf_arrived', 10)

        for robot in self.robots:
            self.action_clients[robot] = ActionClient(
                self, NavigateToPose, f'/{robot}/navigate_to_pose'
            )
        
        # robot_assignments.json'dan waypoint'leri oku
        assignments_path = os.path.expanduser(
            '~/thesis_ws/src/warehouse_multi_robot/config/robot_assignments.json'
        )
        with open(assignments_path) as f:
            self.assignments = json.load(f)
        
        self.timer = self.create_timer(1.0, self.send_goals)
        self.sent = False

    def send_goals(self):
        if self.sent:
            return
            
        for robot in self.robots:
            if not self.action_clients[robot].wait_for_server(timeout_sec=2.0):
                self.get_logger().warn(f'{robot} action server is NOT ready')
                return
        
        self.sent = True
        for robot in self.robots:
            waypoints = self.assignments.get(robot, [])
            if waypoints:
                self.send_next_waypoint(robot, waypoints, 0)

    def send_next_waypoint(self, robot, waypoints, index):
        if index >= len(waypoints):
            self.get_logger().info(f'{robot} completed all waypoints')
            return
        
        wp = waypoints[index]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(wp['x'])
        goal.pose.pose.position.y = float(wp['y'])
        goal.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f'{robot} -> waypoint {index+1}/{len(waypoints)}: x={wp["x"]}, y={wp["y"]}')
        
        future = self.action_clients[robot].send_goal_async(goal)
        future.add_done_callback(
            lambda f, r=robot, w=waypoints, i=index: self.goal_response_callback(f, r, w, i)
        )

    def goal_response_callback(self, future, robot, waypoints, index):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'{robot} goal denied')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, r=robot, w=waypoints, i=index: self.result_callback(f, r, w, i)
        )

    def result_callback(self, future, robot, waypoints, index):
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            wp = waypoints[index]
            msg = String()
            msg.data = f'{{"robot_id": "{robot}", "shelf_id": "shelf_{index}", "timestamp": "{self.get_clock().now().nanoseconds}"}}'
            self.shelf_arrived_pub.publish(msg)
            self.get_logger().info(f'{robot} waypoint {index+1} COMPLETED')
            self.send_next_waypoint(robot, waypoints, index + 1)
        else:
            self.get_logger().warn(f'{robot} waypoint {index+1} FAILED, trying the next one...')
            self.send_next_waypoint(robot, waypoints, index + 1)

def main():
    rclpy.init()
    node = WaypointSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()