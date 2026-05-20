import json
import os
import time
from collections import deque

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from std_msgs.msg import String

ROBOT_TOPICS = ['robot1', 'robot2', 'robot3']
ASSIGNMENTS_PATH = os.path.expanduser('~/thesis_ws/src/warehouse_multi_robot/config/robot_assignments.json')


def format_items(items, limit=8):
    if not items:
        return 'none'
    if len(items) <= limit:
        return ', '.join(items)
    return ', '.join(items[:limit]) + f', ... +{len(items) - limit} more'


class WaypointReporter(Node):
    def __init__(self):
        super().__init__('waypoint_reporter')

        self.initial_assignments = self._load_assignments()
        self.remaining_waypoints = {robot: [] for robot in self.initial_assignments}
        self.odom = {robot: None for robot in self.initial_assignments}
        self.last_update = {robot: 0.0 for robot in self.initial_assignments}
        self.recent_logs = deque(maxlen=50)

        for robot in self.initial_assignments:
            self.create_subscription(
                String,
                f'/{robot}/remaining_waypoints',
                lambda msg, r=robot: self._remaining_cb(msg, r),
                10
            )
            self.create_subscription(
                Odometry,
                f'/{robot}/odom',
                lambda msg, r=robot: self._odom_cb(msg, r),
                10
            )

        self.create_subscription(Log, '/rosout', self._rosout_cb, 50)
        self.create_timer(5.0, self._print_report)

        print('🚀 Waypoint Reporter Başlatıldı... (Çıkmak için Ctrl+C)')
        print('-' * 80)
        print(f'  assignments: {ASSIGNMENTS_PATH}')
        print('  subscribed to: ' + ', '.join(f"/{r}/remaining_waypoints" for r in self.initial_assignments))
        print('  subscribed to: ' + ', '.join(f'/{r}/odom' for r in self.initial_assignments))
        print('  subscribed to: /rosout (waypoint_sender logs)')
        print('-' * 80)

    def _load_assignments(self):
        if not os.path.exists(ASSIGNMENTS_PATH):
            self.get_logger().error(f'Assignments file not found: {ASSIGNMENTS_PATH}')
            return {r: [] for r in ROBOT_TOPICS}

        with open(ASSIGNMENTS_PATH, 'r') as f:
            data = json.load(f)

        return {
            robot: data.get(robot, [])
            for robot in ROBOT_TOPICS
        }

    def _remaining_cb(self, msg: String, robot: str):
        try:
            data = json.loads(msg.data)
            remaining = data.get('remaining', [])
            self.remaining_waypoints[robot] = remaining
            self.last_update[robot] = time.time()
        except json.JSONDecodeError:
            self.get_logger().warn(f'[{robot}] remaining_waypoints JSON parse error')

    def _odom_cb(self, msg: Odometry, robot: str):
        p = msg.pose.pose.position
        self.odom[robot] = (round(p.x, 3), round(p.y, 3))
        self.last_update[robot] = time.time()

    def _rosout_cb(self, msg: Log):
        if 'waypoint_sender' not in msg.name and 'waypoint_sender' not in msg.msg:
            return
        self.recent_logs.append((time.time(), msg.name, msg.msg))

    def _latest_waypoint_log(self, robot: str):
        for _, name, message in reversed(self.recent_logs):
            if robot in name or robot in message:
                return message
        return None

    def _report_robot(self, robot: str):
        initial = self.initial_assignments.get(robot, [])
        initial_ids = [wp.get('shelf_id', repr(wp)) for wp in initial]
        remaining = self.remaining_waypoints.get(robot, [])
        remaining_ids = [wp.get('shelf_id', repr(wp)) for wp in remaining]

        completed_ids = [wp_id for wp_id in initial_ids if wp_id not in remaining_ids]
        orphaned = [wp_id for wp_id in remaining_ids if wp_id not in initial_ids]

        age = time.time() - self.last_update.get(robot, 0.0)
        stale = ' (stale)' if self.last_update.get(robot, 0.0) == 0 else ''

        print('\n')
        print(f'[{robot}] completed={len(completed_ids)} remaining={len(remaining_ids)} age={age:.1f}s{stale}')

        if self.odom[robot] is not None:
            x, y = self.odom[robot]
            print(f'    odom: x={x:.3f} y={y:.3f}')
        else:
            print('    odom: unknown')

        print(f'    completed: {format_items(completed_ids)}')
        print(f'    remaining: {format_items(remaining_ids)}')
        if orphaned:
            print(f'    unknown remaining: {format_items(orphaned)}')

        latest_log = self._latest_waypoint_log(robot)
        if latest_log:
            print(f'    latest waypoint_sender log: {latest_log}')

    def _print_report(self):
        print('\n' + '=' * 80)
        print('WAYPOINT EXECUTION SUMMARY')
        print('=' * 80)
        for robot in self.initial_assignments:
            self._report_robot(robot)
        print('=' * 80)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointReporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n\n🛑 Waypoint Reporter durduruldu. Terminale dönülüyor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
