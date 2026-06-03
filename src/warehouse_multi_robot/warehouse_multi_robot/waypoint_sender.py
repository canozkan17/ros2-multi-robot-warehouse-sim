#!/usr/bin/env python3
"""
waypoint_sender.py — Hybrid Navigation Node
============================================
Nav2: 2-step approach per side (corridor entry → section start)
Final pose: second Nav2 yaw goal to face the shelf after arrival
"""

import json
import math
import os
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from std_msgs.msg import String, Bool as BoolMsg

SDF_PATH = os.path.expanduser(
    "~/thesis_ws/src/warehouse_multi_robot/worlds/tugbot_warehouse_clean.sdf")

APPROACH_DIST     = 1.2
FINAL_GOAL_TOL    = 0.35
SECTION_CENTER_TOL = 0.35
SMALL_SHELF_WIDTH = 3.6
SMALL_SECTIONS    = 4
SMALL_STEP        = SMALL_SHELF_WIDTH / SMALL_SECTIONS   # 0.9 m

BIG_SHELF_LENGTH  = 18.0
BIG_SECTIONS      = 4
BIG_STEP          = BIG_SHELF_LENGTH / BIG_SECTIONS      # 4.5 m

SWEEP_ANG_VEL     = 0.4
SWEEP_LIN_VEL     = 0.15
SWEEP_WAIT_SEC    = 2.0

STALL_TIMEOUT_SEC = 5.0
CORNER_RECOVERY_BACK_SEC = 0.8
CORNER_RECOVERY_BACK_SPEED = -0.12
CORNER_RECOVERY_TURN_SEC = 0.9
CORNER_RECOVERY_TURN_SPEED = 0.35
CORNER_RECOVERY_ESCAPE_DIST = 0.45

Q_X = (0.0, 1.0)        # yaw=0  robot faces +X  (yminus/yplus sides)
Q_Y = (0.7071, 0.7071)  # yaw=90 robot faces +Y  (xminus/xplus sides)

MODEL_ORIGIN_OFFSETS = {
    "shelf": (-0.5, 0.0),
    "shelf_big": (-0.5, 0.0),
    "pallet": (0.0, 0.0),
    "pallet_box_mobile": (0.0, 0.0),
}

ITEM_SIDE_OWNER = {
    "shelf_2_yplus":      "robot1", "shelf_2_yminus":     "robot3",
    "shelf_5_yminus":     "robot1", "shelf_5_yplus":      "robot2",
    "shelf_big_3_xplus":  "robot2", "shelf_big_3_xminus": "robot3",
}

ROBOT_CONFIG = {
    "robot1": {"spawn": (0.0,  1.0), "first_item": "shelf_2",
               "first_side": "yplus",  "owned_regions": [2,1,4],
               "priority_dirs": ["north", "east"], "wp_limit": 48},
    "robot2": {"spawn": (0.0,  0.0), "first_item": "shelf_10",
               "first_side": "yplus", "owned_regions": [4,3],
               "priority_dirs": ["east", "south"], "wp_limit": 47},
    "robot3": {"spawn": (0.0, -1.0), "first_item": "shelf_2",
               "first_side": "yminus", "owned_regions": [2,3],
               "priority_dirs": ["south", "west"], "wp_limit": 47},
}

SINGLE_SIDE   = {"shelf_big_1": ["xminus"]}
SPECIAL_SIDES = {"pallet_box_0": ["xminus", "yplus"]}
MOBILE_GROUP  = ["pallet_box_mobile", "pallet_box_mobile_0", "pallet_box_mobile_1"]
MOBILE_SET    = set(MOBILE_GROUP)
MOBILE_CLUSTER = "mobile_cluster"

def _type(name):
    n = name.lower()
    if "shelf_big" in n: return "shelf_big"
    if "pallet"    in n: return "pallet"
    if "shelf"     in n: return "shelf"
    return None

def _region(x, y):
    if x >= 0 and y >= 0: return 1
    if x <  0 and y >= 0: return 2
    if x <  0 and y <  0: return 3
    return 4

def _small_shelf_sides(name, cx, cy):
    half = SMALL_SHELF_WIDTH / 2.0
    labels = ["A", "B", "C", "D"]
    sides = {}
    for side in ["yminus", "yplus"]:
        approach_y = round(cy + APPROACH_DIST, 3) if side == "yplus" else round(cy - APPROACH_DIST, 3)
        # create points in canonical left->right order (A..D = low x -> high x)
        points = [
            (round(cx - half + SMALL_STEP / 2.0 + i * SMALL_STEP, 3), approach_y, label)
            for i, label in enumerate(labels)
        ]
        sides[side] = {
            "side": side, "key": f"{name}_{side}",
            "approach_x": points[0][0], "approach_y": approach_y,
            "axis": "section_line", "step": 0.0, "sections": SMALL_SECTIONS,
            "qz": Q_X[0], "qw": Q_X[1],
            "section_points_template": points,
            "section_labels_template": labels,
        }
    return sides

def _big_shelf_sides(name, cx, cy, yaw):
    labels = ["A", "B", "C", "D"]
    sides = {}
    for side in SINGLE_SIDE.get(name, ["xminus", "xplus"]):
        ax = (cx - APPROACH_DIST) if side == "xminus" else (cx + APPROACH_DIST)
        points = [
            (round(ax, 3), round(cy + BIG_SHELF_LENGTH / 2.0 - BIG_STEP / 2.0 - i * BIG_STEP, 3), label)
            for i, label in enumerate(labels)
        ]
        sides[side] = {
            "side": side, "key": f"{name}_{side}",
            "approach_x": round(ax, 3), "approach_y": points[0][1],
            "axis": "section_line", "step": 0.0, "sections": BIG_SECTIONS,
            "qz": Q_Y[0], "qw": Q_Y[1],
            "section_points_template": points,
            "section_labels_template": labels,
        }
    return sides

def _pallet_sides(name, cx, cy):
    sides = {}
    for side in SPECIAL_SIDES.get(name, ["xminus", "xplus"]):
        if side in ("xminus", "xplus"):
            ax = (cx - APPROACH_DIST) if side == "xminus" else (cx + APPROACH_DIST)
            ay, qz, qw = cy, Q_Y[0], Q_Y[1]
        else:
            ax = cx
            ay = (cy - APPROACH_DIST) if side == "yminus" else (cy + APPROACH_DIST)
            qz, qw = Q_X[0], Q_X[1]
        sides[side] = {
            "side": side, "key": f"{name}_{side}",
            "approach_x": round(ax, 3), "approach_y": round(ay, 3),
            "axis": None, "step": 0.0, "sections": 1,
            "qz": qz, "qw": qw,
            "section_labels": ["A"] if side == "xminus" else ["B"] if side == "xplus" else ["C"] if side == "yplus" else ["D"],
        }
    return sides

def _mobile_cluster_sides(mobiles):
    sides = {}
    for side_name in ["xminus", "xplus"]:
        # order mobiles left->right by x coordinate so labels A..C map to increasing x
        ordered = sorted(mobiles, key=lambda m: m["x"], reverse=False)
        points = []
        labels = ["A", "B", "C"]
        for m, label in zip(ordered, labels):
            x = m["x"] - APPROACH_DIST if side_name == "xminus" else m["x"] + APPROACH_DIST
            points.append((round(x, 3), round(m["y"], 3), label))
        if not points:
            continue
        qz, qw = Q_Y[0], Q_Y[1]
        sides[side_name] = {
            "side": side_name,
            "key": f"{MOBILE_CLUSTER}_{side_name}",
            "approach_x": points[0][0],
            "approach_y": points[0][1],
            "axis": "section_line",
            "step": 0.0,
            "sections": len(points),
            "qz": qz,
            "qw": qw,
            "section_points_template": points,
            "section_labels_template": labels,
        }
    return sides

def _prepare_section_line_side_for_pose(side, cx, cy):
    pts = side.get("section_points_template", [])
    if not pts:
        return side
    start_dist = _dist(cx, cy, pts[0][0], pts[0][1])
    end_dist = _dist(cx, cy, pts[-1][0], pts[-1][1])
    ordered = list(reversed(pts)) if end_dist < start_dist else list(pts)
    label_template = side.get("section_labels_template", [])
    ordered_labels = list(reversed(label_template)) if end_dist < start_dist else list(label_template)
    prepared = dict(side)
    prepared["section_points"] = ordered
    prepared["section_labels"] = ordered_labels if ordered_labels else label_template
    prepared["sections"] = len(ordered)
    prepared["approach_x"] = ordered[0][0]
    prepared["approach_y"] = ordered[0][1]
    prepared["step"] = 0.0
    return prepared

def _section_label(side, idx):
    labels = side.get("section_labels", [])
    if 0 <= idx < len(labels):
        return labels[idx]
    return "?"

def _apply_xy_offset(x, y, yaw, dx, dy):
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        x + dx * cos_yaw - dy * sin_yaw,
        y + dx * sin_yaw + dy * cos_yaw,
    )

def _model_center(item_type, x, y, yaw):
    dx, dy = MODEL_ORIGIN_OFFSETS.get(item_type, (0.0, 0.0))
    cx, cy = _apply_xy_offset(x, y, yaw, dx, dy)
    return round(cx, 3), round(cy, 3)

def parse_sdf(path):
    world = ET.parse(path).getroot().find("world")
    items = {}
    mobile_refs = []
    for inc in world.findall("include"):
        ne, pe = inc.find("name"), inc.find("pose")
        if ne is None or pe is None: continue
        name = ne.text.strip()
        t = _type(name)
        if not t: continue
        v   = pe.text.strip().split()
        x, y, yaw = float(v[0]), float(v[1]), float(v[5])
        cx, cy = _model_center(t, x, y, yaw)
        if   t == "shelf":      sides = _small_shelf_sides(name, cx, cy)
        elif t == "shelf_big":  sides = _big_shelf_sides(name, cx, cy, yaw)
        else:                   sides = _pallet_sides(name, cx, cy)
        items[name] = {"name": name, "type": t, "x": cx, "y": cy,
                       "model_x": x, "model_y": y, "model_yaw": yaw,
                       "region": _region(cx, cy), "sides": sides, "done": set()}
        if name in MOBILE_SET:
            mobile_refs.append({"name": name, "x": cx, "y": cy})

    if mobile_refs:
        cx = sum(m["x"] for m in mobile_refs) / len(mobile_refs)
        cy = sum(m["y"] for m in mobile_refs) / len(mobile_refs)
        cluster_sides = _mobile_cluster_sides(mobile_refs)
        items[MOBILE_CLUSTER] = {
            "name": MOBILE_CLUSTER,
            "type": "mobile_cluster",
            "x": cx,
            "y": cy,
            "region": _region(cx, cy),
            "sides": cluster_sides,
            "done": set(),
        }
        for name in MOBILE_GROUP:
            items.pop(name, None)
    return items

def _dist(x1,y1,x2,y2): return math.hypot(x1-x2, y1-y2)
def _yaw(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def _adiff(t, c):
    d = t - c
    while d >  math.pi: d -= 2*math.pi
    while d < -math.pi: d += 2*math.pi
    return d

def _travel_yaw(sx, sy, tx, ty):
    return math.atan2(ty - sy, tx - sx)

def _yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)

def _dir_match(direction, rx, ry, ix, iy, eps=1e-6):
    if direction == "north":
        return iy > ry + eps
    if direction == "east":
        return ix > rx + eps
    if direction == "south":
        return iy < ry - eps
    if direction == "west":
        return ix < rx - eps
    return True

def _direction_from_delta(dx, dy):
    if abs(dx) >= abs(dy):
        return "east" if dx > 0 else "west"
    return "north" if dy > 0 else "south"

def _side_direction(side, rx, ry):
    ax = side.get("approach_x")
    ay = side.get("approach_y")
    if ax is None or ay is None:
        return None
    return _direction_from_delta(ax - rx, ay - ry)

def _priority_rank(direction, priority_dirs):
    try:
        return priority_dirs.index(direction)
    except ValueError:
        return None

from .item_selection import (
    select_next_side_for_item,
    select_next_item,
    _remaining_sides,
    _is_claimed_by_other,
    _side_direction,
    _priority_rank
)

IDLE, NAVIGATING, TURNING_TO, FACING, TURNING_BACK, ADVANCING, RECOVER_BACKING, RECOVER_TURNING = (
    "IDLE","NAVIGATING","TURNING_TO","FACING","TURNING_BACK","ADVANCING","RECOVER_BACKING","RECOVER_TURNING")

AMCL_QOS = QoSProfile(depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE)

MISSION_ARMED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

class WaypointSender(Node):
    def __init__(self):
        super().__init__("waypoint_sender")
        self.declare_parameter("robot_name", "robot1")
        self.robot_name = self.get_parameter("robot_name").value
        cfg = ROBOT_CONFIG[self.robot_name]

        self.spawn_x, self.spawn_y = cfg["spawn"]
        self.first_item  = cfg["first_item"]
        self.first_side  = cfg["first_side"]
        self.owned       = cfg["owned_regions"]
        self.priority_dirs = cfg["priority_dirs"]
        self.wp_limit    = cfg["wp_limit"]

        # Localization freshness thresholds (seconds, meters)
        self.declare_parameter('max_amcl_age_sec', 5.0)
        self.declare_parameter('max_amcl_pos_sigma', 0.5)
        self.declare_parameter('require_amcl_age_gate', False)
        self.declare_parameter('goal_reject_retry_sec', 0.8)
        self.declare_parameter('claim_lease_sec', 6.0)
        self.declare_parameter('claim_renew_sec', 1.5)
        self.declare_parameter('corner_recovery_max_tries', 1)
        self.declare_parameter('stall_progress_eps', 0.03)
        self.declare_parameter('stall_watch_period', 0.5)
        self.declare_parameter('auto_start', False)
        self.max_amcl_age_sec = float(self.get_parameter('max_amcl_age_sec').value)
        self.max_amcl_pos_sigma = float(self.get_parameter('max_amcl_pos_sigma').value)
        self.require_amcl_age_gate = bool(self.get_parameter('require_amcl_age_gate').value)
        self.goal_reject_retry_sec = float(self.get_parameter('goal_reject_retry_sec').value)
        self.claim_lease_sec = float(self.get_parameter('claim_lease_sec').value)
        self.claim_renew_sec = float(self.get_parameter('claim_renew_sec').value)
        self.claim_lease_ns = int(self.claim_lease_sec * 1e9)
        self.corner_recovery_max_tries = int(self.get_parameter('corner_recovery_max_tries').value)
        self.corner_recovery_back_sec = CORNER_RECOVERY_BACK_SEC
        self.corner_recovery_back_speed = CORNER_RECOVERY_BACK_SPEED
        self.corner_recovery_turn_sec = CORNER_RECOVERY_TURN_SEC
        self.corner_recovery_turn_speed = CORNER_RECOVERY_TURN_SPEED
        self.corner_recovery_escape_dist = CORNER_RECOVERY_ESCAPE_DIST
        self.stall_timeout_sec = STALL_TIMEOUT_SEC
        self.stall_progress_eps = float(self.get_parameter('stall_progress_eps').value)
        self.stall_watch_period = float(self.get_parameter('stall_watch_period').value)
        # Armed flag: set True immediately if auto_start=True (test mode),
        # otherwise stays False until _mission_armed_cb() fires asynchronously.
        # No blocking wait — _try_start() polls this flag via a 1 Hz timer.
        self.armed = bool(self.get_parameter('auto_start').value)
        self._wait_armed_logged = False
        self._fresh_timer = None

        self.items       = parse_sdf(SDF_PATH)
        self.state       = IDLE
        self.wp_done     = 0
        self.is_failed   = False
        self.override    = False

        self.cur_item    = None
        self.cur_skey    = None
        self.cur_side    = None
        self.cur_section = 0
        self._nav_queue  = []       # list of (x,y,qz,qw)
        self._final_goal  = None
        self._section_goal = None
        self._awaiting_final = False
        self._awaiting_final_yaw = False
        self._candidate_order = []
        self._candidate_ok = []
        self._candidate_idx = 0
        self._item_side_queue = []
        self._deferred_queue = []
        self._deferred_keys = set()
        self._retry_queue = []
        self._awaiting_final_path = False
        self._active_goal = None
        self._recovery_stage = None
        self._recovery_until_ns = 0
        self._recovery_attempts = 0
        self._recovery_turn_sign = 1.0
        self._recovery_escape_active = False
        self._stall_recovery_attempts = 0

        self.face_yaw    = 0.0
        self.par_yaw     = 0.0
        self._adv_sx     = None
        self._adv_sy     = None
        self._adv_target_dist = None
        self._ftimer     = None

        self.cur_x   = self.spawn_x
        self.cur_y   = self.spawn_y
        self.cur_yaw = 0.0
        self.amcl    = None

        # initialize progress tracking after cur_x/cur_y are set
        self._last_progress_ns = self._now_ns()
        self._last_progress_x = self.cur_x
        self._last_progress_y = self.cur_y
        self._pending = []
        self._side_claims = {}

        self.ac = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.pc = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self._gh = None

        self.cv_pub  = self.create_publisher(Twist,  "cmd_vel", 10)
        self.sa_pub  = self.create_publisher(String, "/shelf_arrived", 10)
        self.rem_pub = self.create_publisher(String, "remaining_waypoints", 10)
        self.cmp_pub = self.create_publisher(String, "completed_items", 10)
        self.claim_pub = self.create_publisher(String, "/side_claims", 50)

        self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self._amcl_cb, AMCL_QOS)
        self.create_subscription(String, "add_waypoints", self._addwp_cb, 10)
        self.create_subscription(String, "status",        self._status_cb, 10)
        self.create_subscription(String, "/side_claims", self._claim_cb, 50)
        self.create_subscription(
            BoolMsg, "/mission_armed", self._mission_armed_cb, MISSION_ARMED_QOS)

        self.create_timer(2.0,  self._pub_rem)
        self.create_timer(1.0,  self._try_start)
        self.create_timer(0.05, self._tick)
        self.create_timer(max(0.5, self.claim_renew_sec), self._renew_claim_tick)
        self.create_timer(max(0.2, self.stall_watch_period), self._stall_watchdog)
        self._started = False

        arm_state = "armed" if self.armed else "disarmed (WAIT-ARMED)"
        self.get_logger().info(
            f"[{self.robot_name}] init | {len(self.items)} items | limit={self.wp_limit} | {arm_state}")

    def _mission_armed_cb(self, msg: BoolMsg):
        if not msg.data:
            return
        self._set_armed('mission_armed')

    def _set_armed(self, source: str):
        if self.armed:
            return
        self.armed = True
        self._wait_armed_logged = False
        self.get_logger().info(f"[{self.robot_name}] MISSION-ARMED via {source}")
        self._try_start()

    def _amcl_cb(self, msg):
        self.amcl  = msg
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y
        self.cur_yaw = _yaw(msg)
        if _dist(self.cur_x, self.cur_y, self._last_progress_x, self._last_progress_y) >= self.stall_progress_eps:
            self._last_progress_ns = self._now_ns()
            self._last_progress_x = self.cur_x
            self._last_progress_y = self.cur_y

    def _localization_fresh(self):
        """Return (fresh: bool, age: float, sigma: float).
        Age is seconds since AMCL header stamp. Sigma is euclidean pos std.
        """
        if self.amcl is None:
            return False, None, None
        now_sec = float(self.get_clock().now().nanoseconds) * 1e-9
        st = self.amcl.header.stamp
        stamp_sec = float(st.sec) + float(st.nanosec) * 1e-9
        age = now_sec - stamp_sec
        try:
            sx = math.sqrt(abs(self.amcl.pose.covariance[0]))
            sy = math.sqrt(abs(self.amcl.pose.covariance[7]))
        except Exception:
            sx = sy = float('inf')
        sigma = math.hypot(sx, sy)
        age_ok = True
        if self.require_amcl_age_gate and self.max_amcl_age_sec > 0.0:
            age_ok = age <= self.max_amcl_age_sec
        sigma_ok = True if self.max_amcl_pos_sigma <= 0.0 else (sigma <= self.max_amcl_pos_sigma)
        fresh = age_ok and sigma_ok
        return fresh, age, sigma

    def _fmt_loc_value(self, value, digits, unit=''):
        if value is None:
            return f"n/a{unit}"
        return f"{value:.{digits}f}{unit}"

    def _schedule_fresh_retry(self, resume_cb, delay=1.0):
        # create a one-shot timer to call resume_cb when localization may be fresher
        if getattr(self, '_fresh_timer', None) is not None:
            return
        def _cb():
            try:
                if self._fresh_timer:
                    self._fresh_timer.cancel()
            except Exception:
                pass
            self._fresh_timer = None
            resume_cb()
        self._fresh_timer = self.create_timer(delay, _cb)

    def _corner_recovery_target_yaw(self):
        if self.cur_side is not None and self.cur_side.get("axis") == "section_line":
            pts = self.cur_side.get("section_points", [])
            if pts:
                idx = min(max(self.cur_section, 0), len(pts) - 1)
                if idx + 1 < len(pts):
                    tx, ty, _ = pts[idx + 1]
                else:
                    tx, ty, _ = pts[idx]
                return _travel_yaw(self.cur_x, self.cur_y, tx, ty)
        if self._active_goal is not None:
            tx, ty, _, _ = self._active_goal
            return _travel_yaw(self.cur_x, self.cur_y, tx, ty)
        if self.cur_side is not None:
            return math.atan2(2.0 * self.cur_side.get("qz", 0.0) * self.cur_side.get("qw", 1.0), 1.0 - 2.0 * self.cur_side.get("qz", 0.0) * self.cur_side.get("qz", 0.0))
        return self.cur_yaw

    def _side_facing_yaw(self):
        if self.cur_side is None:
            return self.cur_yaw
        side_name = self.cur_side.get("side")
        if side_name == "yplus":
            return -math.pi / 2.0
        if side_name == "yminus":
            return math.pi / 2.0
        if side_name == "xminus":
            return 0.0
        if side_name == "xplus":
            return math.pi
        qz = float(self.cur_side.get("qz", 0.0))
        qw = float(self.cur_side.get("qw", 1.0))
        return math.atan2(2.0 * qz * qw, 1.0 - 2.0 * qz * qz)

    def _current_section_target(self):
        if self.cur_side is None:
            return None
        pts = self.cur_side.get("section_points", [])
        if pts:
            idx = min(max(self.cur_section, 0), len(pts) - 1)
            return float(pts[idx][0]), float(pts[idx][1])
        if self._final_goal is not None:
            return float(self._final_goal[0]), float(self._final_goal[1])
        if self.cur_skey is not None:
            return float(self.cur_side.get("approach_x", self.cur_x)), float(self.cur_side.get("approach_y", self.cur_y))
        return None

    def _current_goal_id(self):
        if self.cur_item is None or self.cur_skey is None or self.cur_side is None:
            return None
        labels = self._side_section_labels(self.cur_side)
        if 0 <= self.cur_section < len(labels):
            return f"{self.cur_item}_{self.cur_skey}_{labels[self.cur_section]}"
        return f"{self.cur_item}_{self.cur_skey}"

    def _current_item_done_set(self):
        item = self.items.get(self.cur_item)
        if item is None:
            return set()
        done = item.get("done", set())
        return done if isinstance(done, set) else set(done)

    def _side_section_labels(self, side):
        if side is None:
            return []
        if "section_labels" in side and side["section_labels"]:
            return side["section_labels"]
        if "section_labels_template" in side and side["section_labels_template"]:
            return side["section_labels_template"]
        for pts_key in ("section_points", "section_points_template"):
            pts = side.get(pts_key, [])
            if pts and len(pts[0]) > 2:
                return [p[2] for p in pts]
        sections = int(side.get("sections", 1))
        return [chr(ord('A') + i) for i in range(sections)]

    def _next_section_index(self, side, side_key=None):
        if side is None:
            return None
        labels = self._side_section_labels(side)
        done = self._current_item_done_set()
        key = side_key or self.cur_skey or ""
        for idx, label in enumerate(labels):
            if f"{key}_{label}" not in done:
                return idx
        if labels:
            return len(labels) - 1
        sections = int(side.get("sections", 1))
        return 0 if sections <= 0 else min(max(self.cur_section, 0), sections - 1)

    def _side_has_remaining_sections(self, side, side_key=None):
        if side is None:
            return False
        labels = self._side_section_labels(side)
        done = self._current_item_done_set()
        key = side_key or self.cur_skey or ""
        for label in labels:
            if f"{key}_{label}" not in done:
                return True
        return False

    def _schedule_section_nav(self, delay=SWEEP_WAIT_SEC):
        if getattr(self, '_ftimer', None) is not None:
            try:
                self._ftimer.cancel()
            except Exception:
                pass
            self._ftimer = None

        def _cb():
            try:
                if self._ftimer:
                    self._ftimer.cancel()
            except Exception:
                pass
            self._ftimer = None
            self._send_final_nav()

        self._ftimer = self.create_timer(delay, _cb)

    def _start_corner_recovery(self):
        if self._recovery_attempts >= self.corner_recovery_max_tries:
            return False
        if self.cur_side is None or self.cur_skey is None:
            return False
        self._stop()
        self._recovery_attempts += 1
        self._stall_recovery_attempts = self._recovery_attempts
        self._recovery_stage = "back"
        self._recovery_until_ns = self._now_ns() + int(self.corner_recovery_back_sec * 1e9)
        target_yaw = self._corner_recovery_target_yaw()
        diff = _adiff(target_yaw, self.cur_yaw)
        self._recovery_turn_sign = 1.0 if diff >= 0.0 else -1.0
        self._recovery_escape_active = False
        self.state = RECOVER_BACKING
        self.get_logger().warn(
            f"[{self.robot_name}] CORNER-RECOVERY start side={self.cur_skey} attempt={self._recovery_attempts}/{self.corner_recovery_max_tries}")
        return True

    def _send_corner_recovery_escape_goal(self):
        if self.cur_side is None or self.cur_skey is None:
            return False
        target_yaw = self._corner_recovery_target_yaw()
        escape_dist = max(0.25, float(self.corner_recovery_escape_dist))
        escape_x = self.cur_x - escape_dist * math.cos(target_yaw)
        escape_y = self.cur_y - escape_dist * math.sin(target_yaw)
        qz, qw = _yaw_to_quat(target_yaw)
        self._recovery_escape_active = True
        self._recovery_stage = "escape"
        self.state = NAVIGATING
        self._active_goal = (escape_x, escape_y, qz, qw)
        self._awaiting_final = False
        self._awaiting_final_path = False
        self._nav_queue = []
        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose_stamped(escape_x, escape_y, qz, qw)
        try:
            self.get_logger().warn(
                f"[{self.robot_name}] CORNER-ESCAPE goal {escape_x:.3f} {escape_y:.3f} yaw={target_yaw:.3f} side={self.cur_skey}")
        except Exception:
            pass
        fut = self.ac.send_goal_async(goal)
        fut.add_done_callback(self._gr_cb)
        return True

    def _send_final_yaw_goal(self):
        if self.cur_side is None or self.cur_skey is None:
            return False
        target_yaw = self._side_facing_yaw()
        qz, qw = _yaw_to_quat(target_yaw)
        yaw_x = float(self.cur_x)
        yaw_y = float(self.cur_y)
        self._awaiting_final_yaw = True
        self.state = NAVIGATING
        self._active_goal = (yaw_x, yaw_y, qz, qw)
        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose_stamped(yaw_x, yaw_y, qz, qw)
        try:
            self.get_logger().warn(
                f"[{self.robot_name}] FINAL-YAW goal {yaw_x:.3f} {yaw_y:.3f} yaw={target_yaw:.3f} side={self.cur_skey}")
        except Exception:
            pass
        fut = self.ac.send_goal_async(goal)
        fut.add_done_callback(self._gr_cb)
        return True

    def _retry_current_section(self, delay=None):
        if self.cur_side is None or self.cur_skey is None:
            return False
        self._awaiting_final = False
        self._awaiting_final_path = False
        self._active_goal = None
        self._nav_queue = []
        self._candidate_idx = 0
        self._candidate_ok = [False] * len(self._candidate_order)
        self._candidate_paths = [None] * len(self._candidate_order)
        self._last_progress_ns = self._now_ns()
        self._last_progress_x = self.cur_x
        self._last_progress_y = self.cur_y
        if delay is None or delay <= 0.0:
            self._send_final_nav()
        else:
            self._schedule_fresh_retry(self._send_final_nav, delay=delay)
        return True

    def _finish_corner_recovery(self):
        self._stop()
        self.state = IDLE
        self._recovery_stage = None
        self._recovery_until_ns = 0
        self._gh = None
        self._active_goal = None
        self._candidate_idx = 0
        self._candidate_ok = [False] * len(self._candidate_order)
        self._candidate_paths = [None] * len(self._candidate_order)
        self._last_progress_ns = self._now_ns()
        self._last_progress_x = self.cur_x
        self._last_progress_y = self.cur_y
        if self._send_corner_recovery_escape_goal():
            return
        self.get_logger().warn(
            f"[{self.robot_name}] CORNER-RECOVERY retry {self.cur_skey}")
        self._schedule_fresh_retry(self._check_next_candidate_path, delay=0.2)

    def _stall_watchdog(self):
        # don't run watchdog before node has completed startup
        if not getattr(self, '_started', False):
            return
        if self.is_failed or self.state in (IDLE, RECOVER_BACKING, RECOVER_TURNING):
            return
        if self.cur_side is None or self.cur_skey is None:
            return
        if self._gh is None and self._active_goal is None:
            return
        if self._now_ns() - self._last_progress_ns < int(self.stall_timeout_sec * 1e9):
            return
        self.get_logger().warn(
            f"[{self.robot_name}] STALL-DETECTED no_progress>{self.stall_timeout_sec:.1f}s side={self.cur_skey}")
        if self._start_corner_recovery():
            try:
                if self._gh:
                    self._gh.cancel_goal_async()
            except Exception:
                pass
            self._gh = None
            self._active_goal = None
            return
        self._queue_current_side_to_end()

    def _stall_recovery_due(self):
        if self.is_failed:
            return False
        if self.cur_side is None or self.cur_skey is None:
            return False
        if self._now_ns() - self._last_progress_ns < int(self.stall_timeout_sec * 1e9):
            return False
        return True

    def _queue_current_side_to_end(self):
        if self.cur_item is None or self.cur_skey is None or self.cur_side is None:
            self._continue_current_item_or_select_next()
            return
        side_entry = {
            "item_name": self.cur_item,
            "side_key": self.cur_skey,
            "side": dict(self.cur_side),
        }
        self._retry_queue.append(side_entry)
        self.get_logger().warn(
            f"[{self.robot_name}] requeued {self.cur_skey} to retry queue end")
        self._stop()
        self._gh = None
        self._active_goal = None
        self._nav_queue = []
        self._candidate_idx = 0
        self._candidate_ok = [False] * len(self._candidate_order)
        self._candidate_paths = [None] * len(self._candidate_order)
        self._continue_current_item_or_select_next()

    def _now_ns(self):
        return int(self.get_clock().now().nanoseconds)

    def _prune_expired_claims(self):
        now_ns = self._now_ns()
        expired = [
            side_key for side_key, claim in self._side_claims.items()
            if int(claim.get("expire_at_ns", 0)) <= now_ns
        ]
        for side_key in expired:
            self._side_claims.pop(side_key, None)

    def _apply_claim_event(self, robot_id, side_key, action, expire_at_ns):
        if not side_key or not robot_id:
            return
        if action == "release":
            current = self._side_claims.get(side_key)
            if current is None or current.get("robot_id") == robot_id:
                self._side_claims.pop(side_key, None)
            return
        if action not in ("claim", "renew"):
            return
        self._side_claims[side_key] = {
            "robot_id": robot_id,
            "expire_at_ns": int(expire_at_ns),
        }

    def _publish_claim(self, action, side_key):
        if not side_key:
            return
        now_ns = self._now_ns()
        expire_at_ns = now_ns if action == "release" else now_ns + self.claim_lease_ns
        payload = {
            "robot_id": self.robot_name,
            "side_key": side_key,
            "action": action,
            "timestamp_ns": now_ns,
            "expire_at_ns": expire_at_ns,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.claim_pub.publish(msg)
        self._apply_claim_event(self.robot_name, side_key, action, expire_at_ns)

    def _claim_cb(self, msg: String):
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
        self._apply_claim_event(robot_id, side_key, action, expire_at_ns)

    def _is_side_claimed_by_other(self, side_key):
        self._prune_expired_claims()
        return _is_claimed_by_other(
            side_key, self.robot_name, self._side_claims, self._now_ns())

    def _try_claim_side(self, side_key):
        if not side_key:
            return False
        self._prune_expired_claims()
        if self._is_side_claimed_by_other(side_key):
            return False
        self._publish_claim("claim", side_key)
        return True

    def _release_side_claim(self, side_key):
        if not side_key:
            return
        self._publish_claim("release", side_key)

    def _renew_claim_tick(self):
        if self.is_failed:
            return
        if self.state == IDLE:
            return
        if self.cur_skey:
            self._publish_claim("renew", self.cur_skey)

    def _make_pose_stamped(self, x, y, qz=0.0, qw=1.0):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        return pose

    def _try_start(self):
        # Called by a 1 Hz timer — never blocks the executor.
        # Each condition (armed, amcl, action server, localization) is a
        # non-blocking check; if any fails we simply return and the timer
        # will call us again next second.
        if self._started or self.amcl is None: return
        if not self.armed:
            if not self._wait_armed_logged:
                self._wait_armed_logged = True
                self.get_logger().info(
                    f"[{self.robot_name}] WAIT-ARMED — Nav2 may be ready; "
                    f"no goals until /mission_armed is latched "
                    f"(pub /mission_start or auto_start:=true)")
            return
        if not self.ac.server_is_ready() or not self.pc.server_is_ready(): return
        # gate start on localization freshness
        fresh, age, sigma = self._localization_fresh()
        if not fresh:
            self.get_logger().warn(
                f"[{self.robot_name}] WAIT-LOC age={self._fmt_loc_value(age, 1, 's')} "
                f"sigma={self._fmt_loc_value(sigma, 3)} -> delaying start")
            self._schedule_fresh_retry(self._try_start, delay=1.0)
            return
        self._started = True
        self.get_logger().info(f"[{self.robot_name}] Nav2 ready")
        side = self.items[self.first_item]["sides"][self.first_side]
        self._go(self.first_item, side["key"], side)

    # ── 2-step nav ────────────────────────────────────────────────────────────

    def _go(self, iname, skey, side):
        if self.is_failed: return
        if not self._try_claim_side(skey):
            self.get_logger().warn(
                f"[{self.robot_name}] side already claimed by another robot: {skey}")
            self._schedule_fresh_retry(self._select, delay=0.2)
            return
        if side.get("section_points_template"):
            side = _prepare_section_line_side_for_pose(side, self.cur_x, self.cur_y)
        self.cur_item    = iname
        self.cur_skey    = skey
        self.cur_side    = side
        next_section = self._next_section_index(side, skey)
        self.cur_section = 0 if next_section is None else next_section
        self.state       = NAVIGATING
        self._stall_recovery_attempts = 0
        self._last_progress_ns = self._now_ns()
        self._last_progress_x = self.cur_x
        self._last_progress_y = self.cur_y
        ax, ay = side["approach_x"], side["approach_y"]
        # Build two candidate approach points:
        # A: keep current x, move in y to approach_y
        # B: keep current y, move in x to approach_x
        cand_a = (round(self.cur_x, 3), float(ay))
        cand_b = (float(ax), round(self.cur_y, 3))

        # Order candidates by robot-specific priority
        if self.robot_name == "robot2":
            ordered = [cand_b, cand_a]
        else:
            ordered = [cand_a, cand_b]

        nav_queue = []
        for (tx, ty) in ordered:
            nav_yaw = _travel_yaw(self.cur_x, self.cur_y, tx, ty)
            qz, qw = _yaw_to_quat(nav_yaw)
            nav_queue.append((tx, ty, qz, qw))

        # store final approach (ax,ay) to be sent after a successful candidate
        self._final_goal = (float(ax), float(ay))
        self._awaiting_final = False

        self._candidate_order = nav_queue
        self._candidate_ok = [False] * len(nav_queue)
        self._candidate_idx = 0
        self._candidate_paths = [None] * len(nav_queue)
        self._nav_queue = []
        self._active_goal = None
        self._section_goal = None
        self._recovery_attempts = 0
        self._recovery_stage = None
        goal_id = self._current_goal_id() or skey
        self.get_logger().info(
            f"[{self.robot_name}] -> {goal_id} candidates={[ (round(a,3),round(b,3)) for (a,b,_,_) in self._candidate_order ]} ")

        if not self.pc.server_is_ready():
            self.get_logger().warn(f"[{self.robot_name}] planner server not ready, skipping navigation request")
            return

        self._check_next_candidate_path()

    def _check_next_candidate_path(self):
        # ensure localization is fresh before planning
        fresh, age, sigma = self._localization_fresh()
        if not fresh:
            self.get_logger().warn(
                f"[{self.robot_name}] WAIT-LOC age={self._fmt_loc_value(age, 1, 's')} "
                f"sigma={self._fmt_loc_value(sigma, 3)} -> delaying path checks")
            self._schedule_fresh_retry(self._check_next_candidate_path, delay=1.0)
            return
        if self._candidate_idx >= len(self._candidate_order):
            reachable = [i for i, ok in enumerate(self._candidate_ok) if ok]
            if reachable:
                chosen = reachable[0]
                self._nav_queue = [self._candidate_order[chosen]]
                # store chosen candidate path for use when creating final yaw
                self._chosen_candidate_idx = chosen
                self._chosen_candidate_path = self._candidate_paths[chosen]
                self._next_nav()
                return

            self.get_logger().warn(f"[{self.robot_name}] no reachable candidates for {self.cur_skey}")
            self._nav_queue = []
            self._send_final_nav()
            return

        tx, ty, qz, qw = self._candidate_order[self._candidate_idx]
        goal = ComputePathToPose.Goal()
        goal.start = self._make_pose_stamped(self.cur_x, self.cur_y, math.sin(self.cur_yaw / 2.0), math.cos(self.cur_yaw / 2.0))
        goal.goal = self._make_pose_stamped(tx, ty, qz, qw)
        goal.use_start = True
        goal.planner_id = ""
        fut = self.pc.send_goal_async(goal)
        fut.add_done_callback(self._path_gr_cb)

    def _path_gr_cb(self, fut):
        gh = fut.result()
        idx = self._candidate_idx
        if not gh.accepted:
            self.get_logger().warn(f"[{self.robot_name}] path check rejected for candidate {idx}")
            self._candidate_ok[idx] = False
            self._candidate_idx += 1
            self._check_next_candidate_path()
            return
        self._candidate_path_gh = gh
        gh.get_result_async().add_done_callback(self._path_res_cb)

    def _path_res_cb(self, fut):
        r = fut.result()
        idx = self._candidate_idx
        result = r.result
        reachable = (result.error_code == result.NONE) and (len(result.path.poses) > 0)
        self._candidate_ok[idx] = reachable
        # store planner path for later use when setting final goal yaw
        try:
            self._candidate_paths[idx] = result.path
        except Exception:
            self._candidate_paths[idx] = None
        state = "reachable" if reachable else f"blocked({result.error_code})"
        self.get_logger().info(f"[{self.robot_name}] path check {idx} -> {state}")
        self._candidate_idx += 1
        self._check_next_candidate_path()

    def _next_nav(self):
        if not self._nav_queue:
            self._defer_current_side()
            return
        # ensure localization is fresh before sending a navigation goal
        fresh, age, sigma = self._localization_fresh()
        if not fresh:
            self.get_logger().warn(
                f"[{self.robot_name}] WAIT-LOC age={self._fmt_loc_value(age, 1, 's')} "
                f"sigma={self._fmt_loc_value(sigma, 3)} -> postponing SEND_GOAL")
            self._schedule_fresh_retry(self._next_nav, delay=1.0)
            return
        x, y, qz, qw = self._nav_queue.pop(0)
        self._active_goal = (x, y, qz, qw)
        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose_stamped(x, y, qz, qw)
        # log exact goal being sent (x, y, yaw) for external trackers
        try:
            yaw = 2.0 * math.atan2(qz, qw)
            goal_id = self._current_goal_id() or self.cur_skey or "unknown"
            self.get_logger().info(f"[{self.robot_name}] SENT_GOAL {goal_id} {x:.3f} {y:.3f} {yaw:.3f}")
        except Exception:
            pass
        fut = self.ac.send_goal_async(goal)
        fut.add_done_callback(self._gr_cb)

    def _send_final_nav(self):
        # send the actual approach point (final goal) after candidate succeeded
        # gate on localization freshness before computing final path
        fresh, age, sigma = self._localization_fresh()
        if not fresh:
            self.get_logger().warn(
                f"[{self.robot_name}] WAIT-LOC age={self._fmt_loc_value(age, 1, 's')} "
                f"sigma={self._fmt_loc_value(sigma, 3)} -> delaying final approach")
            self._schedule_fresh_retry(self._send_final_nav, delay=1.0)
            return
        target = self._current_section_target()
        if target is None:
            if not hasattr(self, '_final_goal') or self._final_goal is None:
                self._next_nav()
                return
            target = (float(self._final_goal[0]), float(self._final_goal[1]))

        ax, ay = target
        self._section_goal = (float(ax), float(ay))
        nav_yaw = _travel_yaw(self.cur_x, self.cur_y, ax, ay)
        qz, qw = _yaw_to_quat(nav_yaw)

        if _dist(self.cur_x, self.cur_y, ax, ay) <= FINAL_GOAL_TOL:
            self._awaiting_final = True
            self._send_final_goal(ax, ay, qz, qw, nav_yaw)
            return

        self._awaiting_final_path = True
        goal = ComputePathToPose.Goal()
        goal.start = self._make_pose_stamped(self.cur_x, self.cur_y, math.sin(self.cur_yaw / 2.0), math.cos(self.cur_yaw / 2.0))
        goal.goal = self._make_pose_stamped(ax, ay, qz, qw)
        goal.use_start = True
        goal.planner_id = ""
        fut = self.pc.send_goal_async(goal)
        fut.add_done_callback(self._final_path_gr_cb)
        return

    def _send_final_goal(self, ax, ay, qz, qw, nav_yaw=None):
        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose_stamped(ax, ay, qz, qw)
        self._awaiting_final = True
        if nav_yaw is None:
            try:
                nav_yaw = 2.0 * math.atan2(qz, qw)
            except Exception:
                nav_yaw = 0.0
        self._active_goal = (ax, ay, qz, qw)
        # log exact final goal being sent (x, y, yaw) for external trackers
        try:
            goal_id = self._current_goal_id() or self.cur_skey or "unknown"
            self.get_logger().info(f"[{self.robot_name}] SENT_GOAL {goal_id} {ax:.3f} {ay:.3f} {nav_yaw:.3f}")
        except Exception:
            pass
        fut = self.ac.send_goal_async(goal)
        fut.add_done_callback(self._gr_cb)

    def _final_path_gr_cb(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().warn(f"[{self.robot_name}] final approach path rejected")
            self._awaiting_final_path = False
            if self._nav_queue:
                self._next_nav()
                return
            self._defer_current_side()
            return
        gh.get_result_async().add_done_callback(self._final_path_res_cb)

    def _final_path_res_cb(self, fut):
        r = fut.result()
        self._awaiting_final_path = False
        result = r.result
        reachable = (result.error_code == result.NONE) and (len(result.path.poses) > 0)
        if not reachable:
            self.get_logger().warn(f"[{self.robot_name}] final approach path blocked, try next candidate")
            if self._nav_queue:
                self._next_nav()
                return
            if self._retry_current_section(delay=0.5):
                return
            self._defer_current_side()
            return

        if self._final_goal is None:
            self._continue_current_item_or_select_next()
            return

        if self._section_goal is not None:
            ax, ay = self._section_goal
        else:
            ax, ay = self._final_goal
        nav_yaw = _travel_yaw(self.cur_x, self.cur_y, ax, ay)
        qz, qw = _yaw_to_quat(nav_yaw)
        self._send_final_goal(ax, ay, qz, qw, nav_yaw)

    def _gr_cb(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().warn(f"[{self.robot_name}] goal rejected")
            if self._recovery_escape_active:
                self._recovery_escape_active = False
                self._queue_current_side_to_end()
                return
            if self.state == NAVIGATING and self._stall_recovery_due():
                if self._start_corner_recovery():
                    return
            # Nav2 can transiently reject when lifecycle/BT is not ready yet.
            # Avoid flooding new sides; retry current side after a short delay.
            if self.state == NAVIGATING and self.cur_side is not None and self.cur_item is not None:
                self._awaiting_final = False
                self._awaiting_final_path = False
                self._nav_queue = []
                self._candidate_idx = 0
                self._candidate_ok = [False] * len(self._candidate_order)
                self._candidate_paths = [None] * len(self._candidate_order)
                self.get_logger().warn(
                    f"[{self.robot_name}] retry current side after reject: {self.cur_skey}")
                self._schedule_fresh_retry(self._check_next_candidate_path, delay=self.goal_reject_retry_sec)
                return

            if self.cur_side is not None and self.cur_skey is not None:
                if self._retry_current_section(delay=self.goal_reject_retry_sec):
                    return

            # if this was a final goal attempt, try next candidate
            if getattr(self, '_awaiting_final', False):
                self._awaiting_final = False
                if self._nav_queue:
                    self._next_nav()
                    return
            else:
                if self._nav_queue:
                    self._next_nav()
                    return
            self._nav_queue = []
            if self._stall_recovery_attempts > 0:
                self._queue_current_side_to_end()
            else:
                self._defer_current_side()
            return
        self._gh = gh
        gh.get_result_async().add_done_callback(self._res_cb)

    def _res_cb(self, fut):
        self._gh = None
        r = fut.result()
        if r.status == 4:
            # succeeded
            if self._recovery_escape_active:
                self._recovery_escape_active = False
                self._active_goal = None
                self._recovery_attempts = 0
                self._last_progress_ns = self._now_ns()
                self._last_progress_x = self.cur_x
                self._last_progress_y = self.cur_y
                self.get_logger().warn(
                    f"[{self.robot_name}] CORNER-ESCAPE done -> replan {self.cur_skey}")
                self._schedule_fresh_retry(self._check_next_candidate_path, delay=0.2)
                return
            if self._awaiting_final_yaw:
                self._awaiting_final_yaw = False
                self._active_goal = None
                self._final_goal = None
                self._section_goal = None
                self._recovery_attempts = 0
                self._last_progress_ns = self._now_ns()
                self._last_progress_x = self.cur_x
                self._last_progress_y = self.cur_y
                self._on_facing()
                return
            self._recovery_attempts = 0
            self._active_goal = None
            if getattr(self, '_awaiting_final', False):
                # final goal succeeded -> send a second Nav2 goal to face the shelf
                self._awaiting_final = False
                self._send_final_yaw_goal()
            else:
                # candidate succeeded -> send final approach
                self._send_final_nav()
        else:
            self.get_logger().warn(
                f"[{self.robot_name}] nav step failed={r.status}, try next candidate {self.cur_skey}")
            if self._recovery_escape_active:
                self._recovery_escape_active = False
                if self._stall_recovery_attempts > 0:
                    self._queue_current_side_to_end()
                else:
                    self._defer_current_side()
                return
            if self._awaiting_final_yaw:
                self._awaiting_final_yaw = False
                if self._stall_recovery_due() and self._start_corner_recovery():
                    return
                if self._stall_recovery_attempts > 0:
                    self._queue_current_side_to_end()
                else:
                    self._defer_current_side()
                return
            if self.state == NAVIGATING and self._stall_recovery_due():
                if self._start_corner_recovery():
                    return
            if self.cur_side is not None and self.cur_skey is not None:
                if self._retry_current_section(delay=0.5):
                    return
            # try next candidate if available
            if getattr(self, '_awaiting_final', False):
                # final failed -> reset flag and try next candidate
                self._awaiting_final = False
                if self._nav_queue:
                    self._next_nav()
                    return
            else:
                if self._nav_queue:
                    self._next_nav()
                    return
            # try a local corner recovery once before deferring the side
            if self._start_corner_recovery():
                return
            # no candidates left -> defer this side to the end of the queue
            self._nav_queue = []
            if self._stall_recovery_attempts > 0:
                self._queue_current_side_to_end()
            else:
                self._defer_current_side()

    def _tick(self):
        if self.state == RECOVER_BACKING:
            if self._now_ns() >= self._recovery_until_ns:
                self._stop()
                self._recovery_stage = "turn"
                self._recovery_until_ns = self._now_ns() + int(self.corner_recovery_turn_sec * 1e9)
                self.state = RECOVER_TURNING
                return
            cmd = Twist()
            cmd.linear.x = self.corner_recovery_back_speed
            self.cv_pub.publish(cmd)
            return
        if self.state == RECOVER_TURNING:
            if self._now_ns() >= self._recovery_until_ns:
                self._finish_corner_recovery()
                return
            cmd = Twist()
            cmd.angular.z = self._recovery_turn_sign * self.corner_recovery_turn_speed
            self.cv_pub.publish(cmd)
            return
        if self.state == TURNING_TO:
            self._turn(self.face_yaw, FACING, self._on_facing)
        elif self.state == TURNING_BACK:
            self._turn(self.par_yaw, ADVANCING, self._on_parallel)
        elif self.state == ADVANCING:
            self._advance()

    def _turn(self, target, nxt, cb=None):
        diff = _adiff(target, self.cur_yaw)
        if abs(diff) < 0.10:
            self._stop()
            self.state = nxt
            if cb: cb()
        else:
            cmd = Twist()
            cmd.angular.z = math.copysign(max(0.2, min(0.6, abs(diff)*0.4)), diff)
            self.cv_pub.publish(cmd)

    def _on_facing(self):
        self._pub_arrived()
        self._ftimer = self.create_timer(SWEEP_WAIT_SEC, self._facing_done)

    def _facing_done(self):
        if self._ftimer:
            self._ftimer.cancel()
            self._ftimer = None
        if self.cur_side is None:
            return
        # mark this section as done (section-level semantics)
        sec_label = self._side_section_labels(self.cur_side)[self.cur_section] if self.cur_section < len(self._side_section_labels(self.cur_side)) else "?"
        combined = f"{self.cur_skey}_{sec_label}"
        item = self.items.get(self.cur_item)
        if item is not None:
            if combined not in item.get("done", set()):
                item.setdefault("done", set()).add(combined)
                self.wp_done += 1
                self._pub_completed()

        next_section = self._next_section_index(self.cur_side, self.cur_skey)
        if next_section is None or not self._side_has_remaining_sections(self.cur_side, self.cur_skey):
            self._finish()
        else:
            self.cur_section = next_section
            self._awaiting_final = False
            self._awaiting_final_yaw = False
            self._active_goal = None
            self._last_progress_ns = self._now_ns()
            self._last_progress_x = self.cur_x
            self._last_progress_y = self.cur_y
            self._schedule_section_nav(SWEEP_WAIT_SEC)

    def _on_parallel(self):
        self._adv_sx = self.cur_x
        self._adv_sy = self.cur_y
        s = self.cur_side
        if s and s.get("axis") == "section_line":
            pts = s.get("section_points", [])
            nxt = self.cur_section + 1
            if nxt < len(pts):
                x0, y0, _ = pts[self.cur_section]
                x1, y1, _ = pts[nxt]
                self._adv_target_dist = _dist(x0, y0, x1, y1)
                self.par_yaw = math.atan2(y1 - y0, x1 - x0)
            else:
                self._adv_target_dist = 0.0
        else:
            self._adv_target_dist = None

    def _advance(self):
        s = self.cur_side
        if s is None:
            return
        if self._adv_sx is None: self._on_parallel()
        traveled = _dist(self.cur_x, self.cur_y, self._adv_sx, self._adv_sy)
        target_dist = self._adv_target_dist if self._adv_target_dist is not None else abs(s["step"])
        reached_next_section = False
        if s and s.get("axis") == "section_line":
            pts = s.get("section_points", [])
            nxt = self.cur_section + 1
            if nxt < len(pts):
                reached_next_section = _dist(self.cur_x, self.cur_y, pts[nxt][0], pts[nxt][1]) <= SECTION_CENTER_TOL

        if traveled >= target_dist - 0.05 or reached_next_section:
            self._stop()
            self._adv_sx = self._adv_sy = None
            self._adv_target_dist = None
            self.cur_section += 1
            self.state = TURNING_TO
        else:
            cmd = Twist()
            cmd.linear.x = SWEEP_LIN_VEL
            self.cv_pub.publish(cmd)

    def _finish(self):
        if self.cur_side is not None and self._side_has_remaining_sections(self.cur_side, self.cur_skey):
            next_sec = self._next_section_index(self.cur_side, self.cur_skey)
            self.get_logger().warn(f"[{self.robot_name}] SIDE-CONTINUE-GUARD side={self.cur_skey} forcing next_section={next_sec}")
            self.cur_section = next_sec
            self._schedule_section_nav(SWEEP_WAIT_SEC)
            return

        item = self.items.get(self.cur_item)
        sec_label = self._side_section_labels(self.cur_side)[self.cur_section] if self.cur_section < len(self._side_section_labels(self.cur_side)) else "?"
        # side-level finish: sections have been marked individually
        # do not add side-key to done; item["done"] contains section ids
        self.get_logger().info(
            f"[{self.robot_name}] DONE {self.cur_skey} sec={sec_label} ({self.wp_done}/{self.wp_limit})")
        self._release_side_claim(self.cur_skey)
        if self.wp_done >= self.wp_limit:
            self.get_logger().info(
                f"[{self.robot_name}] All done ({self.wp_done}/{self.wp_limit})")
            return
        self._continue_current_item_or_select_next()

    def _continue_current_item_or_select_next(self):
        item = self.items.get(self.cur_item)
        if item is not None:
            # Check if there are other eligible sides for the CURRENT item
            next_side = select_next_side_for_item(
                item, self.robot_name, self.override, self._deferred_keys, self._side_claims, self._now_ns(), self.cur_x, self.cur_y
            )
            if next_side is not None:
                if next_side.get("section_points_template"):
                    from .sdf_parser import _prepare_section_line_side_for_pose
                    next_side = _prepare_section_line_side_for_pose(next_side, self.cur_x, self.cur_y)
                self.get_logger().info(f"[{self.robot_name}] SAME-ITEM pick item={self.cur_item} side={next_side['key']}")
                self._go(self.cur_item, next_side["key"], next_side)
                return

        # If no eligible sides remain for the current item, select a new item
        self._select()

    def _defer_current_side(self):
        if self.cur_item is None or self.cur_skey is None or self.cur_side is None:
            self._continue_current_item_or_select_next()
            return

        self._release_side_claim(self.cur_skey)

        deferred_key = f"{self.cur_item}:{self.cur_skey}"
        if deferred_key not in self._deferred_keys:
            self._deferred_keys.add(deferred_key)
            self._deferred_queue.append({
                "item_name": self.cur_item,
                "side_key": self.cur_skey,
                "side": dict(self.cur_side),
                "deferred_key": deferred_key,
            })
            self.get_logger().warn(
                f"[{self.robot_name}] deferred {self.cur_skey} to end of queue")

        self._continue_current_item_or_select_next()

    def _select_deferred(self):
        attempts = len(self._deferred_queue)
        while attempts > 0 and self._deferred_queue:
            entry = self._deferred_queue.pop(0)
            attempts -= 1
            item_name = entry.get("item_name")
            side = entry.get("side")
            if not side:
                continue
            item = self.items.get(item_name)
            if item is None:
                continue
            if item["region"] not in self.owned:
                continue
            if side.get("section_points_template"):
                side = _prepare_section_line_side_for_pose(side, self.cur_x, self.cur_y)
                entry["side_key"] = side["key"]
            side_key = entry.get("side_key")
            if not side_key:
                continue
            deferred_key = entry.get("deferred_key")
            deferred_view = set(self._deferred_keys)
            if deferred_key:
                deferred_view.discard(deferred_key)
            remaining = _remaining_sides(item, self.robot_name, self.override, deferred_view)
            if side_key not in {candidate["key"] for candidate in remaining}:
                continue
            if self._is_side_claimed_by_other(side_key):
                self._deferred_queue.append(entry)
                continue
            direction = _side_direction(side, self.cur_x, self.cur_y)
            if _priority_rank(direction, self.priority_dirs) is None:
                self._deferred_queue.append(entry)
                continue
            if deferred_key:
                self._deferred_keys.discard(deferred_key)
            if side.get("section_points_template"):
                side = _prepare_section_line_side_for_pose(side, self.cur_x, self.cur_y)
                entry["side_key"] = side["key"]
            self.get_logger().info(
                f"[{self.robot_name}] retry deferred {entry.get('side_key')} after normal queue")
            self._go(item_name, entry["side_key"], side)
            return True
        return False

    # ── selection ─────────────────────────────────────────────────────────────

    def _select(self):
        self.state = IDLE
        
        # SAME-SIDE-LOCK: Never select a new item if the current side still has remaining sections
        if self.cur_side is not None and self._side_has_remaining_sections(self.cur_side, self.cur_skey):
            resume_section = self._next_section_index(self.cur_side, self.cur_skey)
            self.get_logger().warn(
                f"[{self.robot_name}] SAME-SIDE-LOCK side={self.cur_skey} resume_section={resume_section}. Forcing continue.")
            self.cur_section = resume_section
            self._schedule_section_nav(SWEEP_WAIT_SEC)
            return

        # Check if there are unresolved unfinished sections from previous failed navs
        if self._retry_queue:
            entry = self._retry_queue.pop(0)
            side = entry.get("side")
            if side and side.get("section_points_template"):
                side = _prepare_section_line_side_for_pose(side, self.cur_x, self.cur_y)
            if entry.get("item_name") and entry.get("side_key") and side:
                self.get_logger().info(f"[{self.robot_name}] RETRY-QUEUE pick side={entry['side_key']} before new item selection")
                self._go(entry["item_name"], entry["side_key"], side)
                return

        # Handle pending sides (e.g. from first_item bootstrap)
        if self._pending:
            p = self._pending.pop(0)
            iname, skey, side = p.get("item_name"), p.get("side_key"), p.get("side")
            if iname and skey and side:
                if side.get("section_points_template"):
                    side = _prepare_section_line_side_for_pose(side, self.cur_x, self.cur_y)
                self._go(iname, skey, side)
                return

        # Normal selection strategy
        iname, next_side = select_next_item(
            self.items, self.robot_name, self.cur_x, self.cur_y,
            self.owned, self.wp_done, self.wp_limit, self.priority_dirs,
            self.override, self._deferred_keys, self._side_claims, self._now_ns())
        
        if iname is not None and next_side is not None:
            self._go(iname, next_side["key"], next_side)
            return

        if self._select_deferred():
            return
        
        self.get_logger().info(f"[{self.robot_name}] All done ({self.wp_done}/{self.wp_limit})")
    
    # ── pubs / callbacks ──────────────────────────────────────────────────────

    def _stop(self): self.cv_pub.publish(Twist())

    def _pub_arrived(self):
        s = self.cur_side
        if s is None:
            return
        ax, ay = s["approach_x"], s["approach_y"]
        if s.get("axis") == "section_line":
            pts = s.get("section_points", [])
            if 0 <= self.cur_section < len(pts):
                ax, ay = pts[self.cur_section][0], pts[self.cur_section][1]
        sec_label = _section_label(s, self.cur_section)
        combined_id = f"{self.cur_skey}_{sec_label}"
        msg = String()
        msg.data = json.dumps({
            "robot_id": self.robot_name, "shelf_id": combined_id,
            "side_id": self.cur_skey,
            "x": ax, "y": ay,
            "section": sec_label,
            "timestamp": self.get_clock().now().nanoseconds})
        self.sa_pub.publish(msg)

    def _pub_rem(self):
        msg = String()
        msg.data = json.dumps({
            "robot_id": self.robot_name,
            "count": max(0, self.wp_limit - self.wp_done),
            "remaining": []})
        self.rem_pub.publish(msg)

    def _pub_completed(self):
        keys = [k for item in self.items.values() for k in item["done"]]
        msg = String()
        msg.data = json.dumps({"robot_id": self.robot_name, "completed": keys})
        self.cmp_pub.publish(msg)

    def _addwp_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            if data.get("override_regions"): self.override = True
            wps = data.get("waypoints", [])
            if wps:
                self._pending.extend(wps)
                if self.state == IDLE: self._select()
        except json.JSONDecodeError:
            self.get_logger().error("add_waypoints: invalid JSON")

    def _status_cb(self, msg: String):
        if msg.data == "FAILED" and not self.is_failed:
            self.is_failed = True
            self._release_side_claim(self.cur_skey)
            self._stop()
            if self._gh:
                try: self._gh.cancel_goal_async()
                except: pass
                self._gh = None
            self.get_logger().warn(f"[{self.robot_name}] FAILED")


def main(args=None):
    print("[DEBUG] Waypoint Sender Başladı", flush=True)
    rclpy.init(args=args)
    node = WaypointSender()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()