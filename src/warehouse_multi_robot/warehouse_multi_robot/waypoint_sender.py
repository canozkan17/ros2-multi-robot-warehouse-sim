#!/usr/bin/env python3
"""
waypoint_sender.py — Hybrid Navigation Node
============================================
Nav2: 2-step approach per side (corridor entry → section start)
Sweep: cmd_vel direct, AMCL yaw reference, P-controller rotation
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
from std_msgs.msg import String

SDF_PATH = os.path.expanduser(
    "~/thesis_ws/src/warehouse_multi_robot/worlds/tugbot_warehouse_clean.sdf")

APPROACH_DIST     = 1.2
SMALL_SHELF_WIDTH = 3.6
SMALL_SECTIONS    = 4
SMALL_STEP        = SMALL_SHELF_WIDTH / SMALL_SECTIONS   # 0.9 m

BIG_SHELF_LENGTH  = 18.0
BIG_SECTIONS      = 4
BIG_STEP          = BIG_SHELF_LENGTH / BIG_SECTIONS      # 4.5 m

SWEEP_ANG_VEL     = 0.4
SWEEP_LIN_VEL     = 0.15
SWEEP_WAIT_SEC    = 2.0

Q_X = (0.0, 1.0)        # yaw=0  robot faces +X  (yminus/yplus sides)
Q_Y = (0.7071, 0.7071)  # yaw=90 robot faces +Y  (xminus/xplus sides)

ITEM_SIDE_OWNER = {
    "shelf_2_yplus":      "robot1", "shelf_2_yminus":     "robot3",
    "shelf_5_yminus":     "robot1", "shelf_5_yplus":      "robot2",
    "shelf_big_3_xplus":  "robot2", "shelf_big_3_xminus": "robot3",
}

ROBOT_CONFIG = {
    "robot1": {"spawn": (0.0,  1.0), "first_item": "shelf_2",
               "first_side": "yplus",  "owned_regions": [2,1,4],
               "priority_dirs": ["north", "east"], "wp_limit": 48},
    "robot2": {"spawn": (0.0,  0.0), "first_item": "shelf_big_3",
               "first_side": "xminus", "owned_regions": [4,3],
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
        points = [
            (round(cx + half - SMALL_STEP / 2.0 - i * SMALL_STEP, 3), approach_y, label)
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
        ordered = sorted(mobiles, key=lambda m: m["y"], reverse=False)
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
        if   t == "shelf":      sides = _small_shelf_sides(name, x, y)
        elif t == "shelf_big":  sides = _big_shelf_sides(name, x, y, yaw)
        else:                   sides = _pallet_sides(name, x, y)
        items[name] = {"name": name, "type": t, "x": x, "y": y,
                       "region": _region(x, y), "sides": sides, "done": set()}
        if name in MOBILE_SET:
            mobile_refs.append({"name": name, "x": x, "y": y})

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

def _remaining_sides(item, robot, override):
    sides = []
    for side in item["sides"].values():
        key = side["key"]
        if key in item["done"]:
            continue
        owner = ITEM_SIDE_OWNER.get(key)
        if owner and owner != robot and not override:
            continue
        sides.append(side)
    return sides

def _item_direction(item, cx, cy, priority_dirs):
    for direction in priority_dirs:
        if _dir_match(direction, cx, cy, item["x"], item["y"]):
            return direction
    return None

def _make_side_entry(item_name, side):
    return {"item_name": item_name, "side_key": side["key"], "side": side}

def select_next(items, robot, cx, cy, regions, done, limit, priority_dirs, override=False):
    if done >= limit:
        return None, []
    for region in regions:
        best = None
        for item in items.values():
            if item["region"] != region:
                continue
            remaining = _remaining_sides(item, robot, override)
            if not remaining:
                continue
            if item["type"] == "mobile_cluster":
                remaining = [_prepare_section_line_side_for_pose(side, cx, cy) for side in remaining]
            direction = _item_direction(item, cx, cy, priority_dirs)
            if direction is None:
                continue
            ordered = sorted(
                remaining,
                key=lambda side: (
                    _dist(cx, cy, side["approach_x"], side["approach_y"]),
                    side["key"],
                ),
            )
            score = _dist(cx, cy, ordered[0]["approach_x"], ordered[0]["approach_y"])
            candidate = (priority_dirs.index(direction), score, item["name"], ordered)
            if best is None or candidate < best:
                best = candidate
        if best is not None:
            return best[2], best[3]
    return None, []

IDLE, NAVIGATING, TURNING_TO, FACING, TURNING_BACK, ADVANCING = (
    "IDLE","NAVIGATING","TURNING_TO","FACING","TURNING_BACK","ADVANCING")

AMCL_QOS = QoSProfile(depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE)

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
        self._awaiting_final = False
        self._candidate_order = []
        self._candidate_ok = []
        self._candidate_idx = 0
        self._item_side_queue = []

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
        self._pending = []

        self.ac = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.pc = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self._gh = None

        self.cv_pub  = self.create_publisher(Twist,  "cmd_vel", 10)
        self.sa_pub  = self.create_publisher(String, "/shelf_arrived", 10)
        self.rem_pub = self.create_publisher(String, "remaining_waypoints", 10)
        self.cmp_pub = self.create_publisher(String, "completed_items", 10)

        self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self._amcl_cb, AMCL_QOS)
        self.create_subscription(String, "add_waypoints", self._addwp_cb, 10)
        self.create_subscription(String, "status",        self._status_cb, 10)

        self.create_timer(2.0,  self._pub_rem)
        self.create_timer(1.0,  self._try_start)
        self.create_timer(0.05, self._tick)
        self._started = False

        self.get_logger().info(
            f"[{self.robot_name}] init | {len(self.items)} items | limit={self.wp_limit}")

    def _amcl_cb(self, msg):
        self.amcl  = msg
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y
        self.cur_yaw = _yaw(msg)

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
        if self._started or self.amcl is None: return
        if not self.ac.server_is_ready() or not self.pc.server_is_ready(): return
        self._started = True
        self.get_logger().info(f"[{self.robot_name}] Nav2 ready")
        side = self.items[self.first_item]["sides"][self.first_side]
        self._go(self.first_item, side["key"], side)

    # ── 2-step nav ────────────────────────────────────────────────────────────

    def _go(self, iname, skey, side):
        if self.is_failed: return
        if side.get("section_points_template"):
            side = _prepare_section_line_side_for_pose(side, self.cur_x, self.cur_y)
        self.cur_item    = iname
        self.cur_skey    = skey
        self.cur_side    = side
        self.cur_section = 0
        self.state       = NAVIGATING
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
        self.get_logger().info(
            f"[{self.robot_name}] -> {skey} candidates={[ (round(a,3),round(b,3)) for (a,b,_,_) in self._candidate_order ]} ")

        if not self.pc.server_is_ready():
            self.get_logger().warn(f"[{self.robot_name}] planner server not ready, skipping navigation request")
            return

        self._check_next_candidate_path()

    def _check_next_candidate_path(self):
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

            item = self.items.get(self.cur_item)
            if item:
                item["done"].add(self.cur_skey)
            self._nav_queue = []
            self._select()
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
            # no more candidates -> begin sweep
            self._begin_sweep()
            return
        x, y, qz, qw = self._nav_queue.pop(0)
        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose_stamped(x, y, qz, qw)
        # log exact goal being sent (x, y, yaw) for external trackers
        try:
            yaw = 2.0 * math.atan2(qz, qw)
            self.get_logger().info(f"[{self.robot_name}] SENT_GOAL {x:.3f} {y:.3f} {yaw:.3f}")
        except Exception:
            pass
        fut = self.ac.send_goal_async(goal)
        fut.add_done_callback(self._gr_cb)

    def _send_final_nav(self):
        # send the actual approach point (final goal) after candidate succeeded
        if not hasattr(self, '_final_goal') or self._final_goal is None:
            self._next_nav()
            return
        ax, ay = self._final_goal
        nav_yaw = _travel_yaw(self.cur_x, self.cur_y, ax, ay)
        qz, qw = _yaw_to_quat(nav_yaw)
        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose_stamped(ax, ay, qz, qw)
        self._awaiting_final = True
        # log exact final goal being sent (x, y, yaw) for external trackers
        try:
            self.get_logger().info(f"[{self.robot_name}] SENT_GOAL {ax:.3f} {ay:.3f} {nav_yaw:.3f}")
        except Exception:
            pass
        fut = self.ac.send_goal_async(goal)
        fut.add_done_callback(self._gr_cb)

    def _gr_cb(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().warn(f"[{self.robot_name}] goal rejected")
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
            item = self.items.get(self.cur_item)
            if item:
                item["done"].add(self.cur_skey)
            self._nav_queue = []
            self._continue_current_item_or_select_next()
            return
        self._gh = gh
        gh.get_result_async().add_done_callback(self._res_cb)

    def _res_cb(self, fut):
        self._gh = None
        r = fut.result()
        if r.status == 4:
            # succeeded
            if getattr(self, '_awaiting_final', False):
                # final goal succeeded -> clear and proceed (begin sweep)
                self._awaiting_final = False
                # after final, clear final goal and start sweep
                self._final_goal = None
                # proceed to sweep (nav_queue should be empty)
                self._next_nav()
            else:
                # candidate succeeded -> send final approach
                self._send_final_nav()
        else:
            self.get_logger().warn(
                f"[{self.robot_name}] nav step failed={r.status}, try next candidate {self.cur_skey}")
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
            # no candidates left -> mark item done and select next
            item = self.items.get(self.cur_item)
            if item:
                item["done"].add(self.cur_skey)
            self._nav_queue = []
            self._continue_current_item_or_select_next()

    # ── sweep ─────────────────────────────────────────────────────────────────

    def _begin_sweep(self):
        self._stop()
        s = self.cur_side
        if s is None: return
        if   s["side"] == "yplus":  self.face_yaw = -math.pi/2.0
        elif s["side"] == "yminus": self.face_yaw =  math.pi/2.0
        elif s["side"] == "xminus": self.face_yaw =  0.0
        else:                       self.face_yaw =  math.pi

        # Sweep heading is chosen to match the travel direction, so the robot
        # moves forward during the scan instead of backing up.
        if s["axis"] == "section_line":
            pts = s.get("section_points", [])
            if len(pts) >= 2:
                self.par_yaw = math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0])
            else:
                self.par_yaw = math.atan2(2.0*s["qz"]*s["qw"], 1.0-2.0*s["qz"]*s["qz"])
        elif s["axis"] == "x":
            self.par_yaw = 0.0 if s["step"] > 0.0 else math.pi
        elif s["axis"] == "y":
            self.par_yaw = math.pi / 2.0 if s["step"] > 0.0 else -math.pi / 2.0
        else:
            self.par_yaw = math.atan2(2.0*s["qz"]*s["qw"], 1.0-2.0*s["qz"]*s["qz"])
        sec_label = _section_label(s, self.cur_section)
        self.get_logger().info(
            f"[{self.robot_name}] sweep {self.cur_skey} sec={sec_label} idx={self.cur_section} "
            f"face={math.degrees(self.face_yaw):.0f}deg "
            f"move={math.degrees(self.par_yaw):.0f}deg")
        self.state = TURNING_TO

    def _tick(self):
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
        if self.state != FACING: return
        if self.cur_section >= self.cur_side["sections"] - 1:
            self._finish()
        else:
            self.state = TURNING_BACK

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
        if self._adv_sx is None: self._on_parallel()
        traveled = _dist(self.cur_x, self.cur_y, self._adv_sx, self._adv_sy)
        target_dist = self._adv_target_dist if self._adv_target_dist is not None else abs(s["step"])
        if traveled >= target_dist - 0.05:
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
        item = self.items.get(self.cur_item)
        sec_label = _section_label(self.cur_side, self.cur_section)
        if item:
            item["done"].add(self.cur_skey)
            self.wp_done += 1
            self._pub_completed()
        self.get_logger().info(
            f"[{self.robot_name}] DONE {self.cur_skey} sec={sec_label} ({self.wp_done}/{self.wp_limit})")
        if self.wp_done >= self.wp_limit:
            self.get_logger().info(
                f"[{self.robot_name}] All done ({self.wp_done}/{self.wp_limit})")
            return
        self._continue_current_item_or_select_next()

    def _continue_current_item_or_select_next(self):
        if self._item_side_queue:
            entry = self._item_side_queue.pop(0)
            side = entry["side"]
            if side.get("section_points_template"):
                side = _prepare_section_line_side_for_pose(side, self.cur_x, self.cur_y)
                entry["side_key"] = side["key"]
            self._go(entry["item_name"], entry["side_key"], side)
            return
        self._select()

    # ── selection ─────────────────────────────────────────────────────────────

    def _select(self):
        self.state = IDLE
        if self._pending:
            p = self._pending.pop(0)
            iname, skey, side = p.get("item_name"), p.get("side_key"), p.get("side")
            if iname and skey and side:
                if side.get("section_points_template"):
                    side = _prepare_section_line_side_for_pose(side, self.cur_x, self.cur_y)
                self._item_side_queue = []
                self._go(iname, skey, side)
                return

        iname, side_queue = select_next(
            self.items, self.robot_name, self.cur_x, self.cur_y,
            self.owned, self.wp_done, self.wp_limit, self.priority_dirs, self.override)
        if iname is None:
            self.get_logger().info(
                f"[{self.robot_name}] All done ({self.wp_done}/{self.wp_limit})")
            return
        if side_queue and side_queue[0].get("section_points_template"):
            side_queue = [_prepare_section_line_side_for_pose(side, self.cur_x, self.cur_y) for side in side_queue]

        self._item_side_queue = [_make_side_entry(iname, side) for side in side_queue[1:]]
        first_side = side_queue[0]
        self._go(iname, first_side["key"], first_side)

    # ── pubs / callbacks ──────────────────────────────────────────────────────

    def _stop(self): self.cv_pub.publish(Twist())

    def _pub_arrived(self):
        s = self.cur_side
        ax, ay = s["approach_x"], s["approach_y"]
        if s.get("axis") == "section_line":
            pts = s.get("section_points", [])
            if 0 <= self.cur_section < len(pts):
                ax, ay = pts[self.cur_section][0], pts[self.cur_section][1]
        sec_label = _section_label(s, self.cur_section)
        msg = String()
        msg.data = json.dumps({
            "robot_id": self.robot_name, "shelf_id": self.cur_skey,
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
            self._stop()
            if self._gh:
                try: self._gh.cancel_goal_async()
                except: pass
                self._gh = None
            self.get_logger().warn(f"[{self.robot_name}] FAILED")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSender()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()