#!/usr/bin/env python3

"""
generate_waypoints.py

TRUE REGION-AWARE WAREHOUSE PLANNER
-----------------------------------

Architecture
------------
1. Parse SDF
2. Generate object waypoints
3. Build Cartesian regions
4. Sort regions by traversal direction
5. Robot1 north-progressive traversal
6. Robot2 nearest traversal
7. Robot3 remaining traversal
8. Export waypoint_sender.py compatible JSON

- With default parameters (robot_radius=0.27, inflation=0.35, safety=0.02):
python3 generate_waypoints.py

- With custom parameters:
python3 generate_waypoints.py --robot-radius 0.27 --inflation 0.35 --safety 0.10

- Geometry only, without a map (legacy behavior):
python3 generate_waypoints.py --no-snap
"""

import argparse
import ast
import xml.etree.ElementTree as ET
import json
import math
import os
import sys
from collections import defaultdict, deque

try:
    import numpy as np
except ImportError:
    print("ERROR: numpy required. pip install numpy --break-system-packages")
    sys.exit(1)


# PATHS
SDF_PATH = "/home/canozkan/thesis_ws/src/warehouse_multi_robot/worlds/tugbot_warehouse_clean.sdf"
OUTPUT_PATH = "/home/canozkan/thesis_ws/src/warehouse_multi_robot/config/robot_assignments.json"

# ──────────────────────────────────────────────────────────────────────
# MAP LOADING & CLEARANCE
# ──────────────────────────────────────────────────────────────────────

def _load_map_yaml(path):
    data = {}
    with open(path, "r", encoding="utf-8") as f:
        lines = f.readlines()
    i = 0
    while i < len(lines):
        line = lines[i].split("#", 1)[0].rstrip("\n")
        stripped = line.strip()
        if not stripped or ":" not in stripped:
            i += 1
            continue
        key, val = stripped.split(":", 1)
        key, val = key.strip(), val.strip()
        if val == "":
            values = []
            i += 1
            while i < len(lines):
                nl = lines[i].split("#", 1)[0].rstrip("\n")
                if not nl.strip():
                    i += 1
                    continue
                if not nl.startswith(" ") and not nl.startswith("\t"):
                    break
                item = nl.strip()
                if item.startswith("-"):
                    values.append(item[1:].strip())
                    i += 1
                    continue
                break
            data[key] = values
            continue
        data[key] = val
        i += 1

    def _parse(v):
        if isinstance(v, list):
            return [_parse(x) for x in v]
        if isinstance(v, str) and v.startswith("[") and v.endswith("]"):
            return ast.literal_eval(v)
        try:
            return float(v) if ("." in v or "e" in v.lower()) else int(v)
        except Exception:
            return v

    return {k: _parse(v) for k, v in data.items()}


def _read_pgm(path):
    with open(path, "rb") as f:
        magic = f.readline().strip()
        if magic not in (b"P5", b"P2"):
            raise ValueError("Unsupported PGM")
        tokens = []
        while len(tokens) < 3:
            line = f.readline()
            if not line:
                break
            if line.startswith(b"#"):
                continue
            tokens.extend(line.split())
        width, height, maxval = map(int, tokens[:3])
        if magic == b"P5":
            data = f.read(width * height)
            img = np.frombuffer(data, dtype=np.uint8).reshape((height, width))
        else:
            raw = f.read().split()
            img = np.array(list(map(int, raw[:width * height])), dtype=np.uint8).reshape((height, width))
    return img, maxval


def _load_image(path):
    try:
        from PIL import Image
        return np.array(Image.open(path)), 255
    except Exception:
        return _read_pgm(path)


def load_map(map_yaml_path):
    """Returns (dist_cells, origin, res, img_height)"""
    yml = _load_map_yaml(map_yaml_path)
    img_path = yml.get("image")
    if not img_path:
        raise RuntimeError("map yaml missing 'image'")
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(map_yaml_path), img_path)

    res         = float(yml.get("resolution", 0.05))
    origin_raw  = yml.get("origin", [0.0, 0.0, 0.0])
    origin      = [float(x) for x in origin_raw]
    occ_thresh  = float(yml.get("occupied_thresh", 0.65))
    free_thresh = float(yml.get("free_thresh", 0.196))
    negate      = int(yml.get("negate", 0))

    img, maxval = _load_image(img_path)
    if negate == 0:
        prob = (maxval - img.astype(np.float32)) / float(maxval)
    else:
        prob = img.astype(np.float32) / float(maxval)

    occ = prob > occ_thresh

    # Euclidean distance transform (cells)
    try:
        from scipy.ndimage import distance_transform_edt
        dist_cells = distance_transform_edt(~occ)
    except ImportError:
        # BFS fallback
        h, w = occ.shape
        dist_cells = np.full((h, w), np.inf, dtype=np.float32)
        q = deque()
        ys, xs = np.where(occ)
        for y, x in zip(ys, xs):
            dist_cells[y, x] = 0.0
            q.append((y, x))
        nbrs = [(1,0),(-1,0),(0,1),(0,-1)]
        while q:
            cy, cx = q.popleft()
            base = dist_cells[cy, cx] + 1.0
            for dy, dx in nbrs:
                ny, nx = cy + dy, cx + dx
                if 0 <= ny < h and 0 <= nx < w and dist_cells[ny, nx] > base:
                    dist_cells[ny, nx] = base
                    q.append((ny, nx))

    return dist_cells, origin, res, img.shape[0]


def _world_to_grid(x, y, origin, res, height):
    gx = int(math.floor((x - origin[0]) / res))
    gy = int(math.floor((y - origin[1]) / res))
    iy = (height - 1) - gy
    return gx, iy

# Approach direction vectors (move AWAY from shelf to clear obstacles)
_APPROACH_DIR = {
    'yminus': (0, -1),   # approaching from -y -> push further -y
    'yplus':  (0,  1),
    'xminus': (-1, 0),
    'xplus':  ( 1, 0),
}


def snap_waypoint(wp, dist_cells, origin, res, height, min_clear_m, max_shift_m=3.0):
    """
    If waypoint clearance < min_clear_m, shift it along its approach
    direction (away from shelf) in res-sized steps until clearance is met.
    Includes lateral orthogonal tolerance to clear side walls/obstacles.
    """
    side = wp.get("side", "")
    ddx, ddy = _APPROACH_DIR.get(side, (0, 0))
    max_steps = int(math.ceil(max_shift_m / res))
    h, w = dist_cells.shape

    # 1. Determine the lateral direction orthogonal (perpendicular) to the approach direction
    if ddx == 0:  # ​​Approaching along the Y direction (yminus/yplus) -> Lateral shift is along the X axis
        orth_dx, orth_dy = 1.0, 0.0
    else:         # Approaching along the X direction (xminus/xplus) -> Lateral shift is along the Y axis
        orth_dx, orth_dy = 0.0, 1.0

    # 2. List of lateral tolerances (in meters)
    # First, try the exact center (0.0); if that fails, try shifting 5cm, 10cm, 15cm, and 20cm to the right and left, respectively.
    lateral_offsets = [0.0, -0.05, 0.05, -0.10, 0.10, -0.15, 0.15, -0.20, 0.20]

    for lat_offset in lateral_offsets:
        # Apply the lateral tolerance to the starting point (e.g., if a wall is on the right, we will shift to the left)
        start_x = wp["x"] + orth_dx * lat_offset
        start_y = wp["y"] + orth_dy * lat_offset

        x, y = start_x, start_y
        for step in range(max_steps + 1):
            gx, iy = _world_to_grid(x, y, origin, res, height)
            if 0 <= gx < w and 0 <= iy < h:
                # Check the safety of the map cell
                if dist_cells[iy, gx] * res >= min_clear_m:
                    shifted = step * res
                    # Safe point. Return the coordinates, shifted laterally as well. 
                    return round(x, 3), round(y, 3), shifted

            # Advance in the direction of approach (along the line of retreat)
            x += ddx * res
            y += ddy * res

    # If no lateral combination or vertical approach yielded a solution, return the original coordinates (signaling FAILED)
    return round(wp["x"], 3), round(wp["y"], 3), -1.0


def snap_all_waypoints(assignments, dist_cells, origin, res, height, min_clear_m):
    """
    Post-processes all waypoints in assignments dict in-place.
    Prints a summary of how many were shifted and worst cases.
    """
    total = 0
    shifted = 0
    failed = 0
    worst = []

    for robot, wps in assignments.items():
        for wp in wps:
            total += 1
            nx, ny, delta = snap_waypoint(wp, dist_cells, origin, res, height, min_clear_m)
            if delta < 0:
                failed += 1
                worst.append((robot, wp.get("shelf_id", "?"), wp["x"], wp["y"], delta))
            elif delta > 0:
                shifted += 1
                wp["x"] = nx
                wp["y"] = ny

    print(f"\n{'='*60}")
    print(f"CLEARANCE SNAP SUMMARY  (min_clear={min_clear_m:.3f}m)")
    print(f"  Total waypoints : {total}")
    print(f"  Shifted         : {shifted}")
    print(f"  FAILED (no room): {failed}")
    if failed > 0:
        print("  Failed waypoints (may need manual fix or larger APPROACH_DISTANCE):")
        for r, sid, x, y, _ in worst:
            print(f"    {r} | {sid} | x={x:.3f} y={y:.3f}")
    print(f"{'='*60}")

# CONFIG
APPROACH_DISTANCE = 1.2

SMALL_SHELF_WIDTH = 3.6
SMALL_SHELF_SECTIONS = ["A", "B", "C", "D"]
SMALL_SHELF_STEP = SMALL_SHELF_WIDTH / len(SMALL_SHELF_SECTIONS)
SMALL_SHELF_START = -SMALL_SHELF_WIDTH / 2 + SMALL_SHELF_STEP / 2
SHELF_SECS_Y = [
    (sec, SMALL_SHELF_START + idx * SMALL_SHELF_STEP)
    for idx, sec in enumerate(SMALL_SHELF_SECTIONS)
]

SHELF_BIG_SECS_X = [
    ('A', -6.75),
    ('B', -2.25),
    ('C',  2.25),
    ('D',  6.75),
]

Q = {
    'xminus': (0.0,    1.0),
    'xplus':  (1.0,    0.0),
    'yminus': (0.7071, 0.7071),
    'yplus':  (-0.7071,0.7071),
}

SPECIAL_SINGLE_SIDE = {
    "shelf_big_1": ["xminus"],
}

SPECIAL_PALLETS = {
    "pallet_box_0": ["xminus", "yplus"],
}

SPECIAL_PALLET_SIDE_ORDER = {
    "pallet_box_mobile": ["xplus", "xminus"],
    "pallet_box_mobile_0": ["xplus", "xminus"],
    "pallet_box_mobile_1": ["xplus", "xminus"],
}

MOBILE_PALLET_GROUP = [
    "pallet_box_mobile",
    "pallet_box_mobile_0",
    "pallet_box_mobile_1",
]
MOBILE_PALLET_GROUP_SET = set(MOBILE_PALLET_GROUP)


# HELPERS
def dist_xy(x1, y1, x2, y2):
    return math.hypot(x1 - x2, y1 - y2)

def region_of(x, y):
    if x >= 0 and y >= 0:
        return 1
    
    if x < 0 and y >= 0:
        return 2

    if x < 0 and y < 0:
        return 3

    return 4

def get_type(name):

    n = name.lower()
    if "shelf_big" in n:
        return "shelf_big"

    if "pallet" in n:
        return "pallet"

    if "shelf" in n:
        return "shelf"

    return None


# WAYPOINT GENERATION
def make_small_shelf(name, x, y):
    wps = []

    # south side approach (yminus)
    for sec, dx in SHELF_SECS_Y:
        wps.append({
            "shelf_id": f"{name}_yminus_{sec}",
            "group": name,
            "side": "yminus",
            "section": sec,
            "x": round(x + dx, 3),
            "y": round(y - APPROACH_DISTANCE, 3),
            "qz": Q["yminus"][0],
            "qw": Q["yminus"][1],
        })

    # north side approach (yplus)
    for sec, dx in reversed(SHELF_SECS_Y):
        wps.append({
            "shelf_id": f"{name}_yplus_{sec}",
            "group": name,
            "side": "yplus",
            "section": sec,
            "x": round(x + dx, 3),
            "y": round(y + APPROACH_DISTANCE, 3),
            "qz": Q["yplus"][0],
            "qw": Q["yplus"][1],
        })

    return wps

def make_big_shelf(name, x, y):

    wps = []
    sides = ["xminus", "xplus"]

    if name in SPECIAL_SINGLE_SIDE:
        sides = SPECIAL_SINGLE_SIDE[name]

    for side in sides:
        sections = SHELF_BIG_SECS_X

        if side == "xplus":
            sections = reversed(SHELF_BIG_SECS_X)

        for sec, dy in sections:
            dx = -APPROACH_DISTANCE if side == "xminus" else APPROACH_DISTANCE
            wps.append({
                "shelf_id": f"{name}_{side}_{sec}",
                "group": name,
                "side": side,
                "section": sec,
                "x": round(x + dx, 3),
                "y": round(y + dy, 3),
                "qz": Q[side][0],
                "qw": Q[side][1],
            })

    return wps

def make_pallet(name, x, y):

    wps = []
    if name in SPECIAL_PALLETS:
        for side in SPECIAL_PALLETS[name]:
            if side == "xminus":
                wps.append({
                    "shelf_id": f"{name}_{side}",
                    "group": name,
                    "side": side,
                    "section": "A",
                    "x": round(x - APPROACH_DISTANCE, 3),
                    "y": round(y, 3),
                    "qz": Q[side][0],
                    "qw": Q[side][1],
                })

            elif side == "xplus":
                wps.append({
                    "shelf_id": f"{name}_{side}",
                    "group": name,
                    "side": side,
                    "section": "A",
                    "x": round(x + APPROACH_DISTANCE, 3),
                    "y": round(y, 3),
                    "qz": Q[side][0],
                    "qw": Q[side][1],
                })

            else:
                wps.append({
                    "shelf_id": f"{name}_{side}",
                    "group": name,
                    "side": side,
                    "section": "A",
                    "x": round(x, 3),
                    "y": round(y + (-APPROACH_DISTANCE if side == 'yminus' else APPROACH_DISTANCE), 3),
                    "qz": Q[side][0],
                    "qw": Q[side][1],
                })

        return wps

    for side in ["xminus", "xplus"]:
        dx = -APPROACH_DISTANCE if side == "xminus" else APPROACH_DISTANCE
        wps.append({
            "shelf_id": f"{name}_{side}",
            "group": name,
            "side": side,
            "section": "A",
            "x": round(x + dx, 3),
            "y": round(y, 3),
            "qz": Q[side][0],
            "qw": Q[side][1],
        })

    return wps


# PARSE SDF
def parse_sdf():

    tree = ET.parse(SDF_PATH)
    root = tree.getroot()
    grouped = {}
    world = root.find("world")

    for inc in world.findall("include"):
        name_el = inc.find("name")
        pose_el = inc.find("pose")

        if name_el is None or pose_el is None:
            continue

        name = name_el.text.strip()
        t = get_type(name)

        if not t:
            continue

        vals = pose_el.text.strip().split()

        x = float(vals[0])
        y = float(vals[1])

        if t == "shelf":
            wps = make_small_shelf(name, x, y)

        elif t == "shelf_big":
            wps = make_big_shelf(name, x, y)

        else:
            wps = make_pallet(name, x, y)

        grouped[name] = {
            "name": name,
            "type": t,
            "x": x,
            "y": y,
            "region": region_of(x, y),
            "wps": wps
        }

    return grouped


# BUILD SORTED REGIONS
def build_regions(grouped):

    regions = {
        1: [],
        2: [],
        3: [],
        4: [],
    }

    for obj in grouped.values():
        regions[obj["region"]].append(obj)

    # north -> south sorting
    for r in regions:
        regions[r].sort(
            key=lambda o: o["y"],
            reverse=True
        )

    return regions


# OBJECT HELPERS
def nearest_wp_distance(current_x, current_y, wps):
    return min(
        dist_xy(current_x, current_y, wp["x"], wp["y"])
        for wp in wps
    )

def side_axis(side):
    if side.startswith("y"):
        return "x"
    if side.startswith("x"):
        return "y"
    return "x"

def order_wps_from_nearest_end(current_x, current_y, wps):
    if not wps:
        return [], current_x, current_y

    axis = side_axis(wps[0]["side"])
    min_wp = min(wps, key=lambda wp: wp[axis])
    max_wp = max(wps, key=lambda wp: wp[axis])

    dist_min = dist_xy(current_x, current_y, min_wp["x"], min_wp["y"])
    dist_max = dist_xy(current_x, current_y, max_wp["x"], max_wp["y"])

    ordered = sorted(
        wps,
        key=lambda wp: wp[axis],
        reverse=dist_max < dist_min
    )
    last = ordered[-1]
    return ordered, last["x"], last["y"]

def order_object_wps(
    current_x,
    current_y,
    obj_wps,
    forced_side=None,
    side_order=None
):
    sides = defaultdict(list)
    for wp in obj_wps:
        sides[wp["side"]].append(wp)

    ordered = []

    def take_side(side):
        nonlocal current_x, current_y
        side_wps = sides.pop(side)
        side_ordered, current_x, current_y = order_wps_from_nearest_end(
            current_x,
            current_y,
            side_wps
        )
        ordered.extend(side_ordered)

    if forced_side and forced_side in sides:
        take_side(forced_side)

    if side_order:
        for side in side_order:
            if side in sides:
                take_side(side)

    # Finish all sections on one side before switching sides.
    while sides:
        side = min(
            sides.keys(),
            key=lambda s: nearest_wp_distance(current_x, current_y, sides[s])
        )
        take_side(side)

    return ordered, current_x, current_y

def nearest_object(current_x, current_y, objects, prefer_north=None):
    candidates = [o for o in objects if o["wps"]]

    if prefer_north is True:
        candidates = [
            o for o in candidates
            if o["y"] >= current_y - 0.5
        ]
    elif prefer_north is False:
        candidates = [
            o for o in candidates
            if o["y"] <= current_y + 0.5
        ]

    if not candidates:
        return None

    return min(
        candidates,
        key=lambda o: nearest_wp_distance(current_x, current_y, o["wps"])
    )

def take_object(robot_list, obj, current_x, current_y, limit, forced_side=None):
    remaining_slots = limit - len(robot_list)
    if remaining_slots <= 0:
        return current_x, current_y, False

    side_order = SPECIAL_PALLET_SIDE_ORDER.get(obj["name"])
    ordered, end_x, end_y = order_object_wps(
        current_x,
        current_y,
        obj["wps"],
        forced_side,
        side_order
    )

    if len(ordered) <= remaining_slots:
        robot_list.extend(ordered)
        obj["wps"] = []
        return end_x, end_y, True

    robot_list.extend(ordered[:remaining_slots])
    taken_ids = {wp["shelf_id"] for wp in ordered[:remaining_slots]}
    obj["wps"] = [
        wp for wp in obj["wps"]
        if wp["shelf_id"] not in taken_ids
    ]
    last = robot_list[-1]
    return last["x"], last["y"], False

def take_mobile_pallet_group(robot_list, group_objects, current_x, current_y, limit):
    if not group_objects:
        return current_x, current_y, set(), False

    ordered = sorted(group_objects, key=lambda o: o["y"], reverse=True)
    sequence = []

    for obj in ordered:
        sequence.extend([wp for wp in obj["wps"] if wp["side"] == "xplus"])

    for obj in reversed(ordered):
        sequence.extend([wp for wp in obj["wps"] if wp["side"] == "xminus"])

    if not sequence:
        return current_x, current_y, set(), False

    count_before = len(robot_list)
    taken_ids = set()

    for wp in sequence:
        if len(robot_list) >= limit:
            break
        robot_list.append(wp)
        current_x, current_y = wp["x"], wp["y"]
        taken_ids.add(wp["shelf_id"])

    completed = set()
    if taken_ids:
        for obj in group_objects:
            obj["wps"] = [
                wp for wp in obj["wps"]
                if wp["shelf_id"] not in taken_ids
            ]
            if not obj["wps"]:
                completed.add(obj["name"])

    progressed = len(robot_list) > count_before
    return current_x, current_y, completed, progressed


# ROBOT1
def allocate_robot1(regions, grouped):

    robot = []
    shelf2 = grouped["shelf_2"]
    start_wp = next(
        wp for wp in shelf2["wps"]
        if wp["side"] == "yplus" and wp["section"] == "D"
    )
    current_x = start_wp["x"]
    current_y = start_wp["y"]

    yplus_wps = [
        wp for wp in shelf2["wps"]
        if wp["side"] == "yplus"
    ]
    ordered, current_x, current_y = order_object_wps(
        current_x,
        current_y,
        yplus_wps,
        forced_side="yplus"
    )
    robot.extend(ordered)

    shelf2["wps"] = [
        wp for wp in shelf2["wps"]
        if wp["side"] == "yminus"
    ]

    used = {"shelf_2"}

    
    # REGION 2
    region2 = [
        o for o in regions[2]
        if o["name"] not in used and o["y"] > 0 and o["wps"]
    ]

    while region2 and len(robot) < 48:
        target = nearest_object(
            current_x,
            current_y,
            region2,
            prefer_north=True
        )

        if target is None:
            break

        if target["name"] in MOBILE_PALLET_GROUP_SET:
            group_objs = [
                o for o in region2
                if o["name"] in MOBILE_PALLET_GROUP_SET
            ]
            current_x, current_y, completed, progressed = take_mobile_pallet_group(
                robot,
                group_objs,
                current_x,
                current_y,
                48
            )
            if completed:
                used.update(completed)
                region2 = [
                    o for o in region2
                    if o["name"] not in completed
                ]
            if not progressed or len(robot) >= 48:
                break
            continue

        current_x, current_y, completed = take_object(
            robot,
            target,
            current_x,
            current_y,
            48
        )

        if completed:
            used.add(target["name"])
            region2.remove(target)
        else:
            break

    
    # REGION 1
    region1 = [
        o for o in regions[1]
        if o["name"] not in used and o["wps"]
    ]

    while region1 and len(robot) < 48:
        target = max(region1, key=lambda o: o["y"])
        if target["name"] in MOBILE_PALLET_GROUP_SET:
            group_objs = [
                o for o in region1
                if o["name"] in MOBILE_PALLET_GROUP_SET
            ]
            current_x, current_y, completed, progressed = take_mobile_pallet_group(
                robot,
                group_objs,
                current_x,
                current_y,
                48
            )
            if completed:
                used.update(completed)
                region1 = [
                    o for o in region1
                    if o["name"] not in completed
                ]
            if not progressed or len(robot) >= 48:
                break
            continue

        current_x, current_y, completed = take_object(
            robot,
            target,
            current_x,
            current_y,
            48
        )

        if completed:
            used.add(target["name"])
            region1.remove(target)
        else:
            break

    
    # REGION 4
    region4 = [
        o for o in regions[4]
        if o["name"] not in used and o["wps"]
    ]

    while region4 and len(robot) < 48:
        target = max(region4, key=lambda o: o["y"])
        if target["name"] in MOBILE_PALLET_GROUP_SET:
            group_objs = [
                o for o in region4
                if o["name"] in MOBILE_PALLET_GROUP_SET
            ]
            current_x, current_y, completed, progressed = take_mobile_pallet_group(
                robot,
                group_objs,
                current_x,
                current_y,
                48
            )
            if completed:
                used.update(completed)
                region4 = [
                    o for o in region4
                    if o["name"] not in completed
                ]
            if not progressed or len(robot) >= 48:
                break
            continue

        current_x, current_y, completed = take_object(
            robot,
            target,
            current_x,
            current_y,
            48
        )

        if completed:
            used.add(target["name"])
            region4.remove(target)
        else:
            break

    return robot, used


# ROBOT2
def allocate_robot2(regions, grouped, global_used):

    robot = []
    shelf10 = grouped["shelf_10"]
    start_wp = next(
        wp for wp in shelf10["wps"]
        if wp["side"] == "yplus" and wp["section"] == "D"
    )
    current_x = start_wp["x"]
    current_y = start_wp["y"]

    current_x, current_y, completed = take_object(
        robot,
        shelf10,
        current_x,
        current_y,
        47,
        forced_side="yplus"
    )

    used = set()
    if completed:
        used.add("shelf_10")

    
    # REGION 4
    region4 = [
        o for o in regions[4]
        if o["name"] not in global_used
        and o["name"] not in used
        and o["wps"]
    ]

    while region4 and len(robot) < 47:
        target = nearest_object(
            current_x,
            current_y,
            region4
        )

        if target is None:
            break

        if target["name"] in MOBILE_PALLET_GROUP_SET:
            group_objs = [
                o for o in region4
                if o["name"] in MOBILE_PALLET_GROUP_SET
            ]
            current_x, current_y, completed, progressed = take_mobile_pallet_group(
                robot,
                group_objs,
                current_x,
                current_y,
                47
            )
            if completed:
                used.update(completed)
                region4 = [
                    o for o in region4
                    if o["name"] not in completed
                ]
            if not progressed or len(robot) >= 47:
                break
            continue

        current_x, current_y, completed = take_object(
            robot,
            target,
            current_x,
            current_y,
            47
        )

        if completed:
            used.add(target["name"])
            region4.remove(target)
        else:
            break

    
    # REGION 3 SPILL
    region3 = [
        o for o in regions[3]
        if o["name"] not in global_used
        and o["name"] not in used
        and o["wps"]
    ]

    while region3 and len(robot) < 47:
        target = nearest_object(
            current_x,
            current_y,
            region3
        )

        if target is None:
            break

        if target["name"] in MOBILE_PALLET_GROUP_SET:
            group_objs = [
                o for o in region3
                if o["name"] in MOBILE_PALLET_GROUP_SET
            ]
            current_x, current_y, completed, progressed = take_mobile_pallet_group(
                robot,
                group_objs,
                current_x,
                current_y,
                47
            )
            if completed:
                used.update(completed)
                region3 = [
                    o for o in region3
                    if o["name"] not in completed
                ]
            if not progressed or len(robot) >= 47:
                break
            continue

        current_x, current_y, completed = take_object(
            robot,
            target,
            current_x,
            current_y,
            47
        )

        if completed:
            used.add(target["name"])
            region3.remove(target)
        else:
            break

    return robot, used


# ROBOT3
def allocate_robot3(grouped, used1, used2):

    robot = []
    shelf2 = grouped["shelf_2"]
    start_wp = next(
        wp for wp in shelf2["wps"]
        if wp["side"] == "yminus" and wp["section"] == "D"
    )
    current_x = start_wp["x"]
    current_y = start_wp["y"]

    yminus_wps = [
        wp for wp in shelf2["wps"]
        if wp["side"] == "yminus"
    ]
    ordered, current_x, current_y = order_object_wps(
        current_x,
        current_y,
        yminus_wps,
        forced_side="yminus"
    )
    robot.extend(ordered)
    shelf2["wps"] = []

    remaining = [
        obj for obj in grouped.values()
        if obj["name"] not in used1
        and obj["name"] not in used2
        and obj["name"] != "shelf_2"
        and obj["wps"]
    ]

    while remaining and len(robot) < 47:
        target = nearest_object(
            current_x,
            current_y,
            remaining
        )

        if target is None:
            break

        if target["name"] in MOBILE_PALLET_GROUP_SET:
            group_objs = [
                o for o in remaining
                if o["name"] in MOBILE_PALLET_GROUP_SET
            ]
            current_x, current_y, completed, progressed = take_mobile_pallet_group(
                robot,
                group_objs,
                current_x,
                current_y,
                47
            )
            if completed:
                remaining = [
                    o for o in remaining
                    if o["name"] not in completed
                ]
            if not progressed or len(robot) >= 47:
                break
            continue

        current_x, current_y, completed = take_object(
            robot,
            target,
            current_x,
            current_y,
            47
        )

        if completed:
            remaining.remove(target)
        else:
            break

    return robot


# VALIDATION
def validate(assignments):

    total = sum(len(v) for v in assignments.values())

    if total != 142:
        raise RuntimeError(f"INVALID TOTAL: {total}")

    if len(assignments["robot1"]) != 48:
        raise RuntimeError("robot1 invalid")

    if len(assignments["robot2"]) != 47:
        raise RuntimeError("robot2 invalid")

    if len(assignments["robot3"]) != 47:
        raise RuntimeError("robot3 invalid")


# PRINT SUMMARY
def print_summary(assignments):

    print("\n")
    print("=" * 80)
    print("WAYPOINT DISTRIBUTION")
    print("=" * 80)

    for robot, wps in assignments.items():
        print(f"\n{robot.upper()}")
        counter = defaultdict(int)

        for wp in wps:
            counter[wp["group"]] += 1

        total = 0

        for k, v in counter.items():
            total += v
            print(f"  - {k:25s} = {v:2d} wp")

        print("-" * 50)
        print(f"TOTAL = {total}")


def print_route_details(assignments):

    print("\n")
    print("=" * 80)
    print("WAYPOINT ROUTES")
    print("=" * 80)

    for robot, wps in assignments.items():
        print(f"\n{robot.upper()} ROUTE")
        for idx, wp in enumerate(wps, start=1):
            print(
                f"{idx:02d}. {wp['shelf_id']} "
                f"({wp['group']}, {wp['side']}, {wp['section']}) "
                f"x={wp['x']:.3f} y={wp['y']:.3f}"
            )


# CLEAN OUTPUT
def cleanup(assignments):
    cleaned = {}
    for robot, wps in assignments.items():
        cleaned[robot] = []
        for wp in wps:
            c = dict(wp)
            del c["group"]
            del c["side"]
            del c["section"]
            cleaned[robot].append(c)

    return cleaned


# ──────────────────────────────────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Warehouse waypoint generator with map-based clearance snap"
    )
    parser.add_argument(
        "--map-yaml",
        default=os.path.expanduser(
            "~/thesis_ws/src/warehouse_multi_robot/maps/warehouse_map.yaml"
        ),
        help="Path to Nav2 map YAML"
    )
    parser.add_argument(
        "--robot-radius", type=float, default=0.27,
        help="Robot inscribed radius in metres (default: 0.27)"
    )
    parser.add_argument(
        "--inflation", type=float, default=0.35,
        help="Nav2 inflation_radius in metres (default: 0.35)"
    )
    parser.add_argument(
        "--safety", type=float, default=0.02,
        help="Extra safety margin in metres (default: 0.02)"
    )
    parser.add_argument(
        "--no-snap", action="store_true",
        help="Skip map-based clearance snapping (dry-run geometry only)"
    )
    args = parser.parse_args()

    min_clear_m = args.inflation + args.safety
    print(f"\nMin required clearance: {args.inflation:.2f} + {args.safety:.2f} = {min_clear_m:.3f} m")

    print("\nParsing SDF...")
    grouped = parse_sdf()
    total = sum(len(v["wps"]) for v in grouped.values())
    print(f"Generated Waypoints: {total}")

    regions = build_regions(grouped)
    robot1, used1 = allocate_robot1(regions, grouped)
    robot2, used2 = allocate_robot2(regions, grouped, used1)
    robot3 = allocate_robot3(grouped, used1, used2)

    assignments = {
        "robot1": robot1,
        "robot2": robot2,
        "robot3": robot3,
    }
    validate(assignments)
    print_summary(assignments)
    print_route_details(assignments)

    if not args.no_snap:
        map_yaml = os.path.expanduser(args.map_yaml)
        if not os.path.isfile(map_yaml):
            print(f"\nWARN: map yaml not found at {map_yaml} — skipping snap.")
            print("      Re-run with --map-yaml <path> or --no-snap.")
        else:
            print(f"\nLoading map: {map_yaml}")
            dist_cells, origin, res, img_height = load_map(map_yaml)
            print(f"Map loaded: res={res}m, origin={origin[:2]}, shape={dist_cells.shape}")
            snap_all_waypoints(assignments, dist_cells, origin, res, img_height, min_clear_m)
    else:
        print("\n[--no-snap] Clearance check skipped.")

    cleaned = cleanup(assignments)
    with open(OUTPUT_PATH, "w") as f:
        json.dump(cleaned, f, indent=2)

    print(f"\nSaved: {OUTPUT_PATH}")
    print("\nDONE.")


if __name__ == "__main__":
    main()