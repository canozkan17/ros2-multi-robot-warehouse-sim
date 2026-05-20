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

Author
------
Can Ozkan Thesis Planner
"""

import xml.etree.ElementTree as ET
import json
import math
from collections import defaultdict


# PATHS
SDF_PATH = "/home/canozkan/thesis_ws/src/warehouse_multi_robot/worlds/tugbot_warehouse_clean.sdf"
OUTPUT_PATH = "/home/canozkan/thesis_ws/src/warehouse_multi_robot/config/robot_assignments.json"


# CONFIG
APPROACH_DISTANCE = 1.5

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


# MAIN
def main():

    print("\nParsing SDF...")
    grouped = parse_sdf()
    total = sum(len(v["wps"]) for v in grouped.values())
    print(f"Generated Waypoints: {total}")
    regions = build_regions(grouped)
    robot1, used1 = allocate_robot1(regions, grouped)
    robot2, used2 = allocate_robot2(
        regions,
        grouped,
        used1
    )
    robot3 = allocate_robot3(
        grouped,
        used1,
        used2
    )
    assignments = {
        "robot1": robot1,
        "robot2": robot2,
        "robot3": robot3,
    }
    validate(assignments)
    print_summary(assignments)
    print_route_details(assignments)
    cleaned = cleanup(assignments)
    with open(OUTPUT_PATH, "w") as f:
        json.dump(cleaned, f, indent=2)

    print("\nSaved:")
    print(OUTPUT_PATH)
    print("\nDONE.")



if __name__ == "__main__":
    main()