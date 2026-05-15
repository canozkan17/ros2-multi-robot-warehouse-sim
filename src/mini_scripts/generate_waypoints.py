#!/usr/bin/env python3
"""
generate_waypoints.py - Sprint 2 / Waypoint Revision (v3 - Ultimate Fix)
Reads shelf positions from the SDF file, calculates access points for each shelf,
and generates robot_assignments.json in a flat format compatible with waypoint_sender.py.

Layout Facts:
- Small Shelves (shelf/pallet) are HORIZONTALLY LONG (Long edge along X-axis).
  They are stacked vertically (varying Y coordinates).
  -> To approach the long edge, the robot must go to the TOP or BOTTOM (Y ± offset).
  
- Large Shelves (shelf_big) are VERTICALLY LONG (Long edge along Y-axis).
  They are arranged horizontally (varying X coordinates).
  -> To approach the long edge, the robot must go to the LEFT or RIGHT (X ± offset).
"""

import xml.etree.ElementTree as ET
import json

SDF_PATH   = "/home/canozkan/thesis_ws/src/warehouse_multi_robot/worlds/tugbot_warehouse_clean.sdf"
OUTPUT_PATH = "/home/canozkan/thesis_ws/src/warehouse_multi_robot/config/robot_assignments.json"

SINGLE_SIDE = {
    "shelf_big_1": "y-",   # Top-right corner, upper wall close -> approach from front (y-)
    "pallet_box_0": "x-",  # Bottom-right corner, right wall close -> approach from left (x-)
}

OFFSETS = {
    "shelf":     3.0,   
    "shelf_big": 3.2,   
    "pallet":    3.0,   
}

def get_type(uri: str, name: str) -> str | None:
    combined = (uri + " " + name).lower()
    if "shelf_big" in combined:
        return "shelf_big"
    if "pallet" in combined:
        return "pallet"
    if "shelf" in combined:
        return "shelf"
    return None

def make_waypoints(name: str, x: float, y: float, shelf_type: str) -> list[dict]:
    offset = OFFSETS[shelf_type]
    waypoints = []

    if shelf_type == "shelf" or shelf_type == "pallet":
        # Long edge along X -> Access from Top(Y+) or Bottom(Y-)
        if name in SINGLE_SIDE:
            side = SINGLE_SIDE[name]
            # Y+ means top, looking down -> yaw = -90 (qz=-0.7071)
            # Y- means bottom, looking up -> yaw = +90 (qz=0.7071)
            if "y" in side:
                # Single-sided from the Y-axis (standard for shelves)
                dy = offset if "+" in side else -offset
                qz = -0.7071 if "+" in side else 0.7071
                qw = 0.7071
                waypoints.append({
                    "shelf_id": f"{name}_side",
                    "x": round(x, 3),
                    "y": round(y + dy, 3),
                    "qz": qz, "qw": qw
                })
            else:
                # Unilateral from the X-axis (corner cases such as pallet_box_0)
                dx = offset if "+" in side else -offset
                qz = 1.0 if "+" in side else 0.0
                qw = 0.0 if "+" in side else 1.0
                waypoints.append({
                    "shelf_id": f"{name}_side",
                    "x": round(x + dx, 3),
                    "y": round(y, 3),
                    "qz": qz, "qw": qw
                })
        else:
            waypoints.append({
                "shelf_id": f"{name}_front", # Bottom approach
                "x": round(x, 3),
                "y": round(y - offset, 3),
                "qz": 0.7071, "qw": 0.7071 # Face UP
            })
            waypoints.append({
                "shelf_id": f"{name}_back",  # Top approach
                "x": round(x, 3),
                "y": round(y + offset, 3),
                "qz": -0.7071, "qw": 0.7071 # Face DOWN
            })
    elif shelf_type == "shelf_big":
        # Long edge along Y -> Access from Left(X-) or Right(X+)
        if name in SINGLE_SIDE:
            side = SINGLE_SIDE[name]
            if "x" in side:
                # X+ means right, looking left -> yaw = 180 (qz=1.0)
                # X- means left, looking right -> yaw = 0 (qz=0.0)
                dx = offset if "+" in side else -offset
                qz = 1.0 if "+" in side else 0.0
                qw = 0.0 if "+" in side else 1.0
                waypoints.append({
                    "shelf_id": f"{name}_side",
                    "x": round(x + dx, 3),
                    "y": round(y, 3),
                    "qz": qz, "qw": qw
                })
            else:
                # Y ekseninden tek taraflı (shelf_big_1 gibi üst duvara bitişik)
                dy = offset if "+" in side else -offset
                qz = -0.7071 if "+" in side else 0.7071
                qw = 0.7071
                waypoints.append({
                    "shelf_id": f"{name}_side",
                    "x": round(x, 3),
                    "y": round(y + dy, 3),
                    "qz": qz, "qw": qw
                })
        else:
            waypoints.append({
                "shelf_id": f"{name}_left", # Left approach
                "x": round(x - offset, 3),
                "y": round(y, 3),
                "qz": 0.0, "qw": 1.0 # Face RIGHT
            })
            waypoints.append({
                "shelf_id": f"{name}_right", # Right approach
                "x": round(x + offset, 3),
                "y": round(y, 3),
                "qz": 1.0, "qw": 0.0 # Face LEFT
            })
        
    return waypoints

def main():
    tree = ET.parse(SDF_PATH)
    root = tree.getroot()
    all_waypoints = []

    for inc in root.find('world').findall('include'):
        name_el = inc.find('name')
        uri_el  = inc.find('uri')
        pose_el = inc.find('pose')

        if name_el is None or uri_el is None or pose_el is None:
            continue

        name = name_el.text.strip()
        uri  = uri_el.text.strip()
        n    = name.lower()

        if 'shelf' not in n and 'pallet' not in n:
            continue

        vals = pose_el.text.strip().split()
        x, y = float(vals[0]), float(vals[1])

        shelf_type = get_type(uri, name)
        if wps := make_waypoints(name, x, y, shelf_type):
            all_waypoints.extend(wps)
            print(f"{name:25s} ({shelf_type:9s}) x={x:8.3f} y={y:8.3f} -> {len(wps)} wp")

    print(f"\nTotal waypoints: {len(all_waypoints)}")

    robot_waypoints = {"robot1": [], "robot2": [], "robot3": []}
    for i, wp in enumerate(all_waypoints):
        robot_id = f"robot{(i % 3) + 1}"
        robot_waypoints[robot_id].append(wp)

    for robot, wps in robot_waypoints.items():
        print(f"  {robot}: {len(wps)} waypoint")

    with open(OUTPUT_PATH, 'w') as f:
        json.dump(robot_waypoints, f, indent=2)

    print(f"\nSaved: {OUTPUT_PATH}")

if __name__ == '__main__':
    main()
