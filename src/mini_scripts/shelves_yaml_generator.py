#!/usr/bin/env python3
import xml.etree.ElementTree as ET
import yaml
import os
import sys

# SDF file path
sdf_path = "/home/canozkan/.gz/fuel/fuel.gazebosim.org/openrobotics/worlds/tugbot in warehouse/2/tugbot_warehouse.sdf"

if not os.path.exists(sdf_path):
    print(f"SDF file not found: {sdf_path}")
    sys.exit(1)

tree = ET.parse(sdf_path)
root = tree.getroot()

def parse_pose_text(pose_text):
    parts = pose_text.strip().split()
    parts += ["0"] * (6 - len(parts))
    try:
        nums = [float(p) for p in parts[:6]]
    except ValueError:
        return None
    return nums

shelves = []

# Parse <include> elements
for include in root.findall('.//include'):
    name_elem = include.find('name')
    if name_elem is None or not name_elem.text:
        continue
    name = name_elem.text
    if 'shelf' in name.lower() or 'pallet' in name.lower():
        pose_elem = include.find('pose')
        if pose_elem is None or not pose_elem.text:
            pose_elem = include.find('.//pose')
        if pose_elem is None or not pose_elem.text:
            continue
        pose = parse_pose_text(pose_elem.text)
        if pose is None:
            continue
        if any(s['id'] == name for s in shelves):
            continue
        shelves.append({
                            'id': name,
                            'position': {'x': pose[0], 'y': pose[1], 'z': pose[2]},
                            'orientation': {'roll': pose[3], 'pitch': pose[4], 'yaw': pose[5]}
                        })

# Save file
file_name = "shelf_waypoints.yaml"
with open(file_name, "w") as f:
    yaml.safe_dump({"shelves": shelves}, f, sort_keys=False, allow_unicode=True)

print(f"{file_name} saved. Total shelves: {len(shelves)}")
