#!/usr/bin/env python3
"""
sdf_parser.py - SDF World Parser and Geometry Engine.

This module parses the warehouse world file, extracts item locations, corrects
geometric center offsets using transformation matrices, and calculates precise
global coordinates for section-level waypoints and robot approach poses.
"""

import math
import xml.etree.ElementTree as ET
from typing import Dict, List, Tuple, Any

from .config import (
    APPROACH_DISTANCE_M,
    SMALL_SHELF_SPECS,
    BIG_SHELF_SPECS,
    PALLET_BOX_SPECS,
    SINGLE_APPROACH_SIDES,
    SPECIAL_PALLET_APPROACHES,
    MOBILE_PALLET_SET,
    MOBILE_CLUSTER_NAME
)


def apply_transformation_matrix(
    raw_x: float, 
    raw_y: float, 
    yaw: float, 
    local_dx: float, 
    local_dy: float
) -> Tuple[float, float]:
    """
    Applies a 2D rotation matrix to map local offsets into global world coordinates.

    Formula:
        gx = raw_x + local_dx * cos(yaw) - local_dy * sin(yaw)
        gy = raw_y + local_dx * sin(yaw) + local_dy * cos(yaw)

    Args:
        raw_x: Model origin X coordinate in world.
        raw_y: Model origin Y coordinate in world.
        yaw: Orientation angle of the model (radyan).
        local_dx: Local offset along the model's longitudinal axis.
        local_dy: Local offset along the model's lateral axis.

    Returns:
        Transformed global coordinates (gx, gy) rounded to 3 decimal places.
    """
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    global_x = raw_x + (local_dx * cos_yaw) - (local_dy * sin_yaw)
    global_y = raw_y + (local_dx * sin_yaw) + (local_dy * cos_yaw)
    return round(global_x, 3), round(global_y, 3)


def get_item_geometric_center(
    item_type: str, 
    raw_x: float, 
    raw_y: float, 
    yaw: float
) -> Tuple[float, float]:
    """
    Calculates the true geometric center of an item by applying its local offset.

    Args:
        item_type: Category of the item ('shelf', 'shelf_big', etc.)
        raw_x: SDF origin X coordinate.
        raw_y: SDF origin Y coordinate.
        yaw: Model rotation (radyan).

    Returns:
        True physical center coordinates (cx, cy).
    """
    if item_type in ("shelf", "shelf_big"):
        # Shelves have -0.5m offset along local X in Gazebo
        local_dx = -0.5
        local_dy = 0.0
    else:
        local_dx = 0.0
        local_dy = 0.0
        
    return apply_transformation_matrix(raw_x, raw_y, yaw, local_dx, local_dy)


def classify_cartesian_region(x: float, y: float) -> int:
    """
    Classifies global 2D coordinates into one of four Cartesian regions.

    Args:
        x: World coordinate X.
        y: World coordinate Y.

    Returns:
        Region ID (1: Top-Right, 2: Top-Left, 3: Bottom-Left, 4: Bottom-Right).
    """
    if x >= 0.0 and y >= 0.0:
        return 1
    if x < 0.0 and y >= 0.0:
        return 2
    if x < 0.0 and y < 0.0:
        return 3
    return 4


def build_small_shelf_sides(name: str, cx: float, cy: float, yaw: float) -> Dict[str, Any]:
    """
    Generates sides and section waypoints for a standard small shelf.

    Small Shelf dimensions are 3.6m width (local X) by 0.6m depth (local Y).
    Approaches are calculated along local Y axis (yplus, yminus).
    Sections (A, B, C, D) are split along the local X axis.
    """
    sides = {}
    labels = ["A", "B", "C", "D"]
    half_width = SMALL_SHELF_SPECS["width"] / 2.0
    step = SMALL_SHELF_SPECS["section_step"]
    
    offset_dist = (SMALL_SHELF_SPECS["depth"] / 2.0) + APPROACH_DISTANCE_M # 1.5m
    
    # Coordinates of the two physical sides in the local +Y and -Y directions
    side_1_pts = []
    for i in range(len(labels)):
        local_dx = -half_width + (step / 2.0) + (i * step)
        gx, gy = apply_transformation_matrix(cx, cy, yaw, local_dx, offset_dist)
        side_1_pts.append((gx, gy))
        
    side_2_pts = []
    for i in range(len(labels)):
        local_dx = -half_width + (step / 2.0) + (i * step)
        gx, gy = apply_transformation_matrix(cx, cy, yaw, local_dx, -offset_dist)
        side_2_pts.append((gx, gy))
        
    # yplus and yminus sides based on the magnitude along the global Y-axis
    side_1_y_avg = sum(pt[1] for pt in side_1_pts) / len(side_1_pts)
    side_2_y_avg = sum(pt[1] for pt in side_2_pts) / len(side_2_pts)
    
    if side_1_y_avg > side_2_y_avg:
        yplus_raw, yminus_raw = side_1_pts, side_2_pts
    else:
        yplus_raw, yminus_raw = side_2_pts, side_1_pts
        
    # Arrange the sections for both sides on the global X-axis from left to right (ascending X).
    for side_name, raw_pts in [("yplus", yplus_raw), ("yminus", yminus_raw)]:
        raw_pts.sort(key=lambda pt: pt[0]) # X increasing order
        section_points_list = [(pt[0], pt[1], labels[idx]) for idx, pt in enumerate(raw_pts)]
        
        sides[side_name] = {
            "side": side_name,
            "key": f"{name}_{side_name}",
            "approach_x": section_points_list[0][0],
            "approach_y": section_points_list[0][1],
            "axis": "section_line",
            "sections": SMALL_SHELF_SPECS["sections"],
            "section_points_template": section_points_list,
            "section_labels_template": labels,
        }
    return sides


def build_big_shelf_sides(name: str, cx: float, cy: float, yaw: float) -> Dict[str, Any]:
    """
    Generates sides and section waypoints for a big shelf.

    Big Shelf dimensions are 2.1m width (local X) by 18.0m length (local Y).
    Approaches are calculated along local X axis (xplus, xminus).
    Sections (A, B, C, D) are split along local Y axis (top to bottom).
    """
    sides = {}
    labels = ["A", "B", "C", "D"]
    half_length = BIG_SHELF_SPECS["length"] / 2.0
    step = BIG_SHELF_SPECS["section_step"]
    
    offset_dist = (BIG_SHELF_SPECS["width"] / 2.0) + APPROACH_DISTANCE_M # 2.25m
    
    side_1_pts = []
    for i in range(len(labels)):
        local_dy = half_length - (step / 2.0) - (i * step)
        gx, gy = apply_transformation_matrix(cx, cy, yaw, offset_dist, local_dy)
        side_1_pts.append((gx, gy))
        
    side_2_pts = []
    for i in range(len(labels)):
        local_dy = half_length - (step / 2.0) - (i * step)
        gx, gy = apply_transformation_matrix(cx, cy, yaw, -offset_dist, local_dy)
        side_2_pts.append((gx, gy))
        
    side_1_x_avg = sum(pt[0] for pt in side_1_pts) / len(side_1_pts)
    side_2_x_avg = sum(pt[0] for pt in side_2_pts) / len(side_2_pts)
    
    if side_1_x_avg > side_2_x_avg:
        xplus_raw, xminus_raw = side_1_pts, side_2_pts
    else:
        xplus_raw, xminus_raw = side_2_pts, side_1_pts
        
    target_sides = SINGLE_APPROACH_SIDES.get(name, ["xminus", "xplus"])
    for side_name in target_sides:
        raw_pts = xplus_raw if side_name == "xplus" else xminus_raw
        # Sort along the global Y-axis from top to bottom (decreasing Y)
        raw_pts.sort(key=lambda pt: pt[1], reverse=True)
        section_points_list = [(pt[0], pt[1], labels[idx]) for idx, pt in enumerate(raw_pts)]
        
        sides[side_name] = {
            "side": side_name,
            "key": f"{name}_{side_name}",
            "approach_x": section_points_list[0][0],
            "approach_y": section_points_list[0][1],
            "axis": "section_line",
            "sections": BIG_SHELF_SPECS["sections"],
            "section_points_template": section_points_list,
            "section_labels_template": labels,
        }
    return sides


def build_pallet_sides(
    name: str, 
    cx: float, 
    cy: float, 
    yaw: float
) -> Dict[str, Any]:
    """
    Generates sides and waypoints for standard static pallet boxes.

    Pallet dimensions are 1.22m length (local X) by 0.8m width (local Y).
    Approaches are generally along X axis (xminus, xplus) unless special-cased.
    """
    sides = {}
    target_sides = SPECIAL_PALLET_APPROACHES.get(name, ["xminus", "xplus"])
    
    for side_name in target_sides:
        if side_name in ("xminus", "xplus"):
            # Approach is along local X axis
            local_dx = (PALLET_BOX_SPECS["length"] / 2.0) + APPROACH_DISTANCE_M
            if side_name == "xminus":
                local_dx = -local_dx
            local_dy = 0.0
            label = "A" if side_name == "xminus" else "D"
        else:
            # Approach is along local Y axis (e.g., pallet_box_0_yplus)
            local_dx = 0.0
            local_dy = (PALLET_BOX_SPECS["width"] / 2.0) + APPROACH_DISTANCE_M
            if side_name == "yminus":
                local_dy = -local_dy
            label = "B" if side_name == "yplus" else "C"

        gx, gy = apply_transformation_matrix(cx, cy, yaw, local_dx, local_dy)
        sides[side_name] = {
            "side": side_name,
            "key": f"{name}_{side_name}",
            "approach_x": gx,
            "approach_y": gy,
            "axis": None,
            "sections": 1,
            "section_labels": [label],
            "section_points_template": [(gx, gy, label)],
            "section_labels_template": [label],
        }
    return sides


def build_mobile_cluster_sides(mobiles_list: List[Dict[str, Any]]) -> Dict[str, Any]:
    """
    Groups dynamic mobile pallet boxes into a single 'mobile_cluster' entity.

    The pallets align along the Y axis, and the sweep runs left-to-right (ordered by X).
    """
    sides = {}
    # Order pallets from West to East (lowest X to highest X)
    sorted_mobiles = sorted(mobiles_list, key=lambda m: m["x"])
    labels = ["A", "B", "C"]
    
    for side_name in ["xminus", "xplus"]:
        section_points_list = []
        for index, mobile in enumerate(sorted_mobiles):
            # Apply approach offset on world coordinates (Yaw is zero)
            offset_x = -APPROACH_DISTANCE_M if side_name == "xminus" else APPROACH_DISTANCE_M
            gx = round(mobile["x"] + offset_x, 3)
            gy = round(mobile["y"], 3)
            label = labels[index]
            section_points_list.append((gx, gy, label))
            
        if not section_points_list:
            continue
            
        sides[side_name] = {
            "side": side_name,
            "key": f"{MOBILE_CLUSTER_NAME}_{side_name}",
            "approach_x": section_points_list[0][0],
            "approach_y": section_points_list[0][1],
            "axis": "section_line",
            "sections": len(section_points_list),
            "section_points_template": section_points_list,
            "section_labels_template": labels,
        }
    return sides


def parse_sdf(path: str) -> Dict[str, Dict[str, Any]]:
    """
    Main XML Parser for the Gazebo SDF World.

    Extracts all physical static models, parses poses, translates local centers
    to global coordinates, resolves section lines, and dynamically merges the
    mobile pallets into a unified cluster.

    Args:
        path: Absolute path to the .sdf world file.

    Returns:
        A mapping of item names to their complete geometry configuration.
    """
    try:
        world_root = ET.parse(path).getroot().find("world")
    except Exception as e:
        print(f"[Parser Error] Failed to read SDF world from path {path}: {e}")
        return {}

    items = {}
    parsed_mobile_pallets = []

    for model_include in world_root.findall("include"):
        name_element = model_include.find("name")
        pose_element = model_include.find("pose")
        if name_element is None or pose_element is None:
            continue

        name = name_element.text.strip()
        
        # Classify model type from name string
        lower_name = name.lower()
        if "shelf_big" in lower_name:
            item_type = "shelf_big"
        elif "pallet" in lower_name:
            item_type = "pallet"
        elif "shelf" in lower_name:
            item_type = "shelf"
        else:
            continue

        pose_values = pose_element.text.strip().split()
        raw_x = float(pose_values[0])
        raw_y = float(pose_values[1])
        yaw = float(pose_values[5])

        # Resolve coordinate offsets
        cx, cy = get_item_geometric_center(item_type, raw_x, raw_y, yaw)

        # Build side and section specifications
        if item_type == "shelf":
            sides = build_small_shelf_sides(name, cx, cy, yaw)
        elif item_type == "shelf_big":
            sides = build_big_shelf_sides(name, cx, cy, yaw)
        else:
            sides = build_pallet_sides(name, cx, cy, yaw)

        items[name] = {
            "name": name,
            "type": item_type,
            "x": cx,
            "y": cy,
            "model_x": raw_x,
            "model_y": raw_y,
            "model_yaw": yaw,
            "region": classify_cartesian_region(cx, cy),
            "sides": sides,
            "done": set(),
        }

        # Cache mobile reference for cluster consolidation
        if name in MOBILE_PALLET_SET:
            parsed_mobile_pallets.append({"name": name, "x": cx, "y": cy})

    # Consolidate mobile pallets into a cluster
    if parsed_mobile_pallets:
        cluster_cx = sum(m["x"] for m in parsed_mobile_pallets) / len(parsed_mobile_pallets)
        cluster_cy = sum(m["y"] for m in parsed_mobile_pallets) / len(parsed_mobile_pallets)
        cluster_sides = build_mobile_cluster_sides(parsed_mobile_pallets)

        items[MOBILE_CLUSTER_NAME] = {
            "name": MOBILE_CLUSTER_NAME,
            "type": "mobile_cluster",
            "x": cluster_cx,
            "y": cluster_cy,
            "region": classify_cartesian_region(cluster_cx, cluster_cy),
            "sides": cluster_sides,
            "done": set(),
        }
        
        # Remove individual mobile pallets to keep clean item list
        for mobile_pallet in parsed_mobile_pallets:
            items.pop(mobile_pallet["name"], None)

    # Steel Column Structural Bypass Exception
    # Dynamically shift Section C of shelf_big_0_xplus and shelf_big_4_xminus 
    # to Y = -13.50 to safely clear the physical steel column obstacle at Y = -15.00
    structural_shifts = [
        ("shelf_big_0", "xplus"),
        ("shelf_big_4", "xminus")
    ]
    
    for shelf_name, side_name in structural_shifts:
        if shelf_name in items and side_name in items[shelf_name]["sides"]:
            side_spec = items[shelf_name]["sides"][side_name]
            if "section_points_template" in side_spec:
                shifted_points = []
                for gx, gy, label in side_spec["section_points_template"]:
                    if label == "C":
                        # Safely offset Y coordinate to -13.50 to clear inflation boundary
                        gy = -13.50
                    shifted_points.append((gx, gy, label))
                
                side_spec["section_points_template"] = shifted_points
                
                # Re-sync approach coordinates if they point to the shifted Section C
                if side_spec["section_points_template"]:
                    side_spec["approach_x"] = side_spec["section_points_template"][0][0]
                    side_spec["approach_y"] = side_spec["section_points_template"][0][1]

    return items