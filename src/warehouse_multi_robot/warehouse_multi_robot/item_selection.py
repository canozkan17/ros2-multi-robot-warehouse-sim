#!/usr/bin/env python3
"""
item_selection.py - Advanced Decision and Target Selection Engine.

This module encapsulates the high-level decision-making logic of the robots,
including Same-Side-Lock, region/priority sweep selection (Heuristic mode),
and dynamic target reallocation (Greedy mode) for robust multi-agent coordination.
"""

import math
from typing import Dict, List, Tuple, Any, Optional, Set

from .config import ITEM_SIDE_OWNERS, MOBILE_CLUSTER_NAME, SHARED_CORRIDORS
from .sdf_parser import classify_cartesian_region


# ==============================================================================
# 1. MATHEMATICAL AND DIRECTIONAL HELPERS
# ==============================================================================

def calculate_distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """Calculates the Euclidean distance between two 2D points."""
    return math.hypot(x1 - x2, y1 - y2)


def is_item_in_direction(
    current_x: float, 
    current_y: float, 
    target_x: float, 
    target_y: float, 
    direction: str,
    epsilon: float = 1e-3
) -> bool:
    """
    Determines if a target lies within a strict 90-degree angular quadrant (cone)
    relative to the robot's current coordinates to ensure precise cardinal sweeps.
    """
    dx = target_x - current_x
    dy = target_y - current_y
    
    abs_dx = abs(dx)
    abs_dy = abs(dy)
    
    # Widened conic check from 90 to 120 degrees (slope factor 0.58) to accommodate rectangular aisles
    cone_factor = 0.58
    if direction == "north":
        return dy >= (abs_dx * cone_factor) - epsilon and dy > 0
    elif direction == "south":
        return -dy >= (abs_dx * cone_factor) - epsilon and dy < 0
    elif direction == "east":
        return dx >= (abs_dy * cone_factor) - epsilon and dx > 0
    elif direction == "west":
        return -dx >= (abs_dy * cone_factor) - epsilon and dx < 0
    return False


# ==============================================================================
# 2. CLAIM AND LEASE VALIDATORS
# ==============================================================================

def is_side_claimed_by_other(
    side_key: str, 
    robot_name: str, 
    active_claims: Dict[str, Dict[str, Any]], 
    now_ns: int,
    greedy_mode: bool = False
) -> bool:
    """
    Verifies if a specific item side (entire corridor segment) is currently claimed by another active robot.

    Includes a robust timeout check to prevent permanent deadlocks if a peer robot crashes.

    Args:
        side_key: Unique identifier of the side corridor (e.g., 'shelf_big_3_xminus').
        robot_name: Name of the checking robot (e.g., 'robot1').
        active_claims: Global dict of active claims.
        now_ns: Current system time in nanoseconds.

    Returns:
        True if the entire side/corridor is actively claimed by another robot, False otherwise.
    """
    if not active_claims:
        return False
    
    # 1. Direct Side Check: Is the exact target side claimed by another robot?
    claim = active_claims.get(side_key)
    if claim:
        expire_at_ns = int(claim.get("expire_at_ns", 0))
        if expire_at_ns > now_ns and claim.get("robot_id") != robot_name:
            return True
            
    # 2. Corridor Mutex Check: If system is healthy, check the opposite side of the narrow corridor
    if not greedy_mode and SHARED_CORRIDORS:
        corridor_id = SHARED_CORRIDORS.get(side_key)
        if corridor_id:
            # Find the peer side key sharing this corridor
            for peer_side, cid in SHARED_CORRIDORS.items():
                if cid == corridor_id and peer_side != side_key:
                    peer_claim = active_claims.get(peer_side)
                    if peer_claim:
                        peer_expire = int(peer_claim.get("expire_at_ns", 0))
                        if peer_expire > now_ns and peer_claim.get("robot_id") != robot_name:
                            return True
                            
    return False


# ==============================================================================
# 3. TASK AND SECTION STATE VERIFIERS
# ==============================================================================

def get_ordered_sections(
    side_spec: Dict[str, Any], 
    current_x: float, 
    current_y: float
) -> List[Tuple[float, float, str]]:
    """
    Orders section points starting from the one closest to the robot's position,
    with physics-informed dead-end wall avoidance and optimal side-to-side transitions.
    """
    points = side_spec.get("section_points_template", [])
    if not points:
        return []
        
    # Calculate physical layout averages of the shelf points
    avg_x = sum(p[0] for p in points) / len(points)
    avg_y = sum(p[1] for p in points) / len(points)

    # 1. Physics-Informed Wall Rule: Near Right Wall (X > 10.0). East side is a dead end.
    # For right wall shelves, force yplus to sweep Westward (D->A) and yminus to sweep Eastward (A->D).
    # This guarantees the robot is always positioned at the open aisle (West) during side-transitions!
    if avg_x > 10.0 and abs(points[0][0] - points[-1][0]) > 0.1:
        if "yplus" in side_spec["key"]:
            return sorted(points, key=lambda p: p[0], reverse=True)  # Westward sweep (D first)
        else:
            return sorted(points, key=lambda p: p[0])  # Eastward sweep (A first)

    # 2. Physics-Informed Wall Rule: Near Top Wall (Y > 15.0) for vertical shelves. North side is a dead end.
    # Force xplus to sweep Southward (descending Y) and xminus to sweep Northward (ascending Y)
    if avg_y > 15.0 and abs(points[0][1] - points[-1][1]) > 0.1:
        if "xplus" in side_spec["key"]:
            return sorted(points, key=lambda p: p[1])  # Northward sweep
        else:
            return sorted(points, key=lambda p: p[1], reverse=True)  # Southward sweep
            
    # 3. Standard Euclidean Proximity Fallback (for non-blocked open aisle shelves)
    distance_to_start = calculate_distance(current_x, current_y, points[0][0], points[0][1])
    distance_to_end = calculate_distance(current_x, current_y, points[-1][0], points[-1][1])
    
    # Reverse sweep trajectory if closer to the end of the template
    if distance_to_end < distance_to_start:
        return list(reversed(points))
    return list(points)


def get_side_section_labels(side_spec: Dict[str, Any]) -> List[str]:
    """Retrieves all section labels (e.g., ['A', 'B', 'C', 'D']) for a side."""
    if "section_labels_template" in side_spec and side_spec["section_labels_template"]:
        return side_spec["section_labels_template"]
    if "section_labels" in side_spec and side_spec["section_labels"]:
        return side_spec["section_labels"]
    
    # Fallback default naming based on section count
    section_count = int(side_spec.get("sections", 1))
    return [chr(ord('A') + i) for i in range(section_count)]


def is_side_completed(item: Dict[str, Any], side_spec: Dict[str, Any]) -> bool:
    """Checks if all sections of a specific side are marked as done."""
    side_key = side_spec["key"]
    labels = get_side_section_labels(side_spec)
    completed_set = item.get("done", set())
    
    return all(f"{side_key}_{label}" in completed_set for label in labels)


def get_next_uncompleted_section_index(
    item: Dict[str, Any], 
    side_spec: Dict[str, Any],
    ordered_sections: List[Tuple[float, float, str]]
) -> Optional[int]:
    """Finds the index of the first uncompleted section in the ordered list."""
    side_key = side_spec["key"]
    completed_set = item.get("done", set())
    
    for index, (_, _, label) in enumerate(ordered_sections):
        section_key = f"{side_key}_{label}"
        if section_key not in completed_set:
            return index
    return None


# ==============================================================================
# 4. ENGINE DECISION ALGORITHM (HEURISTIC & GREEDY)
# ==============================================================================

def select_next_target(
    items: Dict[str, Any],
    robot_name: str,
    current_x: float,
    current_y: float,
    owned_regions: List[int],
    priority_dirs: List[str],
    active_claims: Dict[str, Dict[str, Any]],
    now_ns: int,
    completed_count: int,
    waypoint_limit: int,
    current_item_name: Optional[str] = None,
    current_side_key: Optional[str] = None,
    greedy_mode: bool = False,
    reallocation_reference_coords: Optional[Tuple[float, float]] = None,
    deferred_side_keys: Optional[Set[str]] = None,
    reallocated_assigned_side_keys: Optional[Set[str]] = None  # Battery-split filter
) -> Optional[Dict[str, Any]]:
    """
    Main State-Free Target Decision Tree.

    This function determines the absolute best next section waypoint for the robot,
    enforcing Same-Side-Lock, item-level side changes, and cardinal search sweeps.

    Args:
        items: Global dictionary of all items and their completions.
        robot_name: Name of the current robot.
        current_x: Current X coordinate of the robot.
        current_y: Current Y coordinate of the robot.
        owned_regions: Ordered list of regions owned by this robot.
        priority_dirs: Cardinal search priority directions list.
        active_claims: Global claim dictionary.
        now_ns: Current timestamp in nanoseconds.
        completed_count: Total waypoints completed by this robot so far.
        waypoint_limit: The target quota threshold for the robot.
        current_item_name: Item the robot is currently positioned at.
        current_side_key: Side the robot is currently positioned at.
        greedy_mode: If True, bypasses all regions, directions, and static owners (Failure State).
        reallocation_reference_coords: (X, Y) of crashed robot's last completed task.

    Returns:
        A dictionary containing keys: 'item_name', 'side_key', 'side_spec', 
        'section_label', 'x', 'y' or None if mission is fully finished.
    """
    # Check if robot has fully reached its assignment quota
    if completed_count >= waypoint_limit:
        return None

    # ==========================================================================
    # RULE 1: SAME-SIDE-LOCK (Enforce completing current side first)
    # ==========================================================================
    if current_item_name and current_side_key:
        item = items.get(current_item_name)
        if item:
            for side_name, side_spec in item["sides"].items():
                if side_spec["key"] == current_side_key:
                    # Order the sections relative to current position
                    ordered = get_ordered_sections(side_spec, current_x, current_y)
                    uncompleted_idx = get_next_uncompleted_section_index(item, side_spec, ordered)
                    
                    if uncompleted_idx is not None:
                        tx, ty, label = ordered[uncompleted_idx]
                        return {
                            "item_name": current_item_name,
                            "side_key": current_side_key,
                            "side_spec": side_spec,
                            "section_label": label,
                            "x": tx,
                            "y": ty,
                            "mode": "SAME_SIDE_LOCK"
                        }

    # ==========================================================================
    # RULE 2: SAME-ITEM SIDE TRANSITION (Check other sides of the current item)
    # ==========================================================================
    if current_item_name:
        item = items.get(current_item_name)
        if item:
            # Sort sides by proximity to current robot position to prevent backtracking
            sorted_sides = sorted(
                item["sides"].values(),
                key=lambda s: calculate_distance(current_x, current_y, s["approach_x"], s["approach_y"])
            )
            for side_spec in sorted_sides:
                # Skip if already completed
                if is_side_completed(item, side_spec):
                    continue
                    
                # Skip if the entire side (corridor segment) is claimed by other robots
                # Bypasses physical corridor cross-checking automatically if greedy_mode is True
                if is_side_claimed_by_other(side_spec["key"], robot_name, active_claims, now_ns, greedy_mode):
                    continue
                    
                # Verify Static Item Ownership (unless in Greedy Recovery Mode)
                if not greedy_mode:
                    static_owner = ITEM_SIDE_OWNERS.get(side_spec["key"])
                    if static_owner and static_owner != robot_name:
                        continue  # This side belongs exclusively to another robot

                # Found eligible side inside same item -> Stand at closest section end point
                ordered = get_ordered_sections(side_spec, current_x, current_y)
                uncompleted_idx = get_next_uncompleted_section_index(item, side_spec, ordered)
                if uncompleted_idx is not None:
                    tx, ty, label = ordered[uncompleted_idx]
                    return {
                        "item_name": current_item_name,
                        "side_key": side_spec["key"],
                        "side_spec": side_spec,
                        "section_label": label,
                        "x": tx,
                        "y": ty,
                        "mode": "SAME_ITEM_SIDE_TRANSITION"
                    }

    # ==============================================================================
    # RULE 3: GREEDY ALLOCATION MODE (Active Failure/Reallocation Recovery)
    # ==============================================================================
    if greedy_mode:
        ref_x = reallocation_reference_coords[0] if reallocation_reference_coords else current_x
        ref_y = reallocation_reference_coords[1] if reallocation_reference_coords else current_y

        best_target = None
        min_distance = float('inf')

        for item_name, item in items.items():
            for side_name, side_spec in item["sides"].items():
                if is_side_completed(item, side_spec):
                    continue
                
                # Exclude permanently blocked/deferred tasks from allocation
                if deferred_side_keys and side_spec["key"] in deferred_side_keys:
                    continue

                # Water-tight Boundary Check: Verify static owner to ensure strict loading balance
                static_owner = ITEM_SIDE_OWNERS.get(side_spec["key"])
                if static_owner and static_owner != robot_name:
                    # If this side belongs to a failed robot, we ONLY take it if it was assigned to us during split
                    if reallocated_assigned_side_keys is not None:
                        if side_spec["key"] not in reallocated_assigned_side_keys:
                            continue  # Belongs to failed robot but split assigned it to another survivor
                    else:
                        continue  # Safe fallback: split info missing

                # Corridor Check: Verify if the target side is already claimed by another survivor.
                # Setting greedy_mode=True bypasses the cross-corridor check as requested.
                if is_side_claimed_by_other(side_spec["key"], robot_name, active_claims, now_ns, greedy_mode=True):
                    continue

                ordered = get_ordered_sections(side_spec, ref_x, ref_y)
                uncompleted_idx = get_next_uncompleted_section_index(item, side_spec, ordered)
                
                if uncompleted_idx is not None:
                    tx, ty, label = ordered[uncompleted_idx]
                    dist = calculate_distance(ref_x, ref_y, tx, ty)
                    if dist < min_distance:
                        min_distance = dist
                        best_target = {
                            "item_name": item_name,
                            "side_key": side_spec["key"],
                            "side_spec": side_spec,
                            "section_label": label,
                            "x": tx,
                            "y": ty,
                            "mode": "GREEDY_REALLOCATION"
                        }
        return best_target

    # ==========================================================================
    # RULE 4: SEQUENTIAL SWEEP ARCHITECTURE (Standard Heuristic Mode)
    # ==========================================================================
    current_region = None
    if current_item_name:
        active_item = items.get(current_item_name)
        if active_item:
            current_region = active_item["region"]
            
    if current_region is None:
        current_region = classify_cartesian_region(current_x, current_y)

    # 1. Step: Search ONLY inside CURRENT region for PRIORITY DIRECTION 1
    if len(priority_dirs) >= 1:
        dir1 = priority_dirs[0]
        candidate_item, candidate_side, candidate_sec = _search_closest_item_in_direction_and_region(
            items, robot_name, current_x, current_y, current_region, dir1, active_claims, now_ns, deferred_side_keys, greedy_mode=greedy_mode
        )
        if candidate_item:
            return _build_target_response(candidate_item, candidate_side, candidate_sec, "HEURISTIC_DIR1")

    # 2. Step: Search ALL owned regions for PRIORITY DIRECTION 2
    if len(priority_dirs) >= 2:
        dir2 = priority_dirs[1]
        best_cand_b = None
        min_dist_b = float('inf')
        
        for region in owned_regions:
            candidate_item, candidate_side, candidate_sec = _search_closest_item_in_direction_and_region(
                items, robot_name, current_x, current_y, region, dir2, active_claims, now_ns, deferred_side_keys, greedy_mode=greedy_mode
            )
            if candidate_item:
                tx, ty, _ = candidate_sec
                dist = calculate_distance(current_x, current_y, tx, ty)
                if dist < min_dist_b:
                    min_dist_b = dist
                    best_cand_b = (candidate_item, candidate_side, candidate_sec)
                    
        if best_cand_b:
            return _build_target_response(best_cand_b[0], best_cand_b[1], best_cand_b[2], "HEURISTIC_DIR2")

    # 3. Step: Fallback search ONLY inside CURRENT region (Direction-Independent)
    candidate_item, candidate_side, candidate_sec = _search_closest_item_in_direction_and_region(
        items, robot_name, current_x, current_y, current_region, None, active_claims, now_ns, deferred_side_keys, reallocated_assigned_side_keys, greedy_mode=greedy_mode
    )
    if candidate_item:
        return _build_target_response(candidate_item, candidate_side, candidate_sec, "HEURISTIC_REGION_FALLBACK")

    # 4. Step: Region/Direction-Independent Fallback
    candidate_item, candidate_side, candidate_sec = _search_closest_item_in_direction_and_region(
        items, robot_name, current_x, current_y, None, None, active_claims, now_ns, deferred_side_keys, reallocated_assigned_side_keys, greedy_mode=greedy_mode
    )
    if candidate_item:
        return _build_target_response(candidate_item, candidate_side, candidate_sec, "HEURISTIC_WORLD_FALLBACK")

    return None


def _search_closest_item_in_direction_and_region(
    items: Dict[str, Any],
    robot_name: str,
    current_x: float,
    current_y: float,
    region: Optional[int],
    direction: Optional[str],
    active_claims: Dict[str, Dict[str, Any]],
    now_ns: int,
    deferred_side_keys: Optional[Set[str]] = None,
    reallocated_assigned_side_keys: Optional[Set[str]] = None,
    greedy_mode: bool = False
) -> Tuple[Optional[str], Optional[Dict[str, Any]], Optional[Tuple[float, float, str]]]:
    closest_item_name = None
    closest_side_spec = None
    closest_section_pt = None
    min_item_distance = float('inf')

    for item_name, item in items.items():
        if region is not None and item["region"] != region:
            continue

        # Calculate the physically closest point on the item's 2D footprint segment to the robot
        item_type = item.get("type", "shelf")
        cx = item["x"]
        cy = item["y"]

        if item_type == "shelf_big":
            # Vertically aligned shelf (length 18.0m, Y ranges from cy - 9.0 to cy + 9.0)
            closest_x = cx
            closest_y = max(cy - 9.0, min(current_y, cy + 9.0))
        elif item_type == "shelf":
            # Horizontally aligned shelf (width 3.6m, X ranges from cx - 1.8 to cx + 1.8)
            closest_x = max(cx - 1.8, min(current_x, cx + 1.8))
            closest_y = cy
        else:
            # Pallets or others, treat as a single point
            closest_x = cx
            closest_y = cy

        # Apply strict 90-degree cardinal direction filter to the closest physical point of the item
        if direction is not None:
            if not is_item_in_direction(current_x, current_y, closest_x, closest_y, direction):
                continue

        best_side_spec = None
        best_section_pt = None
        min_side_distance = float('inf')

        # Sort sides of this valid item by proximity to prevent backtracking within the same item
        sorted_sides = sorted(
            item["sides"].values(),
            key=lambda s: calculate_distance(current_x, current_y, s["approach_x"], s["approach_y"])
        )

        for side_spec in sorted_sides:
            if is_side_completed(item, side_spec):
                continue
                
            # Exclude deferred tasks from standard sweeps
            if deferred_side_keys and side_spec["key"] in deferred_side_keys:
                continue
                
            static_owner = ITEM_SIDE_OWNERS.get(side_spec["key"])
            if static_owner and static_owner != robot_name:
                # In standard heuristic sweeps, we only permit our own static assigned tasks.
                if reallocated_assigned_side_keys is not None and side_spec["key"] in reallocated_assigned_side_keys:
                    pass  # Permitted reallocated task
                else:
                    continue

            # Skip if the entire side (corridor segment) is currently claimed by another robot
            if is_side_claimed_by_other(side_spec["key"], robot_name, active_claims, now_ns, greedy_mode):
                continue

            ordered = get_ordered_sections(side_spec, current_x, current_y)
            uncompleted_idx = get_next_uncompleted_section_index(item, side_spec, ordered)
            if uncompleted_idx is not None:
                sec_pt = ordered[uncompleted_idx]  

                # Side-Level Proximity: No direction check on section points here, we choose the absolute closest uncompleted side!
                dist = calculate_distance(current_x, current_y, sec_pt[0], sec_pt[1])
                if dist < min_side_distance:
                    min_side_distance = dist
                    best_side_spec = side_spec
                    best_section_pt = sec_pt

        if best_side_spec and best_section_pt:
            item_dist = calculate_distance(current_x, current_y, item["x"], item["y"])
            if item_dist < min_item_distance:
                min_item_distance = item_dist
                closest_item_name = item_name
                closest_side_spec = best_side_spec
                closest_section_pt = best_section_pt

    return closest_item_name, closest_side_spec, closest_section_pt

def _build_target_response(
    item_name: str, 
    side_spec: Dict[str, Any], 
    section_pt: Tuple[float, float, str],
    mode_str: str
) -> Dict[str, Any]:
    """Wraps target information into a unified dictionary structure."""
    return {
        "item_name": item_name,
        "side_key": side_spec["key"],
        "side_spec": side_spec,
        "section_label": section_pt[2],
        "x": section_pt[0],
        "y": section_pt[1],
        "mode": mode_str
    }