#!/usr/bin/env python3
"""
item_selection.py — Item/side/section selection logic and geometry helpers

Public API:
  select_next_side_for_item(...) -> side config dict or None
  select_next_item(...)          -> (item_name, side_dict) or (None, None)
  _remaining_sides(...)
  _make_side_entry(...)
  _dist, _adiff, _travel_yaw, _yaw_to_quat
  _side_direction, _priority_rank, _is_claimed_by_other
"""

import math

from .config import ITEM_SIDE_OWNER
from .sdf_parser import _dist


# ── geometry helpers ───────────────────────────────────────────────────────────

def _adiff(t, c):
    """Signed angular difference t - c, wrapped to [-π, π]."""
    d = t - c
    while d >  math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


def _travel_yaw(sx, sy, tx, ty):
    return math.atan2(ty - sy, tx - sx)


def _yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


# ── direction helpers ──────────────────────────────────────────────────────────

def _dir_match(direction, rx, ry, ix, iy, eps=1e-6):
    if direction == "north": return iy > ry + eps
    if direction == "east":  return ix > rx + eps
    if direction == "south": return iy < ry - eps
    if direction == "west":  return ix < rx - eps
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


# ── claim helpers ──────────────────────────────────────────────────────────────

def _claim_owner(side_key, robot, side_claims, now_ns):
    if not side_claims:
        return None
    claim = side_claims.get(side_key)
    if not claim:
        return None
    if int(claim.get("expire_at_ns", 0)) <= int(now_ns):
        return None
    return claim.get("robot_id") or None


def _is_claimed_by_other(side_key, robot, side_claims, now_ns):
    owner = _claim_owner(side_key, robot, side_claims, now_ns)
    return owner is not None and owner != robot


# ── remaining-side filtering ───────────────────────────────────────────────────

def _remaining_sides(item, robot, override, deferred_keys=None):
    """Return sides that still have undone sections and pass ownership checks."""
    sides = []
    for side in item["sides"].values():
        key    = side["key"]
        labels = side.get("section_labels") or side.get("section_labels_template")
        if not labels:
            labels = [chr(ord('A') + i) for i in range(side.get("sections", 1))]

        all_done = all(f"{key}_{lab}" in item.get("done", set()) for lab in labels)
        if all_done:
            continue
        if deferred_keys and f"{item['name']}:{key}" in deferred_keys:
            continue
        owner = ITEM_SIDE_OWNER.get(key)
        if owner and owner != robot and not override:
            continue
        sides.append(side)
    return sides


# ── eligible-side scoring ──────────────────────────────────────────────────────

def _item_direction(item, cx, cy, priority_dirs):
    for direction in priority_dirs:
        if _dir_match(direction, cx, cy, item["x"], item["y"]):
            return direction
    return None


def _item_side_directions(item, cx, cy, robot, override, deferred_keys, priority_dirs=()):
    """Return (rank, dist, key, side, direction) tuples sorted by priority then distance."""
    eligible = []
    for side in _remaining_sides(item, robot, override, deferred_keys):
        direction = _side_direction(side, cx, cy)
        rank      = _priority_rank(direction, priority_dirs)
        if rank is None:
            continue
        eligible.append((
            rank,
            _dist(cx, cy, side["approach_x"], side["approach_y"]),
            side["key"], side, direction,
        ))
    eligible.sort()
    return eligible


def _make_side_entry(item_name, side):
    return {"item_name": item_name, "side_key": side["key"], "side": side}


# ── main selection ─────────────────────────────────────────────────────────────

def select_next_side_for_item(item, robot, override, deferred_keys, side_claims, now_ns, cx, cy):
    """
    Returns the next eligible side (as a dictionary) for the given item.
    Eligible sides are those with remaining sections, not claimed by another robot,
    and passing ITEM_SIDE_OWNER check.
    If multiple sides remain, picks the one whose approach point is closest to (cx, cy).
    Priority directions are NOT used here because the robot is already at this item.
    """
    remaining = _remaining_sides(item, robot, override, deferred_keys)
    if not remaining:
        return None
    remaining = [
        side for side in remaining
        if not _is_claimed_by_other(side["key"], robot, side_claims, now_ns)
    ]
    if not remaining:
        return None

    from .sdf_parser import _prepare_section_line_side_for_pose
    if item["type"] == "mobile_cluster":
        remaining = [_prepare_section_line_side_for_pose(s, cx, cy) for s in remaining]

    best_side = None
    best_dist = float('inf')
    for side in remaining:
        ax = side.get("approach_x")
        ay = side.get("approach_y")
        if ax is None or ay is None:
            # Fallback distance
            d = _dist(cx, cy, item["x"], item["y"])
        else:
            d = _dist(cx, cy, ax, ay)
        
        if d < best_dist:
            best_dist = d
            best_side = side

    return best_side

def select_next_item(items, robot, cx, cy, regions, done_count, limit, priority_dirs,
                     override=False, deferred_keys=None, side_claims=None, now_ns=0):
    """
    Return (best_item_name, initial_side) for the best next item to process,
    or (None, None) when nothing remains.
    
    Selection rule:
    1. Iterate over regions in order.
    2. Over all items in the region with at least one eligible side matching a priority_dir:
       Pick the item whose center is closest to (cx, cy).
    3. For that chosen item, pick the side matching priority_dir that has the closest approach point to (cx, cy).
    """
    if done_count >= limit:
        return None, None

    from .sdf_parser import _prepare_section_line_side_for_pose

    for region in regions:
        best_item_name = None
        best_item_dist = float('inf')
        best_ordered_sides = None

        for item in items.values():
            if item["region"] != region:
                continue
            
            remaining = _remaining_sides(item, robot, override, deferred_keys)
            if not remaining:
                continue
            remaining = [
                side for side in remaining
                if not _is_claimed_by_other(side["key"], robot, side_claims, now_ns)
            ]
            if not remaining:
                continue
            if item["type"] == "mobile_cluster":
                remaining = [_prepare_section_line_side_for_pose(s, cx, cy) for s in remaining]
            
            eligible = _item_side_directions(
                item, cx, cy, robot, override, deferred_keys, priority_dirs)
            eligible = [
                e for e in eligible
                if not _is_claimed_by_other(e[3]["key"], robot, side_claims, now_ns)
            ]
            
            if not eligible:
                continue

            # Distance to the item itself
            item_dist = _dist(cx, cy, item["x"], item["y"])
            if item_dist < best_item_dist:
                best_item_dist = item_dist
                best_item_name = item["name"]
                
                # Pick the eligible side with the best rank, then closest dist
                eligible.sort(key=lambda x: (x[0], x[1]))
                best_ordered_sides = [eligible[0][3]]

        if best_item_name is not None:
            return best_item_name, best_ordered_sides[0]

    return None, None