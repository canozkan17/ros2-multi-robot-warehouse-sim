#!/usr/bin/env python3
"""
sdf_parser.py — SDF world file parsing and side/section geometry builders

Public API:
  parse_sdf(path)                              -> items dict
  _prepare_section_line_side_for_pose(side, cx, cy)
  _section_label(side, idx)
  _dist(x1, y1, x2, y2)
"""

import math
import xml.etree.ElementTree as ET

from .config import (
    APPROACH_DIST,
    SMALL_SHELF_WIDTH, SMALL_SECTIONS, SMALL_STEP,
    BIG_SHELF_LENGTH,  BIG_SECTIONS,   BIG_STEP,
    Q_X, Q_Y, MODEL_ORIGIN_OFFSETS,
    SINGLE_SIDE, SPECIAL_SIDES,
    MOBILE_GROUP, MOBILE_SET, MOBILE_CLUSTER,
)


# ── geometry helpers ───────────────────────────────────────────────────────────

def _dist(x1, y1, x2, y2):
    return math.hypot(x1 - x2, y1 - y2)


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


# ── item type / region classification ─────────────────────────────────────────

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


# ── side builders ──────────────────────────────────────────────────────────────

def _small_shelf_sides(name, cx, cy):
    half   = SMALL_SHELF_WIDTH / 2.0
    labels = ["A", "B", "C", "D"]
    sides  = {}
    for side in ["yminus", "yplus"]:
        approach_y = round(cy + APPROACH_DIST, 3) if side == "yplus" \
                     else round(cy - APPROACH_DIST, 3)
        # canonical left→right order: A=low x … D=high x
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
    sides  = {}
    for side in SINGLE_SIDE.get(name, ["xminus", "xplus"]):
        ax = (cx - APPROACH_DIST) if side == "xminus" else (cx + APPROACH_DIST)
        # canonical top→bottom order: A=highest y … D=lowest y
        points = [
            (round(ax, 3),
             round(cy + BIG_SHELF_LENGTH / 2.0 - BIG_STEP / 2.0 - i * BIG_STEP, 3),
             label)
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
            "section_labels": (
                ["A"] if side == "xminus" else
                ["B"] if side == "xplus"  else
                ["C"] if side == "yplus"  else ["D"]
            ),
        }
    return sides


def _mobile_cluster_sides(mobiles):
    sides = {}
    for side_name in ["xminus", "xplus"]:
        # order mobiles left→right by x so A..C map to increasing x
        ordered = sorted(mobiles, key=lambda m: m["x"])
        labels  = ["A", "B", "C"]
        points  = []
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
            "qz": qz, "qw": qw,
            "section_points_template": points,
            "section_labels_template": labels,
        }
    return sides


# ── section ordering / labelling ───────────────────────────────────────────────

def _prepare_section_line_side_for_pose(side, cx, cy):
    """Order section points nearest-end first relative to (cx, cy)."""
    pts = side.get("section_points_template", [])
    if not pts:
        return side
    reversed_      = _dist(cx, cy, pts[-1][0], pts[-1][1]) < \
                     _dist(cx, cy, pts[0][0],  pts[0][1])
    ordered        = list(reversed(pts)) if reversed_ else list(pts)
    label_template = side.get("section_labels_template", [])
    ordered_labels = list(reversed(label_template)) if reversed_ else list(label_template)
    prepared = dict(side)
    prepared["section_points"]  = ordered
    prepared["section_labels"]  = ordered_labels if ordered_labels else label_template
    prepared["sections"]        = len(ordered)
    prepared["approach_x"]      = ordered[0][0]
    prepared["approach_y"]      = ordered[0][1]
    prepared["step"]            = 0.0
    return prepared


def _section_label(side, idx):
    labels = side.get("section_labels", [])
    if 0 <= idx < len(labels):
        return labels[idx]
    return "?"


# ── main parser ────────────────────────────────────────────────────────────────

def parse_sdf(path):
    world       = ET.parse(path).getroot().find("world")
    items       = {}
    mobile_refs = []

    for inc in world.findall("include"):
        ne, pe = inc.find("name"), inc.find("pose")
        if ne is None or pe is None:
            continue
        name = ne.text.strip()
        t    = _type(name)
        if not t:
            continue
        v          = pe.text.strip().split()
        x, y, yaw  = float(v[0]), float(v[1]), float(v[5])
        cx, cy     = _model_center(t, x, y, yaw)

        if   t == "shelf":     sides = _small_shelf_sides(name, cx, cy)
        elif t == "shelf_big": sides = _big_shelf_sides(name, cx, cy, yaw)
        else:                  sides = _pallet_sides(name, cx, cy)

        items[name] = {
            "name": name, "type": t, "x": cx, "y": cy,
            "model_x": x, "model_y": y, "model_yaw": yaw,
            "region": _region(cx, cy), "sides": sides, "done": set(),
        }
        if name in MOBILE_SET:
            mobile_refs.append({"name": name, "x": cx, "y": cy})

    if mobile_refs:
        cx = sum(m["x"] for m in mobile_refs) / len(mobile_refs)
        cy = sum(m["y"] for m in mobile_refs) / len(mobile_refs)
        items[MOBILE_CLUSTER] = {
            "name": MOBILE_CLUSTER, "type": "mobile_cluster",
            "x": cx, "y": cy, "region": _region(cx, cy),
            "sides": _mobile_cluster_sides(mobile_refs), "done": set(),
        }
        for name in MOBILE_GROUP:
            items.pop(name, None)

    return items