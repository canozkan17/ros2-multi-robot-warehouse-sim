#!/usr/bin/env python3
"""
config.py — Warehouse robot configuration and physical constants
"""

import os

# ── paths ──────────────────────────────────────────────────────────────────────
SDF_PATH = os.path.expanduser(
    "~/thesis_ws/src/warehouse_multi_robot/worlds/tugbot_warehouse_clean.sdf")

# ── approach / tolerance ───────────────────────────────────────────────────────
APPROACH_DIST      = 1.2
FINAL_GOAL_TOL     = 0.35
SECTION_CENTER_TOL = 0.35

# ── shelf geometry ─────────────────────────────────────────────────────────────
SMALL_SHELF_WIDTH = 3.6
SMALL_SECTIONS    = 4
SMALL_STEP        = SMALL_SHELF_WIDTH / SMALL_SECTIONS   # 0.9 m

BIG_SHELF_LENGTH  = 18.0
BIG_SECTIONS      = 4
BIG_STEP          = BIG_SHELF_LENGTH / BIG_SECTIONS      # 4.5 m

# ── section arrival wait ───────────────────────────────────────────────────────
SECTION_WAIT_SEC  = 2.0   # pause between sections after arrival yaw goal

# ── stall detection ────────────────────────────────────────────────────────────
STALL_TIMEOUT_SEC = 5.0

# ── corner recovery ────────────────────────────────────────────────────────────
CORNER_RECOVERY_BACK_SEC    = 0.8
CORNER_RECOVERY_BACK_SPEED  = -0.12
CORNER_RECOVERY_TURN_SEC    = 0.9
CORNER_RECOVERY_TURN_SPEED  = 0.35
CORNER_RECOVERY_ESCAPE_DIST = 0.45

# ── quaternion presets ─────────────────────────────────────────────────────────
Q_X = (0.0, 1.0)        # yaw=0   robot faces +X  (yminus/yplus sides)
Q_Y = (0.7071, 0.7071)  # yaw=90  robot faces +Y  (xminus/xplus sides)

# ── model origin offsets ───────────────────────────────────────────────────────
MODEL_ORIGIN_OFFSETS = {
    "shelf":             (-0.5, 0.0),
    "shelf_big":         (-0.5, 0.0),
    "pallet":            (0.0, 0.0),
    "pallet_box_mobile": (0.0, 0.0),
}

# ── side ownership ─────────────────────────────────────────────────────────────
ITEM_SIDE_OWNER = {
    "shelf_2_yplus":      "robot1", "shelf_2_yminus":     "robot3",
    "shelf_5_yminus":     "robot1", "shelf_5_yplus":      "robot2",
    "shelf_big_3_xplus":  "robot2", "shelf_big_3_xminus": "robot3",
}

# ── per-robot configuration ────────────────────────────────────────────────────
ROBOT_CONFIG = {
    "robot1": {"spawn": (0.0,  1.0), "first_item": "shelf_2",
               "first_side": "yplus",  "owned_regions": [2, 1, 4],
               "priority_dirs": ["north", "east"], "wp_limit": 48},
    "robot2": {"spawn": (0.0,  0.0), "first_item": "shelf_10",
               "first_side": "yplus", "owned_regions": [4, 3],
               "priority_dirs": ["east", "south"], "wp_limit": 47},
    "robot3": {"spawn": (0.0, -1.0), "first_item": "shelf_2",
               "first_side": "yminus", "owned_regions": [2, 3],
               "priority_dirs": ["south", "west"], "wp_limit": 47},
}

# ── special item overrides ─────────────────────────────────────────────────────
SINGLE_SIDE    = {"shelf_big_1": ["xminus"]}
SPECIAL_SIDES  = {"pallet_box_0": ["xminus", "yplus"]}
MOBILE_GROUP   = ["pallet_box_mobile", "pallet_box_mobile_0", "pallet_box_mobile_1"]
MOBILE_SET     = set(MOBILE_GROUP)
MOBILE_CLUSTER = "mobile_cluster"