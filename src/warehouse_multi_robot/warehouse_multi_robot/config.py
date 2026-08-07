#!/usr/bin/env python3
"""
config.py - Warehouse Multi-Robot Configuration and Physical Constants.

This module serves as the single source of truth for physical dimensions, 
sensor thresholds, task assignments, and recovery limits for the autonomous 
multi-robot fleet.
"""

import os

# ==============================================================================
# 1. FILE SYSTEM PATHS
# ==============================================================================
# Path to the Gazebo Harmonic World description file (SDF format)
SDF_WORLD_PATH = os.path.expanduser(
    "~/thesis_ws/src/warehouse_multi_robot/worlds/tugbot_warehouse_clean.sdf"
)


# ==============================================================================
# 2. NAVIGATION AND ALIGNMENT TOLERANCES
# ==============================================================================
# The baseline perpendicular distance (in meters) from the shelf/pallet edge 
# to the robot's inspection standing point.
APPROACH_DISTANCE_M = 1.2

# Acceptable distance threshold (in meters) to consider a navigation goal reached
NAVIGATION_FINAL_GOAL_TOLERANCE_M = 0.35

# Tolerance (in meters) to align with a specific section center during sweeps
SECTION_ALIGNMENT_TOLERANCE_M = 0.35


# ==============================================================================
# 3. PHYSICAL SPECIFICATIONS AND GEOMETRY
# ==============================================================================
# Small Shelf (itemName: shelf_*)
# Dimensions: Width = 3.6m (along local X), Depth = 0.6m (along local Y)
# Local Offset: Origin is shifted by -0.5m on local X axis in Gazebo models
SMALL_SHELF_SPECS = {
    "width": 3.6,
    "depth": 0.6,
    "sections": 4,
    "section_step": 0.9,     # 3.6m / 4 sections
    "offset_local_x": -0.5,
    "offset_local_y": 0.0
}

# Big Shelf (itemName: shelf_big_*)
# Dimensions: Width = 2.1m (along local X), Length = 18.0m (along local Y)
# Local Offset: Origin is shifted by -0.5m on local X axis in Gazebo models
BIG_SHELF_SPECS = {
    "width": 2.1,
    "length": 18.0,
    "sections": 4,
    "section_step": 4.5,     # 18.0m / 4 sections
    "offset_local_x": -0.5,
    "offset_local_y": 0.0
}

# Pallet Box / Mobile Pallet Box (itemName: pallet_box_*)
# Dimensions: Length = 1.22m (along local X), Width = 0.8m (along local Y)
# Local Offset: Origin lies perfectly at the geometric center
PALLET_BOX_SPECS = {
    "length": 1.22,
    "width": 0.8,
    "sections": 1,
    "offset_local_x": 0.0,
    "offset_local_y": 0.0
}


# ==============================================================================
# 4. SWEEP AND SCAN PARAMETERS
# ==============================================================================
# The simulated duration (in seconds) the robot pauses to perform visual scan 
# tasks (mimicking XGBoost inference trigger)
SCAN_DURATION_SEC = 2.0


# ==============================================================================
# 5. CORNER RECOVERY AND STALL PROTECTION
# ==============================================================================
# Maximum allowed duration (in seconds) with zero progress before assuming 
# the robot is stuck or blocked by an obstacle
STALL_CANDIDATE_TIMEOUT_SEC = 12.0
STALL_DIRECT_TIMEOUT_SEC = 10.0

# Recovery Maneuvers Config
RECOVERY_BACKWARD_DURATION_SEC = 0.8
RECOVERY_BACKWARD_SPEED_MPS = -0.12
RECOVERY_ROTATION_DURATION_SEC = 0.9
RECOVERY_ROTATION_SPEED_RPS = 0.35
RECOVERY_ESCAPE_DISTANCE_M = 0.45


# ==============================================================================
# 6. STATIC ITEM SIDE OWNERSHIP (ITEM_SIDE_OWNER)
# ==============================================================================
# Explicit assignments to avoid fleet collisions and balance waypoint targets
ITEM_SIDE_OWNERS = {
    "shelf_2_yplus": "robot1",
    "shelf_2_yminus": "robot3",
    "shelf_5_yplus": "robot1",
}


# ==============================================================================
# 7. FLEET SPECIFICATIONS AND WAYPOINT LIMITS
# ==============================================================================
ROBOTS_SPECIFICATION = {
    "robot1": {
        "spawn_coordinates": (0.0, 1.0),
        "first_item": "shelf_2",
        "first_side": "yplus",
        "owned_regions": [2, 1, 4],
        "priority_directions": ["north", "east"],
        "waypoint_limit": 48
    },
    "robot2": {
        "spawn_coordinates": (0.0, 0.0),
        "first_item": "shelf_10",
        "first_side": "yplus",
        "owned_regions": [4, 3],
        "priority_directions": ["south", "west"],
        "waypoint_limit": 47
    },
    "robot3": {
        "spawn_coordinates": (0.0, -1.0),
        "first_item": "shelf_2",
        "first_side": "yminus",
        "owned_regions": [2, 3],
        "priority_directions": ["west", "south"],
        "waypoint_limit": 47
    },
}


# ==============================================================================
# 8. SPECIAL RULES AND EXCLUSIONS
# ==============================================================================
# Big shelves with unidirectional approach only
SINGLE_APPROACH_SIDES = {
    "shelf_big_1": ["xminus"]
}

# Pallets near obstacles with non-standard approach directions
SPECIAL_PALLET_APPROACHES = {
    "pallet_box_0": ["xminus", "yplus"]
}

# Names of dynamic/mobile pallets mapped to cluster configuration
MOBILE_PALLET_GROUP = ["pallet_box_mobile", "pallet_box_mobile_0", "pallet_box_mobile_1"]
MOBILE_PALLET_SET = set(MOBILE_PALLET_GROUP)
MOBILE_CLUSTER_NAME = "mobile_cluster"

# ==============================================================================
# ==============================================================================
# 9. CORRIDORS
# ==============================================================================
# Map opposite shelf-sides that share the exact same narrow physical corridor
SHARED_CORRIDORS = {
    # --- Large Shelves Corridors (Aisles) ---
    "shelf_big_0_xplus": "corridor_west",
    "shelf_big_4_xminus": "corridor_west",
    
    "shelf_big_4_xplus": "corridor_mid",
    "shelf_big_3_xminus": "corridor_mid",
    
    "shelf_big_3_xplus": "corridor_east",
    "shelf_big_2_xminus": "corridor_east",

    # --- Small Shelves West Column Corridors (Aisles) ---
    "shelf_2_yminus": "corridor_small_west_top",
    "shelf_1_yplus": "corridor_small_west_top",

    "shelf_1_yminus": "corridor_small_west_mid",
    "shelf_0_yplus": "corridor_small_west_mid",

    "shelf_0_yminus": "corridor_small_west_bottom",
    "shelf_yplus": "corridor_small_west_bottom",

    # --- Small Shelves East Column Corridors (Aisles) ---
    "shelf_3_yminus": "corridor_small_east_top",
    "shelf_4_yplus": "corridor_small_east_top",

    "shelf_4_yminus": "corridor_small_east_mid",
    "shelf_6_yplus": "corridor_small_east_mid",

    "shelf_6_yminus": "corridor_small_east_bottom",
    "shelf_5_yplus": "corridor_small_east_bottom",

    # --- Small Shelves Southeast Corner Column Corridors (Aisles) ---
    "shelf_9_yplus": "corridor_small_corner_top",
    "shelf_10_yminus": "corridor_small_corner_top",

    "shelf_8_yplus": "corridor_small_corner_mid",
    "shelf_9_yminus": "corridor_small_corner_mid",

    "shelf_7_yplus": "corridor_small_corner_bottom",
    "shelf_8_yminus": "corridor_small_corner_bottom"
}