#!/usr/bin/env python3
"""
window_tiler.py - State-Stabilized Window Management for Multi-Robot Warehouse.

Dynamically queries screen geometry and applies a feedback-driven closed-loop 
stabilization mechanism to align RViz2, Gazebo Sim, and the PyQt5 Dashboard.
"""

import sys
import time
import subprocess
import re


DASHBOARD_TARGET_HEIGHT  = 485   
DASHBOARD_WIDTH_REDUCE   = 20    
DASHBOARD_X_PADDING      = 0     
TASKBAR_PADDING_Y        = 250    
GAP_BETWEEN_PANELS       = 5 

def run_cmd(cmd):
    """Executes system terminal commands and returns stripped output safely."""
    try:
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=False)
        return result.stdout.strip()
    except Exception:
        return ""


def get_screen_geometry():
    """Queries XWayland active display dimensions. Fallback to 1707x1019 on failure."""
    out = run_cmd(["xdotool", "getdisplaygeometry"])
    if out:
        parts = out.split()
        if len(parts) >= 2:
            try:
                return int(parts[0]), int(parts[1])
            except ValueError:
                pass
    return 1707, 1019


def get_window_id(search_term, search_type="name"):
    """Finds window ID of only mapped/visible windows, filtering out tiny splash screens."""
    if search_type == "class":
        out = run_cmd(["xdotool", "search", "--onlyvisible", "--class", search_term])
    else:
        out = run_cmd(["xdotool", "search", "--onlyvisible", "--name", search_term])
    
    if out:
        lines = [line.strip() for line in out.split("\n") if line.strip()]
        for win_id in lines:
            # Query window size to filter out splash screens or loading popups under 300x300
            geom_out = run_cmd(["xdotool", "getwindowgeometry", win_id])
            size_match = re.search(r"Geometry:\s*(\d+)x(\d+)", geom_out)
            if size_match:
                w, h = int(size_match.group(1)), int(size_match.group(2))
                if w > 300 and h > 300:  # Splash screens are typically very small
                    return win_id
    return None


def find_window(options):
    """Iterates through search options to resolve window handles under WSLg."""
    for term, s_type in options:
        win_id = get_window_id(term, s_type)
        if win_id:
            return win_id
    return None


def tile_window_stabilized(window_id, target_x, target_y, target_w, target_h):
    """
    Applies closed-loop alignment stabilization to bypass Windows 11 DWM snapping.
    Detects systematic OS frame padding offsets to prevent infinite retry loops.
    """
    if not window_id:
        return False

    # Step 1: Force remove maximized/fullscreen states
    run_cmd(["wmctrl", "-i", "-r", window_id, "-b", "remove,maximized_vert,maximized_horz,fullscreen"])
    time.sleep(0.05)

    last_dx, last_dy = None, None

    # Step 2: Feedback-driven alignment loop with stable drift interception
    for attempt in range(1, 6):
        run_cmd(["xdotool", "windowmove", "--", window_id, str(target_x), str(target_y)])
        run_cmd(["xdotool", "windowsize", "--", window_id, str(target_w), str(target_h)])
        
        # WSLg Sync Window: Give RDP handshaking thread time to settle
        time.sleep(0.15)

        # Query and parse actual coordinates
        geom_out = run_cmd(["xdotool", "getwindowgeometry", window_id])
        pos_match = re.search(r"Position:\s*(-?\d+),(-?\d+)", geom_out)
        size_match = re.search(r"Geometry:\s*(\d+)x(\d+)", geom_out)

        if pos_match and size_match:
            act_x, act_y = int(pos_match.group(1)), int(pos_match.group(2))
            act_w, act_h = int(size_match.group(1)), int(size_match.group(2))

            dx = abs(act_x - target_x)
            dy = abs(act_y - target_y)
            dw = abs(act_w - target_w)
            dh = abs(act_h - target_h)

            # Accept tight tolerance match
            if dx <= 15 and dy <= 15 and dw <= 15 and dh <= 15:
                print(f"[WINDOW TILER] Window stabilized successfully! (X:{act_x}, Y:{act_y}, W:{act_w}, H:{act_h})", flush=True)
                return True

            # Systematic OS Border Detection: If delta is identical to previous attempt,
            # the window is physically flush against the display bounds. Accept as success.
            if last_dx is not None and last_dy is not None:
                if dx == last_dx and dy == last_dy:
                    print(f"[WINDOW TILER] Systematic OS border detected (Stable Drift - X:{dx}, Y:{dy}). Locked successfully on attempt {attempt}.", flush=True)
                    return True

            last_dx, last_dy = dx, dy
            print(f"[WINDOW TILER] Attempt {attempt} drift: Delta X:{dx}, Y:{dy}, W:{dw}, H:{dh}. Retrying...", flush=True)

    return False


def main():
    print("[WINDOW TILER] State-stabilized orchestrator active. Querying display...", flush=True)
    
    scr_w, scr_h = get_screen_geometry()
    print(f"[WINDOW TILER] Detected XWayland Workspace resolution: {scr_w}x{scr_h}", flush=True)

    # ── DASHBOARD-ANCHOR DYNAMIC CALCULATIONS ────────────────────────────────
    # Fix the dashboard height and shift its Y start coordinate above the taskbar
    bottom_h = DASHBOARD_TARGET_HEIGHT
    bottom_y = scr_h - bottom_h - TASKBAR_PADDING_Y

    # RViz2 & Gazebo dynamically expand to fill 100% of the remaining top screen space
    top_h = bottom_y

    # Boundary guards to protect against coordinate layout crashes
    top_h = max(200, top_h)
    bottom_h = max(100, bottom_h)

    mid_x = scr_w // 2

    targets = {
        "rviz": {
            "options": [("rviz_multi_robot_nav2.rviz", "name"), ("- RViz", "name")],
            "x": 0, 
            "y": 0, 
            "w": mid_x - GAP_BETWEEN_PANELS, 
            "h": top_h,
            "done": False
        },
        "gazebo": {
            "options": [("gz-sim", "class"), ("Gazebo", "name"), ("tugbot_warehouse_clean", "name")],
            "x": mid_x, 
            "y": 0, 
            "w": scr_w - mid_x, 
            "h": top_h,
            "done": False
        },
        "dashboard": {
            "options": [("FLEET INSPECTION TERMINAL", "name")],
            "x": DASHBOARD_X_PADDING, 
            "y": bottom_y, 
            "w": scr_w - DASHBOARD_WIDTH_REDUCE, 
            "h": bottom_h,
            "done": False
        }
    }

    start_time = time.time()
    timeout = 120.0  # 120-second active polling bound for cold boots

    while time.time() - start_time < timeout:
        all_done = True
        for name, cfg in targets.items():
            if not cfg["done"]:
                all_done = False
                win_id = find_window(cfg["options"])
                if win_id:
                    print(f"[WINDOW TILER] Resolved window mapping for {name.upper()} (ID: {win_id}). Aligning...", flush=True)
                    # Set done ONLY if the window has successfully stabilized or hit the OS border limit
                    if tile_window_stabilized(win_id, cfg["x"], cfg["y"], cfg["w"], cfg["h"]):
                        cfg["done"] = True
        
        if all_done:
            print("[WINDOW TILER] All coordinates bound and stabilized! Exiting daemon successfully.", flush=True)
            sys.exit(0)
            
        time.sleep(0.2)

    print("[WINDOW TILER] Daemon timeout reached. Tiling ended with missing nodes.", flush=True)
    sys.exit(0)


if __name__ == "__main__":
    main()