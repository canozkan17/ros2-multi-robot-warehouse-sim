#!/usr/bin/env python3
"""
plot_trajectory_overlay.py - Post-hoc trajectory visualization for the
multi-robot warehouse fleet.

Reads per-robot AMCL pose streams from a recorded rosbag2 (mcap) container
and overlays their X/Y trajectories on the static occupancy grid map used
by all three robots' map_server instances. Optionally marks the position
at which a given robot reported a FAILED status, read from the same bag's
/{robot}/status stream.

This tool is intentionally decoupled from diagnosis.py: it does not
subscribe to any live topic and performs no online analysis. It is a
pure offline artifact generator, run once per completed mission bag.

Usage:
    python3 plot_trajectory_overlay.py \
        --bag /path/to/run_nominal_n3 \
        --map-yaml /path/to/warehouse_map.yaml \
        --output fig_8_2_nominal_trajectories.pdf \
        --robots robot1 robot2 robot3 \
        --title "N=3 Nominal Coverage"

    python3 plot_trajectory_overlay.py \
        --bag /path/to/run13_failure \
        --map-yaml /path/to/warehouse_map.yaml \
        --output fig_8_6_run13_case_study.pdf \
        --robots robot1 robot2 robot3 \
        --mark-failure robot2 \
        --title "Mid-Mission Failure and Reallocation (Run #13)"
"""

import argparse
import json
import os
import yaml

# Headless-safe backend: this tool only ever saves a file, never opens an
# interactive window, so we pin the Agg backend unconditionally to avoid
# WSLg/X11 backend resolution issues (cf. Chapter 7's broader discussion
# of WSL2-specific rendering pitfalls).
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


ROBOT_COLORS = {
    "robot1": "red",
    "robot2": "blue",
    "robot3": "green",
}
DEFAULT_COLOR = "black"


# ==============================================================================
# BAG READING HELPERS
# ==============================================================================

def _detect_storage_id(bag_path: str) -> str:
    """Reads metadata.yaml to determine the storage plugin used at record time."""
    metadata_path = os.path.join(bag_path, "metadata.yaml")
    if not os.path.exists(metadata_path):
        return "mcap"
    with open(metadata_path, "r") as f:
        meta = yaml.safe_load(f)
    try:
        return meta["rosbag2_bagfile_information"]["storage_identifier"]
    except (KeyError, TypeError):
        return "mcap"


def _open_bag_reader(bag_path: str) -> rosbag2_py.SequentialReader:
    storage_id = _detect_storage_id(bag_path)
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def _read_bag_streams(bag_path: str, pose_topics: dict, status_topics: dict):
    """
    Single-pass extraction of pose (x, y) arrays and status payload lists
    for every requested topic, keyed by robot name.

    Returns:
        poses: Dict[str, Tuple[List[float], List[float]]]  -> robot -> (xs, ys)
        statuses: Dict[str, List[dict]]                     -> robot -> [json payloads]
    """
    reader = _open_bag_reader(bag_path)

    topic_type_map = {}
    for meta in reader.get_all_topics_and_types():
        topic_type_map[meta.name] = meta.type

    pose_topic_to_robot = {v: k for k, v in pose_topics.items()}
    status_topic_to_robot = {v: k for k, v in status_topics.items()}

    poses = {robot: ([], []) for robot in pose_topics.keys()}
    statuses = {robot: [] for robot in status_topics.keys()}

    # Resolve message classes only for topics actually relevant to this run
    msg_class_cache = {}

    def _get_msg_class(topic_name: str):
        if topic_name not in msg_class_cache:
            type_str = topic_type_map.get(topic_name)
            if type_str is None:
                msg_class_cache[topic_name] = None
            else:
                msg_class_cache[topic_name] = get_message(type_str)
        return msg_class_cache[topic_name]

    while reader.has_next():
        topic_name, data, _t = reader.read_next()

        if topic_name in pose_topic_to_robot:
            msg_class = _get_msg_class(topic_name)
            if msg_class is None:
                continue
            msg = deserialize_message(data, msg_class)
            robot = pose_topic_to_robot[topic_name]
            xs, ys = poses[robot]
            xs.append(msg.pose.pose.position.x)
            ys.append(msg.pose.pose.position.y)

        elif topic_name in status_topic_to_robot:
            msg_class = _get_msg_class(topic_name)
            if msg_class is None:
                continue
            msg = deserialize_message(data, msg_class)
            robot = status_topic_to_robot[topic_name]
            try:
                payload = json.loads(msg.data)
            except json.JSONDecodeError:
                continue
            statuses[robot].append(payload)

    return poses, statuses


# ==============================================================================
# MAP LOADING
# ==============================================================================

def _load_map(map_yaml_path: str):
    """Loads the PGM occupancy grid and computes its metric extent from the
    accompanying YAML descriptor, following the standard nav2 map_server
    convention (image row 0 corresponds to the map's maximum-Y edge)."""
    with open(map_yaml_path, "r") as f:
        map_meta = yaml.safe_load(f)

    map_dir = os.path.dirname(os.path.abspath(map_yaml_path))
    pgm_path = os.path.join(map_dir, map_meta["image"])

    resolution = float(map_meta["resolution"])
    origin_x, origin_y = float(map_meta["origin"][0]), float(map_meta["origin"][1])

    img = Image.open(pgm_path)
    img_array = np.array(img)
    height, width = img_array.shape[0], img_array.shape[1]

    extent = [
        origin_x,
        origin_x + width * resolution,
        origin_y,
        origin_y + height * resolution,
    ]
    return img_array, extent


# ==============================================================================
# PLOTTING
# ==============================================================================

def plot_overlay(bag_path, map_yaml_path, output_path, robots,
                  mark_failure_robots, title, dpi):
    pose_topics = {r: f"/{r}/amcl_pose" for r in robots}
    status_topics = {r: f"/{r}/status" for r in robots}

    poses, statuses = _read_bag_streams(bag_path, pose_topics, status_topics)
    img_array, extent = _load_map(map_yaml_path)

    width_m = extent[1] - extent[0]
    height_m = extent[3] - extent[2]
    fig_w = 10.0
    fig_h = fig_w * (height_m / width_m)

    fig, ax = plt.subplots(figsize=(fig_w, fig_h))
    ax.imshow(img_array, cmap="gray", extent=extent, origin="upper", zorder=0)

    for robot in robots:
        xs, ys = poses[robot]
        if not xs:
            continue
        color = ROBOT_COLORS.get(robot, DEFAULT_COLOR)
        ax.plot(xs, ys, "-", color=color, linewidth=1.3, alpha=0.9,
                label=robot, zorder=2)
        # Mark spawn point
        ax.plot(xs[0], ys[0], "o", color=color, markersize=5, zorder=3)

    for robot in mark_failure_robots:
        failed_payloads = [p for p in statuses.get(robot, [])
                            if p.get("status") == "FAILED"]
        if not failed_payloads:
            print(f"[WARN] No FAILED status message found for {robot} in bag.")
            continue
        fx = failed_payloads[-1]["x"]
        fy = failed_payloads[-1]["y"]
        ax.scatter([fx], [fy], marker="X", s=140, color="black",
                   edgecolors="white", linewidths=1.0, zorder=4,
                   label=f"{robot} failure point")
        ax.annotate(f"({fx:.2f}, {fy:.2f})", (fx, fy),
                    textcoords="offset points", xytext=(8, 8),
                    fontsize=8, color="black")

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=8, framealpha=0.9)
    if title:
        ax.set_title(title)

    fig.tight_layout()
    fig.savefig(output_path, dpi=dpi)
    print(f"[OK] Saved figure to: {output_path}")


# ==============================================================================
# CLI ENTRYPOINT
# ==============================================================================

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True,
                         help="Path to the rosbag2 directory (containing metadata.yaml).")
    parser.add_argument("--map-yaml", required=True,
                         help="Path to warehouse_map.yaml.")
    parser.add_argument("--output", required=True,
                         help="Output image path (.png or .pdf).")
    parser.add_argument("--robots", nargs="+", default=["robot1", "robot2", "robot3"])
    parser.add_argument("--mark-failure", nargs="*", default=[],
                         help="Robot name(s) whose FAILED status point should be marked.")
    parser.add_argument("--title", default=None)
    parser.add_argument("--dpi", type=int, default=300)
    args = parser.parse_args()

    plot_overlay(
        bag_path=args.bag,
        map_yaml_path=args.map_yaml,
        output_path=args.output,
        robots=args.robots,
        mark_failure_robots=args.mark_failure,
        title=args.title,
        dpi=args.dpi,
    )


if __name__ == "__main__":
    main()