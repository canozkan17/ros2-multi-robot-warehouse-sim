"""Bag analysis helper for odom, AMCL, and TF consistency.

Usage:
    python3 analyze_bag.py /path/to/bag_dir [--robot robot1]

The script reads a ROS 2 MCAP bag directory and prints a compact summary with:
- odom and AMCL rates
- odom straightness metrics in the odom frame
- map->odom correction jump sizes from /tf
- odom-to-AMCL consistency when both streams are available
"""

from __future__ import annotations

import argparse
import math
import statistics
from pathlib import Path

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from tf2_msgs.msg import TFMessage


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def timestamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def nearest_before(series, timestamp: float):
    if not series or timestamp < series[0][0]:
        return None

    low = 0
    high = len(series) - 1
    while low <= high:
        mid = (low + high) // 2
        if series[mid][0] <= timestamp:
            low = mid + 1
        else:
            high = mid - 1
    return series[high]


def extract_pose_from_odom(message: Odometry):
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    return position.x, position.y, quaternion_to_yaw(
        orientation.x, orientation.y, orientation.z, orientation.w
    )


def extract_pose_from_amcl(message: PoseWithCovarianceStamped):
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    return position.x, position.y, quaternion_to_yaw(
        orientation.x, orientation.y, orientation.z, orientation.w
    )


def analyze_bag(bag_path: Path, robot_name: str):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag_path), storage_id="mcap"), ConverterOptions("", ""))

    odom = []
    amcl = []
    map_odom = []

    odom_topic = f"/{robot_name}/odom"
    amcl_topic = f"/{robot_name}/amcl_pose"

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == odom_topic:
            message = deserialize_message(data, Odometry)
            odom.append((timestamp_seconds(message.header.stamp), *extract_pose_from_odom(message)))
        elif topic == amcl_topic:
            message = deserialize_message(data, PoseWithCovarianceStamped)
            amcl.append((timestamp_seconds(message.header.stamp), *extract_pose_from_amcl(message)))
        elif topic == "/tf":
            message = deserialize_message(data, TFMessage)
            for transform in message.transforms:
                if transform.header.frame_id == "map" and transform.child_frame_id == f"{robot_name}/odom":
                    translation = transform.transform.translation
                    rotation = transform.transform.rotation
                    map_odom.append(
                        (
                            timestamp_seconds(transform.header.stamp),
                            translation.x,
                            translation.y,
                            quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w),
                        )
                    )

    odom.sort(key=lambda item: item[0])
    amcl.sort(key=lambda item: item[0])
    map_odom.sort(key=lambda item: item[0])

    if len(odom) < 2:
        raise RuntimeError("Not enough odom samples to analyze")

    odom_dists = []
    odom_yaw_steps = []
    for prev, curr in zip(odom, odom[1:]):
        _, x0, y0, yaw0 = prev
        _, x1, y1, yaw1 = curr
        odom_dists.append(math.hypot(x1 - x0, y1 - y0))
        odom_yaw_steps.append(abs(normalize_angle(yaw1 - yaw0)))

    start_x, start_y = odom[0][1], odom[0][2]
    end_x, end_y = odom[-1][1], odom[-1][2]
    line_length = math.hypot(end_x - start_x, end_y - start_y)
    lateral_deviation = []
    if line_length > 1e-9:
        for _, x, y, _ in odom:
            lateral_deviation.append(
                abs((end_x - start_x) * (start_y - y) - (start_x - x) * (end_y - start_y)) / line_length
            )
    else:
        lateral_deviation = [0.0]

    amcl_rate = 0.0
    if len(amcl) >= 2:
        amcl_rate = len(amcl) / (amcl[-1][0] - amcl[0][0])

    map_odom_steps = []
    map_odom_yaw_steps = []
    for prev, curr in zip(map_odom, map_odom[1:]):
        _, x0, y0, yaw0 = prev
        _, x1, y1, yaw1 = curr
        map_odom_steps.append(math.hypot(x1 - x0, y1 - y0))
        map_odom_yaw_steps.append(abs(normalize_angle(yaw1 - yaw0)))

    consistency_errors = []
    for ts, odom_x, odom_y, odom_yaw in odom:
        transform = nearest_before(map_odom, ts)
        amcl_pose = nearest_before(amcl, ts)
        if transform is None or amcl_pose is None:
            continue

        _, map_x, map_y, map_yaw = transform
        _, amcl_x, amcl_y, amcl_yaw = amcl_pose

        cos_yaw = math.cos(map_yaw)
        sin_yaw = math.sin(map_yaw)
        predicted_x = map_x + cos_yaw * odom_x - sin_yaw * odom_y
        predicted_y = map_y + sin_yaw * odom_x + cos_yaw * odom_y
        predicted_yaw = normalize_angle(map_yaw + odom_yaw)

        consistency_errors.append(
            (
                math.hypot(predicted_x - amcl_x, predicted_y - amcl_y),
                abs(normalize_angle(predicted_yaw - amcl_yaw)),
            )
        )

    return {
        "odom_count": len(odom),
        "amcl_count": len(amcl),
        "map_odom_count": len(map_odom),
        "duration_s": odom[-1][0] - odom[0][0],
        "odom_rate_hz": len(odom) / (odom[-1][0] - odom[0][0]),
        "amcl_rate_hz": amcl_rate,
        "total_odom_distance_m": sum(odom_dists),
        "net_odom_dx_m": end_x - start_x,
        "net_odom_dy_m": end_y - start_y,
        "net_odom_dyaw_rad": normalize_angle(odom[-1][3] - odom[0][3]),
        "max_lateral_deviation_m": max(lateral_deviation),
        "mean_lateral_deviation_m": statistics.mean(lateral_deviation),
        "mean_step_distance_m": statistics.mean(odom_dists),
        "mean_step_yaw_deg": math.degrees(statistics.mean(odom_yaw_steps)),
        "map_odom_mean_step_m": statistics.mean(map_odom_steps) if map_odom_steps else 0.0,
        "map_odom_p95_step_m": sorted(map_odom_steps)[int(0.95 * (len(map_odom_steps) - 1))]
        if map_odom_steps
        else 0.0,
        "map_odom_mean_yaw_step_deg": math.degrees(statistics.mean(map_odom_yaw_steps)) if map_odom_yaw_steps else 0.0,
        "odom_to_amcl_mean_pos_error_m": statistics.mean(err[0] for err in consistency_errors)
        if consistency_errors
        else None,
        "odom_to_amcl_p95_pos_error_m": sorted(err[0] for err in consistency_errors)[
            int(0.95 * (len(consistency_errors) - 1))
        ]
        if consistency_errors
        else None,
        "odom_to_amcl_mean_yaw_error_deg": math.degrees(statistics.mean(err[1] for err in consistency_errors))
        if consistency_errors
        else None,
        "odom_to_amcl_p95_yaw_error_deg": math.degrees(
            sorted(err[1] for err in consistency_errors)[int(0.95 * (len(consistency_errors) - 1))]
        )
        if consistency_errors
        else None,
    }


def print_summary(bag_path: Path, robot_name: str):
    summary = analyze_bag(bag_path, robot_name)
    print(f"Bag: {bag_path}")
    print(f"Robot: {robot_name}")
    for key in [
        "odom_count",
        "amcl_count",
        "map_odom_count",
        "duration_s",
        "odom_rate_hz",
        "amcl_rate_hz",
        "total_odom_distance_m",
        "net_odom_dx_m",
        "net_odom_dy_m",
        "net_odom_dyaw_rad",
        "max_lateral_deviation_m",
        "mean_lateral_deviation_m",
        "mean_step_distance_m",
        "mean_step_yaw_deg",
        "map_odom_mean_step_m",
        "map_odom_p95_step_m",
        "map_odom_mean_yaw_step_deg",
        "odom_to_amcl_mean_pos_error_m",
        "odom_to_amcl_p95_pos_error_m",
        "odom_to_amcl_mean_yaw_error_deg",
        "odom_to_amcl_p95_yaw_error_deg",
    ]:
        print(f"{key}: {summary[key]}")


def parse_args():
    parser = argparse.ArgumentParser(description="Analyze a ROS 2 MCAP bag for odom/AMCL drift.")
    parser.add_argument("bag_path", type=Path, help="Path to the bag directory containing metadata.yaml and .mcap files")
    parser.add_argument("--robot", default="robot1", help="Robot namespace to analyze, default: robot1")
    return parser.parse_args()


def main():
    args = parse_args()
    if not args.bag_path.exists():
        raise SystemExit(f"Bag path does not exist: {args.bag_path}")
    print_summary(args.bag_path, args.robot)


if __name__ == "__main__":
    main()
