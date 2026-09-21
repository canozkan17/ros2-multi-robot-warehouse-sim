#!/usr/bin/env python3
"""
amcl_diagnose.py
Collects waypoint/debug events from robot debug logs and (optionally) samples live AMCL poses,
then correlates events with the nearest AMCL sample to compute age and sigma.

Usage examples:
  # Offline: parse existing debug logs and produce CSV
  python3 amcl_diagnose.py --robots robot1 robot2 robot3 --log-dir src/mini_scripts --out /tmp/amcl_report

  # Live: run while system is running (requires rclpy available)
  python3 amcl_diagnose.py --robots robot1 robot2 robot3 --log-dir src/mini_scripts --out /tmp/amcl_report --live --duration 30

Outputs are written under src/mini_scripts/amcl_report/ by default:
- per-robot event CSVs
- per-robot raw AMCL/odom/TF sample CSVs
- a run_summary.json with aggregate metrics
- a run_summary.txt printed to stdout and saved to disk
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import re
import sys
import threading
import time
from collections import deque, defaultdict
from statistics import mean

try:
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage
    RCLPY_AVAILABLE = True
except Exception:
    RCLPY_AVAILABLE = False

EVENT_PATTERNS = [
    (re.compile(r"SENT_GOAL", re.I), 'sent_goal'),
    (re.compile(r"goal rejected", re.I), 'goal_rejected'),
    (re.compile(r"blocked\(|blocked\]", re.I), 'path_blocked'),
    (re.compile(r"path\[\d+\]=reachable", re.I), 'path_reachable'),
    (re.compile(r"ARRIVED", re.I), 'arrived'),
    (re.compile(r"collision ahead", re.I), 'collision_ahead'),
]

# Accept either plain decimal timestamps or integer/decimal timestamps.
TIMESTAMP_RE = re.compile(r"(\d{6,}(?:\.\d+)?)")
AMCL_QOS = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT)
AMCL_QOS_ALT = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
TF_QOS = QoSProfile(depth=100, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT)


def repo_root():
    return Path(__file__).resolve().parents[2]


def default_output_dir():
    return repo_root() / 'src' / 'mini_scripts' / 'amcl_report'


def now_tag():
    return time.strftime('%Y%m%d_%H%M%S')


def ensure_dir(path):
    path.mkdir(parents=True, exist_ok=True)
    return path


def resolve_log_dir(raw_path):
    p = Path(raw_path)
    if p.is_absolute() and p.exists():
        return p
    if p.exists():
        return p.resolve()
    alt = repo_root() / p
    if alt.exists():
        return alt.resolve()
    return p.resolve()


def stamp_to_seconds(stamp):
    """Convert ROS Time-like objects to seconds with broad field compatibility."""
    sec = getattr(stamp, 'sec', 0)
    nanosec = getattr(stamp, 'nanosec', getattr(stamp, 'nsec', 0))
    return float(sec) + float(nanosec) * 1e-9


def parse_debug_log(path):
    events = []
    if not os.path.exists(path):
        return events
    with open(path, 'r', errors='ignore') as f:
        for line in f:
            m = TIMESTAMP_RE.search(line)
            if not m:
                continue
            t = float(m.group(1))
            l = line.strip()
            for pat, etype in EVENT_PATTERNS:
                if pat.search(l):
                    events.append({'time': t, 'type': etype, 'line': l})
                    break
    return events


def quaternion_to_yaw_deg(z, w):
    return math.degrees(2.0 * math.atan2(z, w))


def nearest_before(samples, event_time):
    if not samples:
        return None
    best = None
    for sample in samples:
        if sample[0] <= event_time:
            best = sample
        else:
            break
    return best


def series_stats(samples):
    if len(samples) < 2:
        return {
            'count': len(samples),
            'duration_s': 0.0,
            'rate_hz': 0.0,
            'mean_step_m': 0.0,
            'p95_step_m': 0.0,
            'max_step_m': 0.0,
        }
    duration = samples[-1][0] - samples[0][0]
    steps = [math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(samples, samples[1:])]
    steps_sorted = sorted(steps)
    idx = max(0, int(0.95 * (len(steps_sorted) - 1)))
    return {
        'count': len(samples),
        'duration_s': duration,
        'rate_hz': len(samples) / duration if duration > 0 else 0.0,
        'mean_step_m': mean(steps) if steps else 0.0,
        'p95_step_m': steps_sorted[idx] if steps_sorted else 0.0,
        'max_step_m': max(steps) if steps else 0.0,
    }


class SampleCollector:
    def __init__(self, robots):
        self.robots = robots
        self.amcl = {r: deque(maxlen=10000) for r in robots}
        self.odom = {r: deque(maxlen=10000) for r in robots}
        self.map_odom = {r: deque(maxlen=10000) for r in robots}
        self.amcl_rx_count = {r: 0 for r in robots}
        self.amcl_parse_errors = {r: 0 for r in robots}
        self.amcl_parse_first_error = {r: '' for r in robots}
        self._last_amcl_stamp = {r: float('nan') for r in robots}
        self._lock = threading.Lock()

    def _amcl_cb_factory(self, robot):
        def cb(msg):
            with self._lock:
                self.amcl_rx_count[robot] += 1

            parse_error = False
            err_parts = []

            try:
                stamp = stamp_to_seconds(msg.header.stamp)
            except Exception as e:
                stamp = float('nan')
                parse_error = True
                err_parts.append(f'stamp:{type(e).__name__}:{e}')

            try:
                pose = msg.pose.pose
                x = float(pose.position.x)
                y = float(pose.position.y)
            except Exception as e:
                x = float('nan')
                y = float('nan')
                parse_error = True
                err_parts.append(f'pose:{type(e).__name__}:{e}')

            try:
                cov = msg.pose.covariance
                if cov is not None and len(cov) > 7:
                    sx = math.sqrt(abs(float(cov[0])))
                    sy = math.sqrt(abs(float(cov[7])))
                    sigma = math.hypot(sx, sy)
                else:
                    sigma = float('nan')
                    parse_error = True
                    err_parts.append('cov:missing_or_short')
            except Exception as e:
                sigma = float('nan')
                parse_error = True
                err_parts.append(f'cov:{type(e).__name__}:{e}')

            if parse_error:
                with self._lock:
                    self.amcl_parse_errors[robot] += 1
                    if not self.amcl_parse_first_error[robot]:
                        self.amcl_parse_first_error[robot] = '; '.join(err_parts)[:300]

            # If stamp is invalid, do not store this sample; otherwise we can still
            # store pose with NaN sigma and keep the age/rate diagnostics usable.
            if math.isnan(stamp):
                return

            with self._lock:
                # If the same sample arrives via multiple QoS subscriptions, keep one.
                if not math.isnan(self._last_amcl_stamp[robot]) and abs(stamp - self._last_amcl_stamp[robot]) < 1e-9:
                    return
                self._last_amcl_stamp[robot] = stamp
                self.amcl[robot].append((stamp, x, y, sigma))
        return cb

    def _odom_cb_factory(self, robot):
        def cb(msg):
            try:
                stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
                pose = msg.pose.pose
                x = float(pose.position.x)
                y = float(pose.position.y)
                yaw = quaternion_to_yaw_deg(pose.orientation.z, pose.orientation.w)
            except Exception:
                return
            with self._lock:
                self.odom[robot].append((stamp, x, y, yaw))
        return cb

    def _tf_cb(self, msg):
        try:
            transforms = msg.transforms
        except Exception:
            return
        with self._lock:
            for transform in transforms:
                if transform.header.frame_id != 'map':
                    continue
                for robot in self.robots:
                    if transform.child_frame_id != f'{robot}/odom':
                        continue
                    stamp = float(transform.header.stamp.sec) + float(transform.header.stamp.nanosec) * 1e-9
                    translation = transform.transform.translation
                    rotation = transform.transform.rotation
                    yaw = quaternion_to_yaw_deg(rotation.z, rotation.w)
                    self.map_odom[robot].append((stamp, float(translation.x), float(translation.y), yaw))

    def run(self, duration):
        if not RCLPY_AVAILABLE:
            raise RuntimeError('rclpy not available; cannot run live collector')
        rclpy.init()
        node = rclpy.create_node('amcl_diagnose_collector')
        subs = []
        try:
            for robot in self.robots:
                amcl_topic = f'/{robot}/amcl_pose' if not robot.startswith('/') else f'{robot}/amcl_pose'
                odom_topic = f'/{robot}/odom' if not robot.startswith('/') else f'{robot}/odom'
                subs.append(node.create_subscription(PoseWithCovarianceStamped, amcl_topic, self._amcl_cb_factory(robot), AMCL_QOS))
                subs.append(node.create_subscription(PoseWithCovarianceStamped, amcl_topic, self._amcl_cb_factory(robot), AMCL_QOS_ALT))
                subs.append(node.create_subscription(Odometry, odom_topic, self._odom_cb_factory(robot), 10))
            subs.append(node.create_subscription(TFMessage, '/tf', self._tf_cb, TF_QOS))

            end = time.time() + duration
            while time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
        finally:
            try:
                for sub in subs:
                    node.destroy_subscription(sub)
            except Exception:
                pass
            node.destroy_node()
            rclpy.shutdown()


def preflight_probe_amcl(robots, probe_sec=2.0):
    """Quick probe to check if AMCL messages are visible to this process.

    Returns a dict: {robot: sample_count_seen_during_probe}
    """
    if not RCLPY_AVAILABLE:
        return {r: 0 for r in robots}

    counts = {r: 0 for r in robots}
    rclpy.init()
    node = rclpy.create_node('amcl_diagnose_preflight')
    subs = []
    try:
        for robot in robots:
            topic = f'/{robot}/amcl_pose' if not robot.startswith('/') else f'{robot}/amcl_pose'

            def _cb(msg, rr=robot):
                counts[rr] += 1

            subs.append(node.create_subscription(PoseWithCovarianceStamped, topic, _cb, AMCL_QOS))
            subs.append(node.create_subscription(PoseWithCovarianceStamped, topic, _cb, AMCL_QOS_ALT))

        end = time.time() + probe_sec
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        try:
            for sub in subs:
                node.destroy_subscription(sub)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

    return counts


def correlate_events(events, amcl_data, odom_data, tf_data):
    rows = []
    for ev in events:
        robot = ev.get('robot')
        amcl = nearest_before(amcl_data.get(robot, []), ev['time'])
        odom = nearest_before(odom_data.get(robot, []), ev['time'])
        tf = nearest_before(tf_data.get(robot, []), ev['time'])

        if amcl:
            amcl_stamp, amcl_x, amcl_y, amcl_sigma = amcl
            raw_age = ev['time'] - amcl_stamp
            # If clocks are from different domains (e.g., old epoch log vs sim time),
            # age is meaningless and must not be treated as stale localization.
            amcl_age = raw_age if abs(raw_age) <= 120.0 else ''
            clock_domain_mismatch = (amcl_age == '')
        else:
            amcl_stamp = amcl_x = amcl_y = amcl_sigma = amcl_age = ''
            clock_domain_mismatch = False

        if odom:
            odom_stamp, odom_x, odom_y, odom_yaw = odom
            odom_age = ev['time'] - odom_stamp
        else:
            odom_stamp = odom_x = odom_y = odom_yaw = odom_age = ''

        if tf:
            tf_stamp, tf_x, tf_y, tf_yaw = tf
            tf_age = ev['time'] - tf_stamp
        else:
            tf_stamp = tf_x = tf_y = tf_yaw = tf_age = ''

        rows.append({
            'robot': robot,
            'event_time': ev['time'],
            'event_type': ev['type'],
            'amcl_stamp': amcl_stamp,
            'amcl_age': amcl_age,
            'amcl_x': amcl_x,
            'amcl_y': amcl_y,
            'amcl_sigma': amcl_sigma,
            'clock_domain_mismatch': int(clock_domain_mismatch),
            'odom_stamp': odom_stamp,
            'odom_age': odom_age,
            'odom_x': odom_x,
            'odom_y': odom_y,
            'odom_yaw': odom_yaw,
            'map_odom_stamp': tf_stamp,
            'map_odom_age': tf_age,
            'map_odom_x': tf_x,
            'map_odom_y': tf_y,
            'map_odom_yaw': tf_yaw,
            'line': ev['line'][:240],
        })
    return rows


def summarize_rows(rows, age_thr=2.0, sigma_thr=0.35):
    per_robot = defaultdict(lambda: {'total': 0, 'bad_age': 0, 'bad_sigma': 0, 'clock_mismatch': 0})
    for row in rows:
        robot = row['robot']
        per_robot[robot]['total'] += 1
        try:
            if int(row.get('clock_domain_mismatch', 0)) == 1:
                per_robot[robot]['clock_mismatch'] += 1
        except Exception:
            pass
        try:
            if row['amcl_age'] != '' and float(row['amcl_age']) > age_thr:
                per_robot[robot]['bad_age'] += 1
        except Exception:
            pass
        try:
            if row['amcl_sigma'] != '' and float(row['amcl_sigma']) > sigma_thr:
                per_robot[robot]['bad_sigma'] += 1
        except Exception:
            pass
    return per_robot


def summarize_event_counts(events):
    per_robot = defaultdict(lambda: defaultdict(int))
    for event in events:
        per_robot[event['robot']][event['type']] += 1
    return per_robot


def write_csv(path, fieldnames, rows):
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_series_csv(path, samples, fieldnames):
    rows = []
    for sample in samples:
        row = {}
        for idx, field in enumerate(fieldnames):
            row[field] = sample[idx]
        rows.append(row)
    write_csv(path, fieldnames, rows)


def build_summary_text(summary, event_counts, sample_stats):
    lines = []
    for robot in sorted(summary.keys()):
        stat = summary[robot]
        ss = sample_stats.get(robot, {})
        ec = event_counts.get(robot, {})
        lines.append(
            f"{robot}: events={stat['total']}, bad_age={stat['bad_age']}, bad_sigma={stat['bad_sigma']}, "
            f"amcl_rate={ss.get('amcl_rate_hz', 0.0):.2f}Hz, odom_rate={ss.get('odom_rate_hz', 0.0):.2f}Hz, "
            f"tf_rate={ss.get('tf_rate_hz', 0.0):.2f}Hz, amcl_p95_step={ss.get('amcl_p95_step_m', 0.0):.3f}m, "
            f"odom_p95_step={ss.get('odom_p95_step_m', 0.0):.3f}m, tf_p95_step={ss.get('tf_p95_step_m', 0.0):.3f}m, "
            f"sent_goal={ec.get('sent_goal', 0)}, goal_rejected={ec.get('goal_rejected', 0)}, "
            f"collision_ahead={ec.get('collision_ahead', 0)}, arrived={ec.get('arrived', 0)}"
        )
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--robots', nargs='+', required=True)
    ap.add_argument('--log-dir', default='src/mini_scripts')
    ap.add_argument('--out', default=str(default_output_dir()))
    ap.add_argument('--live', action='store_true', help='subscribe to /robotN/amcl_pose and /robotN/odom while running')
    ap.add_argument('--duration', type=float, default=300.0, help='seconds to collect live samples')
    args = ap.parse_args()

    out_root = Path(args.out)
    if not out_root.is_absolute():
        out_root = repo_root() / out_root
    run_dir = ensure_dir(out_root / now_tag())

    log_dir = resolve_log_dir(args.log_dir)

    all_events = []
    for robot in args.robots:
        log_path = log_dir / f'{robot}_debug_log.txt'
        events = parse_debug_log(str(log_path))
        for event in events:
            event['robot'] = robot
            all_events.append(event)
        print(f'{robot}: parsed {len(events)} events from {log_path}')

    collector = SampleCollector(args.robots)
    if args.live:
        if not RCLPY_AVAILABLE:
            print('rclpy not available, cannot run in --live mode', file=sys.stderr)
            sys.exit(1)
        probe_counts = preflight_probe_amcl(args.robots, probe_sec=2.0)
        print('AMCL preflight samples (2s): ' + ', '.join(f"{r}={probe_counts[r]}" for r in args.robots))
        print(f'Collecting live samples for {args.duration}s...')
        collector.run(args.duration)
        print('Finished collecting live samples')

    amcl_data = {robot: sorted(list(collector.amcl[robot]), key=lambda sample: sample[0]) for robot in args.robots}
    odom_data = {robot: sorted(list(collector.odom[robot]), key=lambda sample: sample[0]) for robot in args.robots}
    tf_data = {robot: sorted(list(collector.map_odom[robot]), key=lambda sample: sample[0]) for robot in args.robots}

    rows = correlate_events(all_events, amcl_data, odom_data, tf_data)
    rows_by_robot = defaultdict(list)
    for row in rows:
        rows_by_robot[row['robot']].append(row)

    fieldnames = [
        'robot', 'event_time', 'event_type',
        'amcl_stamp', 'amcl_age', 'amcl_x', 'amcl_y', 'amcl_sigma', 'clock_domain_mismatch',
        'odom_stamp', 'odom_age', 'odom_x', 'odom_y', 'odom_yaw',
        'map_odom_stamp', 'map_odom_age', 'map_odom_x', 'map_odom_y', 'map_odom_yaw',
        'line',
    ]

    for robot in args.robots:
        write_csv(run_dir / f'{robot}_events_amcl.csv', fieldnames, rows_by_robot.get(robot, []))
        write_series_csv(run_dir / f'{robot}_amcl_samples.csv', amcl_data.get(robot, []), ['stamp', 'x', 'y', 'sigma_xy'])
        write_series_csv(run_dir / f'{robot}_odom_samples.csv', odom_data.get(robot, []), ['stamp', 'x', 'y', 'yaw_deg'])
        write_series_csv(run_dir / f'{robot}_map_odom_samples.csv', tf_data.get(robot, []), ['stamp', 'x', 'y', 'yaw_deg'])
        print(f"{robot}: wrote {len(rows_by_robot.get(robot, []))} correlated events to {run_dir}")

    summary = summarize_rows(rows)
    event_counts = summarize_event_counts(all_events)
    sample_stats = {
        robot: {
            'amcl_rate_hz': series_stats(amcl_data.get(robot, [])).get('rate_hz', 0.0),
            'amcl_p95_step_m': series_stats(amcl_data.get(robot, [])).get('p95_step_m', 0.0),
            'odom_rate_hz': series_stats(odom_data.get(robot, [])).get('rate_hz', 0.0),
            'odom_p95_step_m': series_stats(odom_data.get(robot, [])).get('p95_step_m', 0.0),
            'tf_rate_hz': series_stats(tf_data.get(robot, [])).get('rate_hz', 0.0),
            'tf_p95_step_m': series_stats(tf_data.get(robot, [])).get('p95_step_m', 0.0),
            'amcl_count': len(amcl_data.get(robot, [])),
            'odom_count': len(odom_data.get(robot, [])),
            'tf_count': len(tf_data.get(robot, [])),
        }
        for robot in args.robots
    }

    if args.live:
        for robot in args.robots:
            ss = sample_stats[robot]
            print(
                f"AMCL callback stats {robot}: rx={collector.amcl_rx_count.get(robot, 0)}, "
                f"stored={ss['amcl_count']}, parse_errors={collector.amcl_parse_errors.get(robot, 0)}, "
                f"first_error={collector.amcl_parse_first_error.get(robot, '') or 'none'}"
            )
            if ss['amcl_count'] == 0 or ss['odom_count'] == 0 or ss['tf_count'] == 0:
                print(
                    f"WARNING: {robot} missing live samples -> "
                    f"amcl={ss['amcl_count']}, odom={ss['odom_count']}, tf={ss['tf_count']}"
                )

    print('\nSummary (age>2.0s or sigma>0.35):')
    for robot in args.robots:
        stat = summary.get(robot, {'total': 0, 'bad_age': 0, 'bad_sigma': 0})
        ss = sample_stats[robot]
        print(
            f"{robot}: total_events={stat['total']}, bad_age={stat['bad_age']}, bad_sigma={stat['bad_sigma']}, clock_mismatch={stat.get('clock_mismatch', 0)}, "
            f"amcl_rate={ss['amcl_rate_hz']:.2f}Hz, odom_rate={ss['odom_rate_hz']:.2f}Hz, tf_rate={ss['tf_rate_hz']:.2f}Hz, "
            f"amcl_samples={ss['amcl_count']}, odom_samples={ss['odom_count']}, tf_samples={ss['tf_count']}"
        )

    summary_text = build_summary_text(summary, event_counts, sample_stats)
    summary_json = {
        'robots': args.robots,
        'live': args.live,
        'duration_s': args.duration if args.live else None,
        'output_dir': str(run_dir),
        'thresholds': {'amcl_age_s': 2.0, 'amcl_sigma_xy_m': 0.35},
        'clock_domain_rule': 'amcl_age is ignored when |event_time-amcl_stamp| > 120s',
        'event_counts': {robot: dict(event_counts.get(robot, {})) for robot in args.robots},
        'sample_stats': sample_stats,
        'correlation_summary': {robot: summary.get(robot, {}) for robot in args.robots},
        'amcl_callback_stats': {
            robot: {
                'rx': collector.amcl_rx_count.get(robot, 0),
                'stored': sample_stats[robot]['amcl_count'],
                'parse_errors': collector.amcl_parse_errors.get(robot, 0),
                'first_error': collector.amcl_parse_first_error.get(robot, ''),
            }
            for robot in args.robots
        },
        'note': 'Correlation uses the latest sample at or before each event time for causal inspection.',
    }

    with open(run_dir / 'run_summary.txt', 'w', encoding='utf-8') as f:
        f.write(summary_text + '\n')
    with open(run_dir / 'run_summary.json', 'w', encoding='utf-8') as f:
        json.dump(summary_json, f, indent=2, sort_keys=True)

    print(f'\nSaved run summary to {run_dir / "run_summary.txt"}')
    print(f'Saved structured metrics to {run_dir / "run_summary.json"}')
    print('\nDone. Inspect CSVs in the run directory for per-event details.')


if __name__ == '__main__':
    main()
