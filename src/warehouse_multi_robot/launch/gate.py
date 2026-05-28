#!/usr/bin/env python3
"""
gate.py — Deterministic readiness gates for ROS 2 / Gazebo launch sequencing.

Each function blocks the calling thread until the condition is met.
Designed to be called from threading.Thread inside OpaqueFunction.

Usage:
    from gate import wait_for_world, wait_for_clock, wait_for_topics, wait_for_lifecycle
"""

import re
import subprocess
import time


# ── internal ──────────────────────────────────────────────────────────────────

def _run(cmd, timeout=4.0):
    """Run a subprocess, return CompletedProcess or None on failure/timeout."""
    try:
        return subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
            check=False,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None


# ── public gates ──────────────────────────────────────────────────────────────

def wait_for_world(world_name: str, poll_sec: float = 1.0) -> None:
    """
    Block until Gazebo reports a running world.

    Polls `gz topic -l` and looks for /world/<name>/stats or /world/<name>/clock.
    Also accepts any topic matching /world/*/stats or /world/*/clock so the
    caller does not need to know the exact world name ahead of time.
    """
    print(f'[gate:world] waiting for world "{world_name}"...', flush=True)
    while True:
        result = _run(['gz', '--force-version', '8', 'topic', '-l'], timeout=3.0)
        if result and result.returncode == 0:
            topics = result.stdout.splitlines()
            exact   = (f'/world/{world_name}/stats' in topics
                       or f'/world/{world_name}/clock' in topics)
            pattern = any(re.match(r'^/world/.+/(stats|clock)$', t) for t in topics)
            if exact or pattern:
                print(f'[gate:world] ready — world "{world_name}" is up', flush=True)
                return
        print(f'[gate:world] not ready yet, retrying in {poll_sec}s', flush=True)
        time.sleep(poll_sec)


def wait_for_clock(poll_sec: float = 1.0) -> None:
    """
    Block until at least one message arrives on /clock.

    Guarantees the ROS ↔ Gazebo clock bridge is alive before any
    node with use_sim_time=True is started.
    """
    print('[gate:clock] waiting for /clock...', flush=True)
    while True:
        result = _run(
            ['ros2', 'topic', 'echo', '--once', '/clock',
             '--field', 'clock.sec'],
            timeout=3.0,
        )
        if result and result.returncode == 0 and result.stdout.strip():
            print('[gate:clock] ready — /clock is publishing', flush=True)
            return
        print(f'[gate:clock] not ready yet, retrying in {poll_sec}s', flush=True)
        time.sleep(poll_sec)


def wait_for_topics(topics: list, poll_sec: float = 1.0) -> None:
    """
    Block until every topic in `topics` has at least one active publisher.

    Uses `ros2 topic info --verbose` and checks Publisher count > 0.
    All topics must be satisfied simultaneously before returning.
    """
    assert topics, '[gate:topics] topics list must not be empty'
    print(f'[gate:topics] waiting for {len(topics)} topic(s): {topics}', flush=True)
    while True:
        missing = []
        for topic in topics:
            result = _run(['ros2', 'topic', 'info', topic, '--verbose'], timeout=4.0)
            if not result or result.returncode != 0:
                missing.append(topic)
                continue
            match = re.search(r'Publisher count:\s*(\d+)', result.stdout)
            if not match or int(match.group(1)) < 1:
                missing.append(topic)
        if not missing:
            print('[gate:topics] ready — all topics have publishers', flush=True)
            return
        print(f'[gate:topics] still waiting: {missing}', flush=True)
        time.sleep(poll_sec)


def wait_for_lifecycle(nodes: list, poll_sec: float = 1.0) -> None:
    """
    Block until every lifecycle node in `nodes` reports state active.

    Uses `ros2 lifecycle get <node>` and accepts both "active" and "active [3]"
    output formats (Humble vs Iron/Jazzy differences).

    `nodes` should be fully-qualified names, e.g. ['/robot1/amcl', ...].
    All nodes must be active simultaneously before returning.
    """
    assert nodes, '[gate:lifecycle] nodes list must not be empty'
    print(f'[gate:lifecycle] waiting for {len(nodes)} node(s) to become active', flush=True)
    while True:
        not_active = []
        for node in nodes:
            result = _run(['ros2', 'lifecycle', 'get', node], timeout=4.0)
            if not result or result.returncode != 0:
                not_active.append(node)
                continue
            out = result.stdout.strip().lower()
            if not (out.startswith('active') or 'active [3]' in out):
                not_active.append(node)
        if not not_active:
            print('[gate:lifecycle] ready — all nodes active', flush=True)
            return
        print(f'[gate:lifecycle] still waiting ({len(not_active)}/{len(nodes)}): {not_active}', flush=True)
        time.sleep(poll_sec)
