#!/usr/bin/env python3
"""
spawn.py — Reliable robot spawning for Gazebo via ros_gz_sim.

Wraps `ros2 run ros_gz_sim create` with:
  - idempotency  : skips spawn if the model is already present in the world
  - retry loop   : retries the create command until it succeeds
  - presence check: polls `gz model` until Gazebo confirms the model is visible

Usage:
    from spawn import spawn_robot

    spawn_robot(
        world_name='tugbot_warehouse_clean',
        robot_name='robot1',
        sdf_file='/path/to/model_robot1.sdf',
        x='0.0', y='1.0', z='0.01',
    )
"""

import subprocess
import time
import os


# ── internal ──────────────────────────────────────────────────────────────────

def _run(cmd, timeout=30.0):
    try:
        return subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
            check=False,
            env=os.environ.copy(),  # launch ortamını miras al
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None


def _model_present(robot_name: str, quick: bool = False) -> bool:
    result = _run(['gz', 'model', '--list'], timeout=5.0)
    return bool(
        result
        and result.returncode == 0
        and robot_name in result.stdout
    )

# ── public ────────────────────────────────────────────────────────────────────

def spawn_robot(
    world_name: str,
    robot_name: str,
    sdf_file: str,
    x: str,
    y: str,
    z: str,
    retry_sec: float = 2.0,
) -> None:
    """
    Spawn `robot_name` into the running Gazebo world and block until
    the model is confirmed visible.

    Steps:
      1. Idempotency check — return immediately if already present.
      2. Run `ros2 run ros_gz_sim create` in a retry loop until exit-code 0.
      3. Poll `gz model` until Gazebo confirms the model exists.

    Args:
        world_name : Gazebo world name (used only for log messages).
        robot_name : Model name inside Gazebo (must match SDF <model name>).
        sdf_file   : Absolute path to the robot's SDF file.
        x, y, z    : Spawn coordinates as strings (e.g. '0.0').
        retry_sec  : Seconds to wait between retries.
    """
    # ── 1. idempotency ────────────────────────────────────────────────────────
    if _model_present(robot_name):
        print(f'[spawn:{robot_name}] already present in world "{world_name}", skipping', flush=True)
        return

    print(f'[spawn:{robot_name}] spawning into world "{world_name}" at ({x}, {y}, {z})', flush=True)

    cmd = [
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-name', robot_name,
        '-file', sdf_file,
        '-x', x,
        '-y', y,
        '-z', z,
    ]

    # ── 2. create with retry ──────────────────────────────────────────────────
    while True:
        result = _run(cmd, timeout=30.0)
        if result and result.returncode == 0:
            print(f'[spawn:{robot_name}] create command succeeded', flush=True)
            break
        stderr = (result.stderr.strip() if result else 'process failed or timed out')
        print(f'[spawn:{robot_name}] create failed, retrying in {retry_sec}s | {stderr}', flush=True)
        time.sleep(retry_sec)

    # ── 3. wait for Gazebo model visibility ───────────────────────────────────
    print(f'[spawn:{robot_name}] waiting for Gazebo model visibility...', flush=True)
    while not _model_present(robot_name):
        print(f'[spawn:{robot_name}] not visible yet, retrying in {retry_sec}s', flush=True)
        time.sleep(retry_sec)
        
    time.sleep(1.0)
    print(f'[spawn:{robot_name}] ready', flush=True)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--world-name', required=True)
    parser.add_argument('--robot-name', required=True)
    parser.add_argument('--sdf-file',   required=True)
    parser.add_argument('--x',          required=True)
    parser.add_argument('--y',          required=True)
    parser.add_argument('--z',          required=True)
    parser.add_argument('--retry-sec',  type=float, default=2.0)
    args = parser.parse_args()
    spawn_robot(
        world_name=args.world_name,
        robot_name=args.robot_name,
        sdf_file=args.sdf_file,
        x=args.x, y=args.y, z=args.z,
        retry_sec=args.retry_sec,
    )