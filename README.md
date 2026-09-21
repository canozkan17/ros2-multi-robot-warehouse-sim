![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)
![Nav2](https://img.shields.io/badge/Nav2-enabled-brightgreen)
![Python](https://img.shields.io/badge/Python-3.12-blue)
![Platform](https://img.shields.io/badge/Platform-WSL2%20%2F%20Ubuntu%2024.04-lightgrey)
![License](https://img.shields.io/badge/License-MIT-lightgrey)

# Fault-Tolerant Decentralized Multi-Robot Warehouse Inspection

A peer-to-peer, masterless fleet of three TurtleBot3 Waffle robots that audits a simulated 22-shelf warehouse of 142 inspection points, coordinates over ROS 2 without a central dispatcher, recovers from localization drift, kinematic stalls, and full agent failure, and flags shelf-surface anomalies with an edge-deployed, explainable computer vision pipeline.

This repository contains the full implementation, simulation assets, diagnostic tooling, and thesis manuscript for the Master's thesis *"Fault-Tolerant Decentralized Multi-Agent Coordination for Smart Warehouse Inspection Systems with Inventory Anomaly Detection."*

---

## Table of Contents

- [Overview](#overview)
- [Key Contributions](#key-contributions)
- [System Architecture](#system-architecture)
- [Repository Structure](#repository-structure)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Core Nodes](#core-nodes)
- [Fault Tolerance Mechanisms](#fault-tolerance-mechanisms)
- [Perception Pipeline](#perception-pipeline)
- [Experimental Results](#experimental-results)
- [Diagnostics and Offline Tooling](#diagnostics-and-offline-tooling)
- [Dataset and Model Assets](#dataset-and-model-assets)
- [Thesis Document](#thesis-document)
- [Known Limitations](#known-limitations)
- [Configuration Notes](#configuration-notes)
- [Citation](#citation)
- [Acknowledgments](#acknowledgments)
- [License](#license)

---

## Overview

Centralized fleet dispatchers are simple to reason about but represent a single point of failure, and dense steel racking in a realistic warehouse routinely disrupts the wireless links such dispatchers depend on. This project investigates whether a small, homogeneous fleet can sustain near-linear coverage throughput and recover autonomously from an agent failure while relying on no centralized planning, monitoring, or arbitration process whatsoever, not even a temporary one.

Three TurtleBot3 Waffle robots operate in a Gazebo Harmonic warehouse world under ROS 2 Jazzy and Nav2. Target waypoints are not hardcoded; they are extracted at runtime from the Gazebo world description, transformed through a homogeneous SE(2) pipeline, and distributed with a deterministic cardinal-sweep heuristic. Robots negotiate exclusive access to narrow aisles through a lease-based, peer-to-peer spatial mutex, recover from AMCL divergence and physical stalls through layered watchdogs, and redistribute a failed peer's remaining workload proportionally to surviving battery levels. A handcrafted LBP/HOG feature pipeline feeds a linear SVM (with an XGBoost fallback) to flag anomalous cardboard shelving, with a closed-form linear back-projection producing a live explainability overlay on the operator dashboard.

## Key Contributions

- A deterministic, cardinal-direction sweep allocator with same-side-lock and same-item priority rules for consistent, near-costless task assignment across independent robot instances.
- A peer-to-peer, TTL-lease spatial mutex protocol for narrow-aisle mutual exclusion, requiring no central lock server.
- A hierarchical, two-stage fault recovery framework spanning AMCL localization divergence, progress-based kinematic stall detection, and Pythagorean parallel-axis goal shifting around a failed peer.
- A battery-proportional, fully decentralized workload reallocation mechanism triggered by either an explicit self-declaration channel or a passive heartbeat timeout.
- An edge-deployable LBP/HOG/linear-SVM anomaly classifier with a structurally dissimilar XGBoost fallback and a closed-form spatial back-projection for explainable AI, avoiding the cost of model-agnostic post-hoc explanation methods.
- A gate-based, deterministic multi-robot launch architecture that replaces hardcoded startup delays with readiness polling on Gazebo, the simulation clock, ROS 2 topics, and Nav2 lifecycle nodes.
- An eighteen-run empirical campaign isolating fleet-size throughput scaling, failure-to-reallocation latency, coverage completeness under timed failure injection, and perception pipeline latency.

## System Architecture

```
                     ┌─────────────────────────────┐
                     │   PyQt5 Operator Dashboard   │
                     │  (status, XAI camera feed,   │
                     │   failure injection panel)   │
                     └───────────────┬──────────────┘
                                     │ ROS 2 topics
        ┌────────────────────────────┼────────────────────────────┐
        │                            │                             │
 ┌──────▼──────┐             ┌───────▼───────┐             ┌───────▼───────┐
 │   robot1    │             │    robot2     │             │    robot3     │
 │ waypoint_   │◄──────────► │  waypoint_    │◄──────────► │  waypoint_    │
 │ sender      │  /side_     │  sender       │  /add_      │  sender       │
 │ + AMCL/Nav2 │  claims,    │  + AMCL/Nav2  │  waypoints, │  + AMCL/Nav2  │
 │ + battery_  │  /shelf_    │  + battery_   │  /mission_  │  + battery_   │
 │   monitor   │  arrived    │    monitor    │  armed      │    monitor    │
 └─────────────┘             └───────────────┘             └───────────────┘
```

Each `waypoint_sender` process embeds three cooperating in-process helper objects rather than three separate ROS 2 nodes, trading fault isolation for zero-latency shared-memory coordination on a 10 Hz control loop:

- **FleetHealthMonitor** — tracks peer heartbeats and triggers reallocation on a five-second timeout.
- **ClaimManager** — issues, renews, and releases six-second TTL leases on shared corridor segments.
- **TaskCoordinator** — owns the parsed warehouse geometry and executes the battery-proportional task-pool split after a confirmed peer failure.

## Repository Structure

```
.
├── README.md
├── LICENSE
├── .gitignore
├── src/
│   ├── warehouse_multi_robot/            # Ament Python ROS 2 package
│   │   ├── warehouse_multi_robot/        # Node and library implementations
│   │   │   ├── waypoint_sender.py        # Core FSM, ClaimManager, TaskCoordinator
│   │   │   ├── item_selection.py         # Cardinal-sweep target selection engine
│   │   │   ├── sdf_parser.py             # Runtime world geometry parser
│   │   │   ├── recovery.py               # Stall detection and open-loop escape
│   │   │   ├── config.py                 # Physical constants and static assignments
│   │   │   ├── battery_monitor.py        # Hybrid battery discharge simulation
│   │   │   ├── mission_gate.py           # Global mission-armed latch
│   │   │   ├── monitoring_dashboard.py   # PyQt5 operator terminal
│   │   │   ├── imu_relay.py              # IMU frame relabeling for Cartographer
│   │   │   ├── odom_cov_adjuster.py      # Standalone odometry covariance relay
│   │   │   ├── window_tiler.py           # WSLg window layout stabilizer
│   │   │   └── waypoint_domain.py        # Convenience re-export module
│   │   ├── launch/
│   │   │   ├── multi_robot_controlled.launch.py  # Gate-based deterministic launch
│   │   │   ├── slam.launch.py            # Standalone Cartographer mapping run
│   │   │   ├── gate.py                   # Readiness polling primitives
│   │   │   └── spawn.py                  # Idempotent Gazebo model spawner
│   │   ├── config/                       # Nav2, bridge, Cartographer, RViz configs
│   │   ├── maps/                         # Cartographer-generated occupancy grid
│   │   ├── models/                       # Namespaced TurtleBot3 Waffle SDF models
│   │   └── worlds/                       # Modified warehouse world description
│   ├── mini_scripts/                     # Offline diagnostic and analysis tooling
│   └── cardbox_dataset/                  # LBP/HOG/SVM training data and frozen weights
├── results/                              # Logged artifacts from the evaluation campaign
│   ├── logs/                             # Diagnosis, localization, and full-run text logs
│   ├── metrics/                          # anomaly_latency, coverage_loss, reallocation_latency, total_audit_time CSVs
│   └── bags/
│       ├── run13_failure/                # rosbag2 recording behind the Chapter 8 corridor-occlusion case study
│       └── run_nominal_n3/               # rosbag2 recording behind the N=3 nominal baseline run
└── writing/
    ├── Proposal/                         # Thesis proposal (IEEEtran)
    └── Thesis/                           # Full manuscript (LaTeX source)
```

## Requirements

| Component | Version |
|---|---|
| OS | Ubuntu 24.04 LTS (tested under WSL2) |
| ROS 2 | Jazzy Jalisco |
| Simulator | Gazebo Harmonic |
| Navigation | Nav2 (`amcl`, `planner_server`, `controller_server`, `behavior_server`, `bt_navigator`) |
| SLAM | Cartographer ROS |
| Python | 3.12 |
| GUI | PyQt5 |
| Perception | OpenCV, scikit-image, scikit-learn, XGBoost, NumPy |

Python-side dependencies are not yet pinned in a `requirements.txt`, and `package.xml` currently declares only the base `rclpy` and `std_msgs` dependencies. Until the manifest is extended, install `opencv-python`, `scikit-image`, `scikit-learn`, `xgboost`, `numpy`, and `PyQt5` manually into the environment used by `colcon build`, in addition to a standard ROS 2 Jazzy, Gazebo Harmonic, Nav2, and Cartographer ROS installation.

## Installation

The repository root doubles as the colcon workspace root (it already contains a `src/` directory), so clone it directly under the name you want the workspace to have:

```bash
git clone https://github.com/canozkan17/ros2-multi-robot-warehouse-sim.git thesis_ws
cd thesis_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Usage

Bring up the full three-robot fleet, Gazebo, RViz2, and the operator dashboard with the deterministic gate-based launch file:

```bash
ros2 launch warehouse_multi_robot multi_robot_controlled.launch.py
```

Arm the mission from the dashboard's dispatch control, or publish the trigger directly:

```bash
ros2 topic pub /mission_start std_msgs/msg/Bool "data: true" --once
```

To regenerate the occupancy grid map from scratch with a single unnamespaced robot:

```bash
ros2 launch warehouse_multi_robot slam.launch.py
```

## Core Nodes

| Executable | Responsibility |
|---|---|
| `waypoint_sender` | Per-robot navigation FSM, spatial claims, task allocation, ML anomaly scanning |
| `battery_monitor` | Simulated hybrid idle/motion battery discharge and fail-stop trigger |
| `mission_gate` | Transient-local latch that arms the fleet exactly once |
| `monitoring_dashboard` | PyQt5 operator terminal with live telemetry, XAI camera overlay, and failure injection |
| `window_tiler` | Feedback-driven RViz2 / Gazebo / dashboard window layout under WSLg |
| `imu_relay` | Relabels raw Gazebo IMU frames for the Cartographer mapping pass |

## Fault Tolerance Mechanisms

- **Spatial mutex** — twelve shared narrow-aisle corridors are protected by six-second TTL leases with a last-come-first-served overwrite rule and a 1.5-second renewal cycle.
- **Localization recovery** — a covariance-gated, two-stage watchdog (local spin, then global reinitialization) responds to AMCL divergence beyond a relaxed startup threshold.
- **Kinematic stall recovery** — a position-progress filter, immune to the false negatives of current-based stall detection on low-friction concrete floors, triggers an alternating backward-drive and rotation escape sequence.
- **Peer-occlusion avoidance** — a Pythagorean parallel-axis coordinate shift routes a robot around a stationary failed peer while preserving orthogonal camera alignment to the shelf face.
- **Decentralized blacklist propagation** — a two-strike deferral counter permanently excludes persistently unreachable shelf sides and dynamically shrinks the affected robot's target quota.
- **Battery-proportional reallocation** — on a confirmed peer failure, survivors deterministically re-partition the entire remaining task pool in proportion to their own state of charge.

## Perception Pipeline

Grayscale frames are reduced to a 160-dimensional uniform LBP texture descriptor and a 1764-dimensional HOG structural descriptor, fused into a 1924-dimensional vector, standardized, and projected onto a 125-component PCA subspace retaining 95 percent of variance. A linear SVM classifies the projected vector, with a structurally dissimilar XGBoost model held in reserve for analytical redundancy. Because the SVM decision boundary is linear in the PCA-reduced space, its weights are back-projected onto the original LBP grid in closed form, producing a JET-colormap explainability overlay without the computational cost of model-agnostic post-hoc methods such as LIME or SHAP.

## Experimental Results

Summary of the eighteen-run evaluation campaign reported in Chapter 8 of the thesis (Gazebo Harmonic, WSL2, real-time factor approximately 1.0):

| Fleet size | Mean Total Audit Time | Speedup | Parallel efficiency |
|---|---|---|---|
| N = 1 | 2443.72 s | 1.00x | 100.0 percent |
| N = 2 | 1229.78 s | 1.99x | 99.4 percent |
| N = 3 | 901.71 s | 2.71x | 90.3 percent |

| Metric | Result |
|---|---|
| Failure-to-reallocation latency (self-declaration channel, 15 samples) | typically a few milliseconds, with two documented WSL2-scheduling outliers up to several hundred milliseconds; every sample remained well below the 5 s heartbeat timeout |
| Mission coverage under timed failure injection (25 / 50 / 75 percent) | 99.77 / 98.36 / 99.30 percent of 142 targets |
| Perception pipeline latency (mean / 99th percentile, 7278 scans as analyzed in the thesis) | 10.17 ms / 32.26 ms, well inside the 1.5 to 4.0 s mechanical scan window |

The underlying per-run and per-scan records are versioned under `results/metrics/` and `results/logs/`; two representative rosbag2 recordings (the N = 3 nominal baseline and the Run 13 corridor-occlusion case study) are versioned under `results/bags/`. Full methodology, per-run breakdowns, and the corridor-occlusion case study behind the largest observed coverage deficit are documented in Chapter 8 of the thesis manuscript.

## Diagnostics and Offline Tooling

`src/mini_scripts/` collects passive, non-intrusive tools used to validate the live system without perturbing its control loops:

| Script | Purpose |
|---|---|
| `diagnosis.py` | Event-driven live mission state, claims, and proximity monitor |
| `localization_diagnose.py` | Fixed-rate AMCL covariance and TF-latency telemetry sampler |
| `amcl_diagnose.py` | Offline correlation of debug-log events against nearest-preceding AMCL samples |
| `analyze_bag.py` | Post-hoc consistency check between filtered odometry and AMCL pose from a rosbag2 recording |
| `bag_to_csv.py` | Extracts odometry, AMCL, and ground-truth model-state streams from a bag into CSV |
| `nav_performance_analyzer.py` | Control-loop oscillation and RMS steering analysis |
| `custom_slam_teleop.py` | Interactive yaw calibration against Gazebo ground truth for the offline mapping pass |
| `plot_trajectory_overlay.py` | Multi-robot trajectory overlay rendering on the static occupancy grid |
| `generate_chapter8_plots.py` | Regenerates the figures used in the experimental results chapter |

## Dataset and Model Assets

`src/cardbox_dataset/` contains the labeled cardboard-shelf image set used to train the anomaly classifier, together with the frozen `StandardScaler`, PCA projection, linear SVM, and XGBoost weights loaded by `waypoint_sender` at runtime. Dataset provenance and licensing terms are recorded in `README.dataset.txt` and `README.roboflow.txt`; consult those files before reusing the imagery outside this project.

## Thesis Document

The complete manuscript, including the theoretical framework, system design, fault-tolerance analysis, and experimental evaluation chapters, is available under `writing/Thesis/UE_Master_Thesis/`. The earlier project proposal is preserved under `writing/Proposal/`.

## Known Limitations

- All results were collected in simulation on a single WSL2 host; no physical hardware trials have been conducted, and the usual simulation-to-reality gap has not been characterized.
- The fault model is non-Byzantine: a failed agent is assumed to stop responding rather than to report misleading state.
- The passive heartbeat-timeout detection channel and the XGBoost classifier fallback were never exercised across the eighteen-run campaign, since no trial produced a silent agent failure or a primary-classifier exception.
- Blacklist propagation operates at shelf-side granularity rather than per-section, which can prune more inspection targets than strictly necessary after a peer failure inside a narrow aisle.

A complete discussion of limitations and proposed extensions is provided in Chapter 9 of the thesis.

## Configuration Notes

`multi_robot_controlled.launch.py` and several configuration files currently contain absolute filesystem paths tied to the original development workstation (for example, the `WMR` workspace path and the RViz2 configuration path). Update these paths to match your own workspace location before launching on a different machine.

## Citation

If this work is useful in your own research, please cite it as follows:

```bibtex
@mastersthesis{ozkan2026warehouse,
  author = {Can Ozkan},
  title  = {Fault-Tolerant Decentralized Multi-Agent Coordination for Smart Warehouse Inspection Systems with Inventory Anomaly Detection},
  school = {University of Europe for Applied Sciences},
  year   = {2026},
  type   = {Master's Thesis}
}
```

Double-check the institution field against your actual title page before relying on this entry.

## Acknowledgments

This project builds on the ROS 2 Nav2 stack, the TurtleBot3 Waffle reference model, Cartographer ROS, and a cardboard-defect image set sourced from Roboflow (see `src/cardbox_dataset/README.roboflow.txt` for attribution details). All coordination, fault-tolerance, geometry-parsing, and perception logic under `src/warehouse_multi_robot/warehouse_multi_robot/` and `src/mini_scripts/` is original work developed for this thesis.

## License

This repository is released under the MIT License. See `LICENSE` for details. Note that the dataset under `src/cardbox_dataset/` may be subject to separate licensing terms; consult `README.roboflow.txt` before redistribution.
