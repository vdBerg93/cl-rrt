# CL-RRT — Closed-Loop RRT Motion Planner

[![CI](https://github.com/vdBerg93/cl-rrt/actions/workflows/ci.yml/badge.svg)](https://github.com/vdBerg93/cl-rrt/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Docker](https://github.com/vdBerg93/cl-rrt/actions/workflows/docker.yml/badge.svg)](https://github.com/vdBerg93/cl-rrt/actions/workflows/docker.yml)

A real-time motion planning system for autonomous vehicles, implemented in C++ with ROS.
Generates collision-free trajectories at **5 Hz** using a Closed-Loop Rapidly-exploring Random Tree (CL-RRT) with a kinematic bicycle vehicle model.

> Based on: [Kuwata et al., 2009 — "Real-time Motion Planning with Applications to Autonomous Urban Driving"](http://acl.mit.edu/papers/KuwataTCST09.pdf)

---

> **Demo screenshot / GIF:** _Add an Rviz recording here showing the tree expanding and the best path being highlighted in green._

---

## Overview

Standard RRT planners expand the tree by connecting random samples with straight-line segments, which ignores the vehicle's dynamics and produces kinematically infeasible paths. CL-RRT solves this by replacing the straight-line connection with a **closed-loop simulation**: a reference path is generated to the sample point, then a lateral and longitudinal controller drives the simulated vehicle along that reference for a fixed time horizon. Only trajectories that are dynamically feasible and collision-free are added to the tree.

This produces a tree of smooth, drivable trajectories that respect the vehicle's steering limits, speed limits, and turning radius.

## Architecture

```
  ┌─────────────────┐     MotionRequest      ┌──────────────────────────────────────────┐
  │ mission_planner │ ─────────────────────► │              rrt (motion planner)        │
  │                 │                        │                                          │
  │  - 5 Hz loop    │     Trajectory         │  planMotion()                            │
  │  - Rviz goals   │ ◄───────────────────── │    ├── transformStateToLocal()           │
  └─────────────────┘                        │    ├── initializeTree()  ← previous path │
                                             │    ├── expandTree() loop (200 ms)        │
  ┌─────────────────┐     State (50 Hz)      │    │     ├── sampleAroundVehicle()       │
  │ state_estimator │ ─────────────────────► │    │     ├── sortNodes() [heuristic]     │
  │                 │                        │    │     ├── getReference()              │
  │  - odometry     │                        │    │     └── Simulation::propagate()     │
  │  - steer angle  │                        │    └── extractBestPath() [backtrack]     │
  └─────────────────┘                        └──────────────────────────────────────────┘
```

## Key Technical Concepts

### Closed-Loop Prediction
Each tree expansion step runs a forward simulation of the vehicle under closed-loop control. A reference path is constructed from the current node to the sample point, then a PID-based lateral controller and a longitudinal controller track that reference for a fixed time horizon (configurable, default ~2 s). The resulting simulated trajectory becomes a new tree node only if it is admissible (within acceleration/steering limits) and collision-free.

### Dual Heuristics
Node selection uses two heuristics that shift in probability once the goal is first reached:

| Phase | Probability | Heuristic | Objective |
|---|---|---|---|
| Exploration | 70% | Minimum Dubins distance to sample | Maximize coverage of the state space |
| Optimization | 30% | Minimum total travel cost to sample | Refine path quality once goal is reached |

After the goal is found, the ratio inverts (30% / 70%), biasing the tree toward cost improvement over pure exploration.

### Cost Function
Node cost is a weighted sum evaluated along each simulated trajectory segment:

```
cost += W[0] * v * dt           // Travel time
      + W[1] * |κ|              // Path curvature (κ = tan(δ) / L)
      + W[2] * exp(-W[3] * Dobs) // Obstacle proximity (exponential penalty)
      + W[4] * d2Lane           // Lateral deviation from lane centerline
```

Weights are loaded at runtime from the ROS parameter server (`motionplanner/weight_*`).

### Goal-Biased Expansion
After each successful node addition, a goal-directed expansion is attempted. Feasibility is checked geometrically: if the last added node lies outside both minimum-turning-radius circles centered on the goal pose, and the heading alignment is within ±22.5°, a direct goal connection is simulated.

### Vehicle Model
The vehicle is modelled as a kinematic bicycle model extended with first-order actuator dynamics and an understeer gradient:

```
ẋ     = v · cos(θ)
ẏ     = v · sin(θ)
θ̇     = (v / L) · tan(δ_eff)       // δ_eff accounts for understeer: Kus · v²
δ̇     = (δ_cmd - δ) / Td           // Steering actuator lag
v̇     = (a_cmd - a) / Ta           // Acceleration actuator lag
```

Parameters for Toyota Prius are derived from manufacturer specifications and the Kuwata et al. paper. A second vehicle preset (Talos) is also included.

## Packages

| Package | Description |
|---|---|
| `rrt` | Core motion planner: tree expansion, simulation, controller, reference generation |
| `mission_planner` | Reads Rviz goal poses, issues motion requests at 5 Hz |
| `state_estimator` | Reads odometry and steering angle, publishes vehicle state at 50 Hz |
| `car_msgs` | Custom ROS message and service definitions |

## Prerequisites

- ROS Melodic (Ubuntu 18.04)
- `vision_msgs`
- `robot_localization`
- RViz (part of `ros-melodic-desktop-full`)

## Installation

### Option A — Docker (recommended)

Requires [Docker](https://docs.docker.com/get-docker/). On Windows, run from a **WSL2** terminal (Windows 11 includes WSLg, which provides the display server needed for RViz).

A pre-built image is published to GitHub Container Registry on every commit to master:

```bash
# Pull and run the pre-built image
docker run --rm -it \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --network host \
  ghcr.io/vdberg93/cl-rrt:latest \
  roslaunch rrt rrt.launch
```

Or build locally from source:

```bash
git clone https://github.com/vdBerg93/cl-rrt.git && cd cl-rrt
docker compose up --build
```

### Option B — Native ROS

Requires ROS Melodic on Ubuntu 18.04.

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/vdBerg93/cl-rrt.git
cd .. && catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

> Building in Release mode is strongly recommended — it significantly increases the number of tree nodes expanded per second within the 200 ms planning budget.

## Running

```bash
# Docker
docker compose up

# Native
roslaunch rrt rrt.launch
```

This starts:
1. RViz with the tree marker array (`/tree_markerarray`) and the goal pose topic (`/move_base_simple/goal`)
2. The mission planner node
3. The RRT motion planner node

Set a goal by using the **2D Nav Goal** tool in RViz.

## Configuration

All tunable parameters are loaded from `rrt/launch/rrt.launch`:

| Parameter | Default | Description |
|---|---|---|
| `motionplanner/weight_distance` | — | Cost weight for travel time |
| `motionplanner/weight_curvature` | — | Cost weight for path curvature |
| `motionplanner/weight_obstacle_gain` | — | Exponential obstacle penalty gain |
| `motionplanner/weight_obstacle_slope` | — | Exponential obstacle penalty slope |
| `motionplanner/weight_lanedeviation` | — | Cost weight for lane deviation |

### Changing planning frequency
The planner runs at 5 Hz by default. To change this:
1. Update the timer duration in `expandTree()` in `motionplanner.cpp` (currently `Timer timer(200)`)
2. Update the mission planner loop rate in `mission_planner/`

### Adding collision detection
The collision check is intentionally left as a stub (`collisioncheck.cpp`) to allow integration with any obstacle representation. Implement `myCollisionCheck()` to plug in an occupancy grid, object list, or point cloud. The function is called once per simulation step inside `Simulation::propagate()`.

### Changing the vehicle
Vehicle parameters are defined in `rrt/include/rrt/vehicle.h`. Call `veh.setPrius()` or `veh.setTalos()`, or define a new preset using the same struct fields.

## Reference

Y. Kuwata, J. Teo, G. Fiore, S. Karaman, E. Frazzoli, J. P. How,
**"Real-time Motion Planning with Applications to Autonomous Urban Driving,"**
*IEEE Transactions on Control Systems Technology*, 17(5):1105–1118, 2009.
[PDF](http://acl.mit.edu/papers/KuwataTCST09.pdf)

## Author

**Berend van den Berg** — [github.com/vdBerg93](https://github.com/vdBerg93)
