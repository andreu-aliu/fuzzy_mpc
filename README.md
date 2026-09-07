# Fuzzy MPC

<div align="center">

### Model Predictive Control for autonomous racing — with physics and fuzzy learning under one hood.

[![Status: Work in Progress](https://img.shields.io/badge/status-work%20in%20progress-f59e0b?style=for-the-badge)](#project-status)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E?style=for-the-badge&logo=ros)](https://docs.ros.org/en/jazzy/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?style=for-the-badge&logo=cplusplus)](https://isocpp.org/)

</div>

`fuzzy_mpc` is a real-time ROS 2 lateral controller developed for [BCN eMotorsport](https://bcnemotorsport.upc.edu/), the Formula Student Electric and Driverless team from Barcelona. It tracks a planner trajectory, predicts the vehicle response across a finite horizon, and solves a constrained control problem at every update.

> [!WARNING]
> **This project is a work in progress.** Interfaces, parameters, launch files, and controller behaviour may change without notice. It is not ready for safety-critical deployment.

## Why this project?

Classical vehicle models are fast and interpretable, but their accuracy can fade near the limits of handling. Purely learned models can capture nonlinear behaviour, but are often harder to constrain and reason about.

This project explores a middle ground: keep the predictive-control structure, hard limits, and fast quadratic-programming workflow of MPC, while allowing the vehicle dynamics to come from either a linear time-varying bicycle model or a data-driven ANFIS model.

## Highlights

- **Constrained MPC** for lateral trajectory tracking and steering control
- **Two model implementations**: physics-based LTV and learned ANFIS dynamics
- **Fast dense QP solving** with [HPIPM](https://github.com/giaf/hpipm) and [BLASFEO](https://github.com/giaf/blasfeo)
- **Warm-started predictions** using the previous state and control sequence
- **Steering angle, steering-rate, and yaw-moment constraints**
- **Online model-error evaluation** over a configurable rolling horizon
- **ROS 2 visualisation outputs** for the reference and predicted paths
- **Event-oriented configuration** for acceleration, skidpad, autocross, and trackdrive
- **Profiling and numerical snapshots** for controller analysis

## Project status

The core controller, solver integration, ROS interfaces, model implementations, and debugging tools are present. Current development is focused on:

- validating the controller across all Formula Student events;
- completing and benchmarking ANFIS model integration;
- consolidating event-specific launch configuration;
- improving tests, documentation, and failure handling;
- tuning for repeatable real-time performance on the target vehicle.

The controller remains under active development and validation within the team's simulation and vehicle-testing pipeline.

## Team integration

This repository is one component of BCN eMotorsport's autonomous-system pipeline. It depends on the team's state estimation, planning, message, and shared-library packages, and is not intended to operate as a standalone ROS 2 application.

The controller predicts six states — lateral position, lateral velocity, heading, yaw rate, steering angle, and steering rate — and optimises steering plus an optional yaw moment over the horizon. The LTV model is currently selected in `MPC::initialize()`; the ANFIS backend and trained YAML models are included for ongoing integration.

## ROS interface

The default topics are configured in [`config/params.yaml`](config/params.yaml).

| Direction | Topic | Type | Purpose |
|---|---|---|---|
| Subscribe | `/as/c/state` | `cat_msgs/msg/CarState` | Current vehicle state |
| Subscribe | `/as/c/locator` | `cat_msgs/msg/ObjectiveArrayCurv` | Planner trajectory and velocity reference |
| Publish | `/as/c/steering` | `cat_msgs/msg/CarCommands` | Steering command |
| Publish | `/as/c/mpc/model_error` | `std_msgs/msg/Float64` | Rolling prediction error |
| Publish | `/as/c/fuzzy_mpc/vis/predictedPath/path` | `nav_msgs/msg/Path` | Predicted trajectory |
| Publish | `/as/c/fuzzy_mpc/vis/referencePath` | `nav_msgs/msg/Path` | Local reference trajectory |

Topic names are parameters, so the node can be integrated without changing controller code.

## Configuration

The controller configuration is split into two layers:

- [`config/params.yaml`](config/params.yaml) contains sampling, horizon, constraints, scaling, vehicle data, diagnostics, and topic mappings.
- `config/dyn_*.yaml` contains event-specific MPC weights and latency settings.

The most important parameters to review are:

| Parameter | Meaning |
|---|---|
| `MPC.Ts` | Controller sampling period |
| `MPC.n_horizon` | Prediction horizon in samples |
| `MPC.n_evaluation` | Window used for model-error evaluation |
| `MPC.max_steering` | Steering-angle constraint |
| `MPC.max_steering_dot` | Steering-rate constraint |
| `MPC.q_*`, `MPC.p_*` | Stage and terminal state weights |
| `MPC.r_*`, `MPC.rd_*` | Control and control-rate weights |
| `Car.*` | Vehicle and tyre-model parameters |

The ANFIS model definitions and normalisation statistics live under [`models/`](models/). Keep `MPC.Ts` consistent with the sampling time used to train those models.

## Repository map

```text
fuzzy_mpc/
├── include/
│   ├── mpc.hpp              # MPC formulation and prediction pipeline
│   ├── anfis.hpp            # ANFIS inference implementation
│   ├── models/              # LTV and ANFIS model backends
│   └── utils/               # Solver, ROS, config, and geometry helpers
├── src/main.cpp              # ROS 2 node and control loop
├── config/                   # Base and event-specific parameters
├── launch/                   # ROS 2 launch entry points
├── models/                   # Trained ANFIS parameters
├── debug/                    # Snapshot and visualisation utilities
├── mtlb/                     # Model training and analysis work
├── docs/                     # Project documentation
└── external/                 # HPIPM and BLASFEO submodules
```

## Debugging and profiling

Set `save_debug: true` in the base configuration to export the MPC matrices and vectors as CSV snapshots. The scripts in [`debug/`](debug/) can record and visualise a complete MPC iteration. Runtime profiling can be toggled with the `profile` parameter and uses the profiler supplied by `as_lib`.

## Research background

A paper describing the research behind this controller will be published shortly.
