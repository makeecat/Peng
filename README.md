# [![Peng](assets/Peng.svg)](https://github.com/makeecat/Peng)
[![License](https://img.shields.io/badge/license-MIT%2FApache-blue.svg)](https://github.com/makeecat/Peng#license)
[![Docs](https://github.com/makeecat/Peng/actions/workflows/docs.yml/badge.svg)](https://makeecat.github.io/Peng/)
[![CI](https://github.com/makeecat/Peng/actions/workflows/CI.yml/badge.svg)](https://github.com/makeecat/Peng/actions/workflows/CI.yml)
## What is Peng
Peng is a minimal Rust-based quadrotor simulation pipeline. It includes a simulator, controller, and planner, providing a basic framework for simulating quadrotor dynamics and control.
![demo](assets/Peng_demo.gif)
## Getting Started
terminal 1
```
cargo install rerun-cli
rerun
```
terminal 2
```
cargo run --release
```
## Overview

### Quadrotor Simulator
Simulates realistic quadrotor dynamics with properties like position, velocity, orientation, angular velocity, mass, and inertia. Includes methods for updating dynamics with control inputs and simulating IMU readings.

### PID Controller
Controls position and attitude with configurable gains for proportional, integral, and derivative terms. Handles both position and attitude control.

### Trajectory Planners
Includes multiple planners:
- Hover Planner
- Minimum Jerk Line Planner
- Lissajous Curve Planner
- Circular Trajectory Planner
- Landing Planner
- Obstacle Avoidance Planner

### Obstacle Simulation
Simulates moving obstacles in the environment, with collision detection and avoidance capabilities.

### Data Logging and Visualization
Logs comprehensive simulation data including quadrotor state, desired positions, IMU readings, and obstacle positions. Visualizes the simulation using the rerun library.

## Features

- Realistic quadrotor dynamics simulation
- IMU sensor simulation with configurable noise parameters
- Multiple trajectory planners for diverse flight patterns
- PID controller for position and attitude control
- Obstacle generation and avoidance
- Integration with rerun for real-time visualization

## License

Peng is free, open source and permissively licensed!
Except where noted (below and/or in individual files), all code in this repository is dual-licensed under either:
* MIT License ([LICENSE-MIT](LICENSE-MIT) or [http://opensource.org/licenses/MIT](http://opensource.org/licenses/MIT))
* Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or [http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0))
at your option.

This means you can select the license you prefer!

## Why call it Peng?

Peng (traditional Chinese: 鵬; simplified Chinese: 鹏; pinyin: péng; Wade–Giles: p'eng) or Dapeng (大鵬) is a giant bird that transforms from a Kun (鯤; 鲲; kūn; k'un) giant fish in Chinese mythology.

The pipeline is designed to be minimal and for educational purpose.
We chose the name Peng because our pipeline is flexible and can transform to suit different needs, just like the mythical bird.

Reference: https://en.wikipedia.org/wiki/Peng_(mythology)
