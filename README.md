# [![Peng](https://raw.githubusercontent.com/makeecat/Peng/main/assets/Peng.svg)](https://github.com/makeecat/Peng)

[![License](https://img.shields.io/badge/license-MIT%2FApache-blue.svg)](https://github.com/makeecat/Peng#license)
[![Crates.io](https://img.shields.io/crates/v/peng_quad.svg)](https://crates.io/crates/peng_quad)
[![Downloads](https://img.shields.io/crates/d/peng_quad.svg)](https://crates.io/crates/peng_quad)
[![Docs](https://docs.rs/peng_quad/badge.svg)](https://docs.rs/peng_quad/latest/peng_quad/)
[![CI](https://github.com/makeecat/Peng/actions/workflows/CI.yml/badge.svg)](https://github.com/makeecat/Peng/actions/workflows/CI.yml)
[![dependency status](https://deps.rs/repo/github/makeecat/peng/status.svg)](https://deps.rs/repo/github/makeecat/peng)
[![Gitter](https://img.shields.io/gitter/room/peng/peng)](https://app.gitter.im/#/room/#peng:gitter.im)

## What is Peng

Peng is a minimal quadrotor autonomy framework in Rust. It includes a simulator, controller, and planner, providing a basic framework for simulating quadrotor dynamics and control.
![demo](https://raw.githubusercontent.com/makeecat/Peng/main/assets/Peng_demo.gif)

## Getting Started

### Installation from Crates.io

```bash
cargo install rerun-cli # ensure you installed rerun-cli>=0.18.2 by checking rerun --version
cargo install peng_quad
peng_quad config/quad.yaml
```

### Installation from Source

```bash
cargo install rerun-cli
git clone https://github.com/makeecat/Peng.git && cd Peng
cargo run --release config/quad.yaml
```

You can configure the simulation through config file, see [quad.yaml](config/quad.yaml) for example.

Please follow [rerun troubleshooting](https://rerun.io/docs/getting-started/troubleshooting) if you are using Linux or WSL2.

## Overview

### Quadrotor Simulator

Simulates realistic quadrotor dynamics with properties like position, velocity, orientation, angular velocity, mass, and inertia. Includes methods for updating dynamics with control inputs and simulating IMU readings and Depth map rendering.

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
- Waypoint Planner

### Obstacle Simulation

Simulates moving obstacles in the environment, with collision detection and avoidance capabilities based on potential field.

### Data Logging and Visualization

Logs comprehensive simulation data including quadrotor state, desired positions, IMU readings, and depth map rendering. Visualizes the simulation using the rerun library.

## Features

- Realistic quadrotor dynamics simulation
- IMU sensor simulation with configurable noise parameters
- Multiple trajectory planners for diverse flight patterns
- PID controller for position and attitude control
- Obstacle generation and avoidance
- Depth map rendering based on primitives
- Integration with rerun for real-time visualization

## TODO

- [ ] Environment Effect simulation such as wind field
- [ ] Add motor speed simulation
- [ ] Multi-quadrotor simulation
- [ ] MPC controller

## License

Peng is free, open source and permissively licensed!
Except where noted (below and/or in individual files), all code in this repository is dual-licensed under either:

- MIT License ([LICENSE-MIT](LICENSE-MIT) or [http://opensource.org/licenses/MIT](http://opensource.org/licenses/MIT))
- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or [http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0))
  at your option.

This means you can select the license you prefer!

## Why call it Peng?

Peng (traditional Chinese: 鵬; simplified Chinese: 鹏; pinyin: péng; Wade–Giles: p'eng) or Dapeng (大鵬) is a giant bird that transforms from a Kun (鯤; 鲲; kūn; k'un) giant fish in Chinese mythology.

The pipeline is designed to be minimal and for educational purpose.
We chose the name Peng because our pipeline is flexible and can transform to suit different needs, just like the mythical bird.

Reference: https://en.wikipedia.org/wiki/Peng_(mythology)

## Blog posts

- [Peng #1: Minimal quadrotor pipeline in Rust](https://yangrobotics.com/peng-1-minimal-quadrotor-pipeline-in-rust)
- [Peng #2: Error Handling, Configuration System and Obstacle Avoidance Planner](https://yangrobotics.com/peng-2-error-handling-configuration-system-and-obstacle-avoidance-planner)

## Citation

If you use this project in your research or work, please cite it as follows:

```bibtex
@software{peng_quad,
  author       = {Yang Zhou},
  title        = {Peng: A Minimal Quadrotor Autonomy Framework in Rust},
  year         = {2024},
  publisher    = {GitHub},
  journal      = {GitHub repository},
  howpublished = {\url{https://github.com/makeecat/peng}},
}
```
