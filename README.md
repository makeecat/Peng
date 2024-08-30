# [![Peng](assets/Peng.svg)](https://github.com/makeecat/Peng)
![GitHub last commit](https://img.shields.io/github/last-commit/makeecat/peng)
![](https://img.shields.io/badge/Rust-1.80+-orange.svg)
[![License](https://img.shields.io/badge/license-MIT%2FApache-blue.svg)](https://github.com/makeecat/Peng#license)
[![Crates.io](https://img.shields.io/crates/v/peng_quad.svg)](https://crates.io/crates/peng_quad)
[![Downloads](https://img.shields.io/crates/d/peng_quad.svg)](https://crates.io/crates/peng_quad)
[![Docs](https://docs.rs/peng_quad/badge.svg)](https://docs.rs/peng_quad/latest/peng_quad/)
[![CI](https://github.com/makeecat/Peng/actions/workflows/CI.yml/badge.svg)](https://github.com/makeecat/Peng/actions/workflows/CI.yml)
[![dependency status](https://deps.rs/repo/github/makeecat/peng/status.svg)](https://deps.rs/repo/github/makeecat/peng)
## What is Peng
Peng is a minimal Rust-based quadrotor simulation pipeline. It includes a simulator, controller, and planner, providing a basic framework for simulating quadrotor dynamics and control.
![demo](assets/Peng_demo.gif)
## Getting Started

### Installation from Crates.io
```bash
cargo install rerun-cli
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
- [ ] MPC controller

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

## Line Count
I would like to thank [tokei](https://github.com/XAMPPRocky/tokei) for providing the line count statistics!
My goal is to keep the project minimal and easy to understand by keeping the code line count below 1500.
The markdown lines are used for generating the documentation.
```markdown
===============================================================================
 Language            Files        Lines         Code     Comments       Blanks
 SVG                     2          492          480            0           12
 TOML                    1           31           30            0            1
 YAML                    1           94           86            0            8
-------------------------------------------------------------------------------
 Markdown                4          273            0          207           66
 |- Markdown             1           17            0           17            0
 (Total)                            290            0          224           66
-------------------------------------------------------------------------------
 Rust                    3         1487         1438           13           36
 |- Markdown             1          300            0          300            0
 (Total)                           1787         1438          313           36
===============================================================================
 Total                  11         2377         2034          220          123
===============================================================================
 ```

## Star History

[![Star History Chart](https://api.star-history.com/svg?repos=makeecat/Peng&type=Date)](https://star-history.com/#makeecat/Peng&Date)
