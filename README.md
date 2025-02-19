# [![Peng](https://raw.githubusercontent.com/makeecat/Peng/main/assets/Peng.svg)](https://github.com/makeecat/Peng)

[![License](https://img.shields.io/badge/license-MIT%2FApache-blue.svg)](https://github.com/makeecat/Peng#license)
[![Crates.io](https://img.shields.io/crates/v/peng_quad.svg)](https://crates.io/crates/peng_quad)
[![Downloads](https://img.shields.io/crates/d/peng_quad.svg)](https://crates.io/crates/peng_quad)
[![Docs](https://docs.rs/peng_quad/badge.svg)](https://docs.rs/peng_quad/latest/peng_quad/)
[![CI](https://github.com/makeecat/Peng/actions/workflows/CI.yml/badge.svg)](https://github.com/makeecat/Peng/actions/workflows/CI.yml)
[![dependency status](https://deps.rs/repo/github/makeecat/peng/status.svg)](https://deps.rs/repo/github/makeecat/peng)
[![Discord](https://img.shields.io/discord/1324766515077185688?label=Discord&logo=discord)](https://discord.gg/NDsZXpGWPR)

## 🔍 Overview

Peng is a minimal quadrotor autonomy framework written in Rust that provides real-time dynamics simulation, trajectory planning, and control with modern visualization capabilities.

[![rerun demo](https://raw.githubusercontent.com/makeecat/Peng/main/assets/Peng_demo.gif)](https://rerun.io/viewer/version/0.22.0/?url=https%3A%2F%2Fyangrobotics.com%2Ffiles%2Fpeng_v0.5.4.rrd)

## 🎯 Key Features

- 🚁 **Real-time Simulation**
  - High-fidelity quadrotor dynamics with configurable parameters
  - IMU and depth sensor simulation
  - Optional RK4 integration for accurate dynamics
- 🎮 **Advanced Control**
  - PID control for position and attitude with tunable gains
  - Integral windup prevention
  - Support for different control frequencies
- 📍 **Rich Trajectory Planning**
  - Minimum jerk line trajectory planner
  - Lissajous curve planner
  - Circular trajectory planner
  - Obstacle avoidance planner
  - Minimum snap waypoint navigation planner
  - Quadratic Polynomial waypoint navigation planner
  - Landing planner
- 📊 **Visualization & Debug**
  - Real-time 3D visualization via rerun.io
  - Depth map rendering
  - State telemetry logging
  - Configurable logging frequencies
- ⚡ **Performance**
  - Memory-safe and Efficient Rust implementation
  - Multi-threaded depth rendering

## 🚀 Getting Started

### Prerequisites

- [Rust](https://www.rust-lang.org/tools/install)
- [rerun-cli](https://rerun.io/docs/getting-started/installing-viewer)

### Installation from Crates.io

```bash
cargo install rerun-cli
cargo install peng_quad
peng_quad config/quad.yaml
```

### Installation from Source

```bash
cargo install rerun-cli
git clone https://github.com/makeecat/Peng.git
cd Peng
cargo run --release config/quad.yaml
```

## ⚙️ Configuration

- You can configure the simulation through config file, see [quad.yaml](config/quad.yaml) for example.
- Configure simulation parameters such as mass, inertia, and control gains.
- Configure control parameters such as PID gains.
- Configure trajectory planner parameters such as waypoints, obstacles, and trajectory type.
- Configure visualization parameters such as camera intrinsics and depth rendering.

## 🔧 Rerun Troubleshooting

If you encountered any issue with the rerun:

1. Verify rerun-cli version matches rerun version in [Cargo.toml](https://github.com/makeecat/Peng/blob/main/Cargo.toml):

```bash
rerun --version
```

2. For Linux/WSL2 users, consult the [rerun troubleshooting](https://rerun.io/docs/getting-started/troubleshooting).

## 🗺️ Roadmap

- [ ] Wind field and environmental effects
- [ ] Motor dynamics simulation
- [ ] Multi-quadrotor simulation
- [ ] Model Predictive Control (MPC)

## 🤝 Contributing

We welcome contributions of all kinds! Please check out the [Contributing Guidelines](CONTRIBUTING.md) for more details.

## 📄 License

Peng is free, open source and permissively licensed!
Except where noted (below and/or in individual files), all code in this repository is dual-licensed under either:

- MIT License ([LICENSE-MIT](LICENSE-MIT) or [http://opensource.org/licenses/MIT](http://opensource.org/licenses/MIT))
- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or [http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0))
  at your option.

This means you can select the license you prefer!

## 📝 Blog Posts

- [Peng #1: Minimal quadrotor pipeline in Rust](https://yangrobotics.com/peng-1-minimal-quadrotor-pipeline-in-rust)
- [Peng #2: Error Handling, Configuration System and Obstacle Avoidance Planner](https://yangrobotics.com/peng-2-error-handling-configuration-system-and-obstacle-avoidance-planner)
- [Peng #3: Optimization of Depth Rendering and RK4-based Dynamics Update](https://yangrobotics.com/peng-3-optimization-of-depth-rendering-and-rk4-based-dynamics-update)

## 📚 Citation

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
