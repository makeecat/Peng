# [![Peng](assets/Peng.svg)](https://github.com/makeecat/Peng)
[![License](https://img.shields.io/badge/license-MIT%2FApache-blue.svg)](https://github.com/makeecat/Peng#license)
[![Docs](https://github.com/makeecat/Peng/actions/workflows/docs.yml/badge.svg)](https://makeecat.github.io/Peng/)
[![CI](https://github.com/makeecat/Peng/actions/workflows/CI.yml/badge.svg)](https://github.com/makeecat/Peng/actions/workflows/CI)
## What is Peng
Peng is a minimal Rust-based quadrotor simulation pipeline. It includes a simulator, controller, and planner, providing a basic framework for simulating quadrotor dynamics and control.
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
Simulates dynamics with properties like position, velocity, orientation, and mass. Includes methods for updating dynamics with and without control inputs.
### PID Controller
Controls position and attitude with configurable gains for proportional, integral, and derivative terms.
### Planners
Includes a hover planner and a minimum jerk trajectory planner for generating movement commands.
### Data Logging
Logs simulation data for visualization using rerun.
## License
Peng is free, open source and permissively licensed!
Except where noted (below and/or in individual files), all code in this repository is dual-licensed under either:

* MIT License ([LICENSE-MIT](LICENSE-MIT) or [http://opensource.org/licenses/MIT](http://opensource.org/licenses/MIT))
* Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or [http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.   org/licenses/LICENSE-2.0))

at your option.
This means you can select the license you prefer!

## Why call it Peng?
Peng (traditional Chinese: 鵬; simplified Chinese: 鹏; pinyin: péng; Wade–Giles: p'eng) or Dapeng (大鵬) is a giant bird that transforms from a Kun (鯤; 鲲; kūn; k'un) giant fish in Chinese mythology.
Reference: <https://en.wikipedia.org/wiki/Peng_(mythology)>
