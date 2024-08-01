# Peng
Peng is a minimal Rust-based quadrotor simulation pipeline. It includes a simulator, controller, and planner, providing a basic framework for simulating quadrotor dynamics and control.
## Dependencies
nalgebra
rand
rerun
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
Licensed under the GPL-3.0 License. See the LICENSE file for details.

## Why call it Peng?
Peng (traditional Chinese: 鵬; simplified Chinese: 鹏; pinyin: péng; Wade–Giles: p'eng) or Dapeng (大鵬) is a giant bird that transforms from a Kun (鯤; 鲲; kūn; k'un) giant fish in Chinese mythology.
Reference: https://en.wikipedia.org/wiki/Peng_(mythology)

