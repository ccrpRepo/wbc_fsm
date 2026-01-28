# WBC_Deploy Controller

Whole-Body Control deployment system for humanoid robots using reinforcement learning and motion mimicking.

## Features

- **State Machine Control**: Multiple FSM states including Passive, Loco (locomotion), and WBC (whole-body control)
- **Motion Mimicking**: Real-time motion tracking using UMT (Universal Motion Transformer)
- **ONNX Runtime**: Fast inference with ONNX models
- **Configurable**: JSON-based configuration for easy parameter tuning

## Prerequisites

- CMake >= 3.14
- C++17 compiler
- CUDA (optional, for GPU acceleration)
- Required libraries:
  - unitree_sdk2
  - ONNX Runtime 1.22.0
  - Eigen3
  - nlohmann_json >= 3.7.3
  - OpenSSL
  - Boost (optional)

## Building

```bash
mkdir -p build
cd build
cmake ..
make -j4
```

## Configuration

Configuration files are located in `config/`:
- `umt.json`: WBC state configuration
- `loco.json`: Locomotion state configuration
- `config_rl.json`: RL state configuration

Example configuration (`umt.json`):
```json
{
    "model_path": "model/umt/lafan1_0128_1.onnx",
    "folder_path": "motion_data/lafan1/dance12_binary",
    "enter_idx": 0,
    "pause_idx": 350,
    "safe_projgravity_threshold": 0.5
}
```

## Running

```bash
cd build
./wbc_fsm
```

## Project Structure

```
controller/
├── config/           # Configuration files
├── include/          # Header files
│   ├── common/      # Common utilities
│   ├── control/     # Control components
│   ├── FSM/         # State machine states
│   ├── interface/   # Hardware interfaces
│   └── message/     # Message definitions
├── src/             # Source files
│   ├── main.cpp
│   ├── control/
│   ├── FSM/
│   └── interface/
├── model/           # ONNX models
├── motion_data/     # Motion reference data
└── CMakeLists.txt
```

## Controls

- **R1**: Resume WBC state (when paused)
- **R2**: Pause WBC state
- **R2+A**: Switch to Loco mode
- **L2+B**: Switch to Passive mode
- **SELECT**: Exit program

## License

Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.

## Acknowledgments

Based on Unitree Robotics SDK and control framework.
