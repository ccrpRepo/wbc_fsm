# WBC_Deploy Controller

Whole-Body Control deployment system for humanoid robots using reinforcement learning and motion tracking.

English | [中文](README_zh.md)

## Features

- **State Machine Control**: Multiple FSM states including Passive, Loco (locomotion), and WBC (whole-body control)
- **Motion Tracking**: Real-time tracking of LAFAN1 motion dataset retargeted for Unitree G1 humanoid robots
- **ONNX Runtime**: Fast inference with ONNX models
- **Configurable**: JSON-based configuration for easy parameter tuning

## Prerequisites

- CMake >= 3.14
- C++17 compiler
- CUDA
- Required libraries:
  - unitree_sdk2
  - **ONNX Runtime 1.22.0** (see installation below)
  - Eigen3
  - nlohmann_json >= 3.7.3
  - Boost

### Installing ONNX Runtime

Download and extract ONNX Runtime 1.22.0 to the `controller/` directory:

**For x64 (Simulation):**
```bash
cd controller/
wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-x64-1.22.0.tgz
tar -xzf onnxruntime-linux-x64-1.22.0.tgz
```

**For aarch64 (Real Robot):**
```bash
cd controller/
wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-aarch64-1.22.0.tgz
tar -xzf onnxruntime-linux-aarch64-1.22.0.tgz
```

## Building

```bash
mkdir -p build
cd build
cmake ..
make -j4
```

## Configuration

Configuration files are located in `config/`:
- `wbc.json`: WBC state configuration
- `loco.json`: Locomotion state configuration
- `fixedpose.json`: FixedStand state configuration
- `passive.json`: Passive state configuration

Example configuration (`wbc.json`):
```json
{
    "model_path": "model/wbc/lafan1_0128_1.onnx",
    "folder_path": "motion_data/lafan1/dance12_binary",
    "enter_idx": 0,
    "pause_idx": 350,
    "safe_projgravity_threshold": 0.5
}
```

## Running

### Deploy on Mujoco Simulation

1. Install Unitree Mujoco following the instructions at https://github.com/unitreerobotics/unitree_mujoco

2. Set the ONNX Runtime path in `CMakeLists.txt`:
   ```cmake
   set(ONNXRUNTIME_ROOT ${PROJECT_SOURCE_DIR}/onnxruntime-linux-x64-1.22.0)
   ```

3. Configure the network interface in `controller/src/interface/IOSDK.cpp`:
   ```cpp
   ChannelFactory::Instance()->Init(1, "lo"); // lo for simulation
   ```

4. Build the project:
   ```bash
   cd build
   cmake ..
   make -j4
   ```

5. Edit unitree_mujoco/config.yaml and Start the simulation:
    ```yaml
    robot: "g1"  # Robot name, "go2", "b2", "b2w", "h1", "go2w", "g1"
    robot_scene: "scene_29dof.xml" # Robot scene, /unitree_robots/[robot]/scene.xml 
    domain_id: 1  # Domain id
    interface: "lo" # Interface 
    use_joystick: 1 # Simulate Unitree WirelessController using a gamepad
    joystick_type: "xbox" # support "xbox" and "switch" gamepad layout
    joystick_device: "/dev/input/js0" # Device path
    joystick_bits: 16 # Some game controllers may only have 8-bit accuracy
    print_scene_information: 1 # Print link, joint and sensors information of robot
    enable_elastic_band: 1 # Virtual spring band, used for lifting h1
    ```

   ```bash
   cd simulate/build
   ./unitree_mujoco
   ```

6. Run the controller (in a new terminal):
   ```bash
   cd controller/build
   ./wbc_fsm
   ```

### Deploy on Real Robot

1. Copy this project to `/home/unitree` on the Unitree G1 robot's PC2 computer

2. Set the ONNX Runtime path in `CMakeLists.txt`:
   ```cmake
   set(ONNXRUNTIME_ROOT ${PROJECT_SOURCE_DIR}/onnxruntime-linux-aarch64-1.22.0)
   ```

3. Configure the network interface in `controller/src/interface/IOSDK.cpp`:
   ```cpp
   ChannelFactory::Instance()->Init(0, "eth0"); // eth0 for real robot
   ```

4. Build the project:
   ```bash
   cd build
   cmake ..
   make -j4
   ```

5. Run the controller:
   ```bash
   ./wbc_fsm
   ```

## Controls

### Controller Commands

- **R1**: Resume WBC state (when paused)
- **R2**: Pause WBC state (at specified frame)
- **L2**: Pause WBC state (at current frame)
- **R2+A**: Switch to Loco mode
- **L2+B**: Switch to Passive mode
- **SELECT**: Exit program

### Operation Procedure

1. After starting the program, the robot enters **Damping Protection Mode**
2. Press **START** to enter position control mode
3. Suspend the robot (In simulation, `enable_elastic_band` is enabled by default. Press keyboard **9** to release the band, press again to re-suspend. Press **8** to lower, **7** to raise)
4. Press **R2+A** on the controller to enter Loco(AMP) Mode, then release the suspension band
   - Press **R2+up** to enter high speed mode(running)
   - Press **R2+down** to enter low speed mode(walking)
5. Press **R2+B** on the controller to enter Loco(RL) Mode
6. Press **R1+Up** on the controller to enter WBC (Whole-Body Control) Mode
   - Press **R2** to pause the motion
   - Press **R1** to resume the motion

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

## License

This project is based on Unitree Robotics SDK2 framework.

Original framework: Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.

Modified and extended by [ccrpRepo / ZSTU Robotics] © 2026

## Acknowledgments

- Based on Unitree Robotics SDK2
- Motion data from LAFAN1 dataset
- ONNX Runtime for model inference
