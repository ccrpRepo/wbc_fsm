# WBC_Deploy 控制器

基于强化学习和动作跟踪的人形机器人全身控制部署系统。

[English](README.md) | 中文

## 功能特性

- **状态机控制**：包含 Passive（阻尼保护）、Loco（行走）和 WBC（全身控制）等多种 FSM 状态
- **动作跟踪**：实时跟踪重定向到 Unitree G1 人形机器人的 LAFAN1 动作数据集
- **ONNX Runtime**：使用 ONNX 模型进行快速推理
- **可配置**：基于 JSON 的配置系统，便于模式切换和参数调整

## 环境要求

- CMake >= 3.14
- C++17 编译器
- CUDA
- 依赖库：
  - unitree_sdk2
  - **ONNX Runtime 1.22.0**（见下方安装说明）
  - Eigen3
  - nlohmann_json >= 3.7.3
  - Boost

### 安装 ONNX Runtime

下载并解压 ONNX Runtime 1.22.0 到 `controller/` 目录：

**x64 平台（仿真）：**
```bash
cd controller/
wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-x64-1.22.0.tgz
tar -xzf onnxruntime-linux-x64-1.22.0.tgz
```

**aarch64 平台（真实机器人）：**
```bash
cd controller/
wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-aarch64-1.22.0.tgz
tar -xzf onnxruntime-linux-aarch64-1.22.0.tgz
```

## 编译

```bash
mkdir -p build
cd build
cmake ..
make -j4
```

## 配置

配置文件位于 `config/` 目录：
- `wbc.json`：WBC 状态配置
- `loco.json`：运动状态配置
- `fixedpose.json`：固定关节状态配置
- `passive.json`：阻尼状态配置

配置示例（`wbc.json`）：
```json
{
    "model_path": "model/wbc/lafan1_0128_1.onnx",
    "folder_path": "motion_data/lafan1/dance12_binary",
    "enter_idx": 0,
    "pause_idx": 350,
    "safe_projgravity_threshold": 0.5
}
```

## 运行

### 在 Mujoco 仿真中部署

1. 按照 https://github.com/unitreerobotics/unitree_mujoco 的说明安装 Unitree Mujoco

2. 在 `CMakeLists.txt` 中设置 ONNX Runtime 路径：
   ```cmake
   set(ONNXRUNTIME_ROOT ${PROJECT_SOURCE_DIR}/onnxruntime-linux-x64-1.22.0)
   ```

3. 在 `controller/src/interface/IOSDK.cpp` 中配置网络接口：
   ```cpp
   ChannelFactory::Instance()->Init(1, "lo"); // lo 用于仿真
   ```

4. 编译项目：
   ```bash
   cd build
   cmake ..
   make -j4
   ```

5. 修改unitree_mujoco/config.yaml配置，并启动仿真：
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

6. 运行控制器（在新终端中）：
   ```bash
   cd controller/build
   ./wbc_fsm
   ```

### 在真实机器人上部署

1. 将本项目复制到 Unitree G1 机器人 PC2 电脑的 `/home/unitree` 目录下

2. 在 `CMakeLists.txt` 中设置 ONNX Runtime 路径：
   ```cmake
   set(ONNXRUNTIME_ROOT ${PROJECT_SOURCE_DIR}/onnxruntime-linux-aarch64-1.22.0)
   ```

3. 在 `controller/src/interface/IOSDK.cpp` 中配置网络接口：
   ```cpp
   ChannelFactory::Instance()->Init(0, "eth0"); // eth0 用于真实机器人
   ```

4. 编译项目：
   ```bash
   cd build
   cmake ..
   make -j4
   ```

5. 运行控制器：
   ```bash
   ./wbc_fsm
   ```

## 项目结构

```
controller/
├── config/           # 配置文件
├── include/          # 头文件
│   ├── common/      # 通用工具
│   ├── control/     # 控制组件
│   ├── FSM/         # 状态机状态
│   ├── interface/   # 硬件接口
│   └── message/     # 消息定义
├── src/             # 源文件
│   ├── main.cpp
│   ├── control/
│   ├── FSM/
│   └── interface/
├── model/           # ONNX 模型
├── motion_data/     # 动作参考数据
└── CMakeLists.txt
```

## 控制说明

### 操作指令

- **R1**：恢复 WBC 状态（暂停时）
- **R2**：暂停 WBC 状态
- **R2+A**：切换到 Loco 模式
- **L2+B**：切换到 Passive 模式
- **SELECT**：退出程序

### 操作步骤

1. 运行程序后，机器人处于**阻尼保护模式**
2. 按 **START** 键进入位控模式
3. 将机器人悬吊起来（在仿真中默认启用 `enable_elastic_band`，按键盘数字键 **9** 可以松开绑带，再次按下可重新悬吊，数字键 **8** 下放，数字键 **7** 上拉）
4. 按遥控器上的 **R2+A** 进入 Loco(AMP) Mode，此时松开吊绳
5. 按遥控器上的 **R2+A** 进入 Loco(RL) Mode
6. 按遥控器上的 **R1+Up** 进入全身控制模式（WBC Mode）
   - 按 **R2** 可以暂停动作
   - 按 **R1** 可以继续动作

## 许可证

本项目基于 Unitree Robotics SDK2 框架开发。

原始框架：Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. 保留所有权利。

修改和扩展：[ccrpRepo / ZSTU Robotics] © 2026

## 致谢

- 基于 Unitree Robotics SDK2 开发
- 动作数据来自 LAFAN1 数据集
- 使用 ONNX Runtime 进行模型推理
