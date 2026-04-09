# Jetson RL 部署项目 (TensorRT版)

本项目用于在 Jetson 平台上使用 TensorRT 部署强化学习策略，实现双足机器人的实时控制。通过 UDP 与 ODroid 控制器通信，实现 500Hz 的实时推理和电机控制。

## 项目概述

**核心功能**：
- 🚀 **高性能推理**：TensorRT 引擎实现 0.15ms 推理延迟（6666 FPS）
- 🤖 **实时控制**：500Hz 控制循环，支持双足机器人运动
- 📊 **数据记录**：自动记录推理数据并导出为 CSV 文件
- 🎮 **手柄支持**：支持游戏手柄输入（通过 `/dev/input/js0` 接口）
- ⚙️ **灵活配置**：YAML 格式的机器人标定文件

**系统架构**：
```
STM32 → ODroid → [UDP] → Jetson (TensorRT推理) → ODroid → STM32
```

## 项目结构

```
project/
├── CMakeLists.txt                    # CMake 构建配置
├── README.md                         # 本文档
├── CLAUDE.md                         # Claude Code 开发指南
├── robot.yaml                        # 机器人标定配置文件
├── include/
│   ├── types.h                       # 消息结构体定义 (MsgRequest/MsgResponse)
│   ├── communication.h               # UDP 通信类头文件
│   └── trt_inference.h               # TensorRT 推理类头文件
├── src/
│   ├── main.cpp                      # 主程序入口 (500Hz 控制循环 + CSV导出)
│   ├── communication.cpp             # UDP 通信实现
│   ├── trt_inference.cpp             # TensorRT 推理实现
│   └── calibration_tool.cpp          # 关节标定工具
├── test/
│   ├── gamepad_calibration.cpp       # 手柄键码获取工具 (新增)
│   ├── test_udp.cpp                  # UDP通信测试
│   ├── test_motors.cpp               # 电机控制测试
│   ├── test_trt_engine.cpp           # TensorRT引擎测试
│   └── test_trt_engine_csv.cpp       # TensorRT引擎CSV对比测试
├── tools/
│   └── onnx_to_trt.cpp               # ONNX 转 TensorRT 工具
└── scripts/
    └── convert_to_trt.py             # 模型转换脚本
```

## 技术栈

| 组件 | 版本 | 说明 |
|------|------|------|
| **设备** | Jetson Nano B01 | ARM64 架构，4GB 内存 |
| **系统** | Ubuntu 18.04.6 LTS | 官方支持的 Jetson 系统 |
| **CUDA** | 10.2+ | GPU 计算平台 |
| **TensorRT** | 8.2.1.8+ | NVIDIA 推理优化引擎 |
| **GCC** | 7.5.0+ | C++ 编译器 |
| **CMake** | 3.14+ | 构建系统 |
| **C++** | C++14 | 编程语言标准 |

## 主要功能模块

### 1. 通信层 (Communication)
**文件**: `include/communication.h`, `src/communication.cpp`

- UDP 套接字管理和数据收发
- 消息结构体定义 (`MsgRequest`, `MsgResponse`)
- 非阻塞接收和超时处理
- 与 ODroid 控制器的双向通信

**关键数据结构**:
- `MsgRequest` (216 字节): 机器人状态 (39维观测)
- `MsgResponse` (120 字节): 电机指令 (10维动作)

### 2. 推理引擎 (TensorRT Inference)
**文件**: `include/trt_inference.h`, `src/trt_inference.cpp`

- TensorRT 引擎加载和管理
- 观测向量构建 (39维)
- 推理执行和输出处理
- 动作滤波 (指数移动平均)
- 电机指令缩放和限制

**推理流程**:
```
观测 (39D) → TensorRT 推理 → 动作 (10D) → 滤波 → 缩放 → 电机指令
```

### 3. 主控制循环 (Main Control Loop)
**文件**: `src/main.cpp`

- 500Hz 实时控制循环
- YAML 标定文件加载
- 推理数据记录和 CSV 导出
- 优雅关闭处理 (Ctrl+C)
- 自动故障恢复

**关键特性**:
- 推理失败或 NaN 检测时自动回到初始姿态
- 每次推理记录时间戳和电机指令
- 程序退出时自动导出 `inference_data.csv`

### 4. 标定工具 (Calibration Tool)
**文件**: `src/calibration_tool.cpp`

- 交互式关节标定
- YAML 格式配置文件生成
- 单关节或全关节标定模式

### 5. 手柄输入工具 (Gamepad Calibration) - 新增
**文件**: `test/gamepad_calibration.cpp`

- 读取 `/dev/input/js0` joystick 设备事件
- 实时显示摇杆轴值和按钮键码
- 支持映射后的速度命令显示
- 非阻塞事件读取
- 映射方案 A：左摇杆前后→`vx`，左摇杆左右→`vy`，右摇杆左右→`yaw_rate`
- 速度限制：`vx/vy` 最大 `0.2 m/s`，`yaw_rate` 最大 `0.4 rad/s`
- `LT` 按钮用于恢复到 `init_pose`

## 数据流说明

### 系统架构
```
STM32 (传感器/电机)
  ↓
ODroid (控制器)
  ↓ UDP
Jetson (TensorRT 推理)
  ↓ UDP
ODroid (控制器)
  ↓
STM32 (执行电机指令)
```

### 输入观测 (39维)

| 索引 | 维度 | 说明 | 单位 |
|------|------|------|------|
| 0-2 | 3 | 角速度 (wx, wy, wz) | rad/s |
| 3-5 | 3 | 欧拉角 (roll, pitch, yaw) | rad |
| 6-8 | 3 | 控制指令 (vx, vy, yaw_rate) | m/s, rad/s |
| 9-18 | 10 | 关节位置偏差 (q - init_pos) | rad |
| 19-28 | 10 | 关节速度 (dq) | rad/s |
| 29-38 | 10 | 上次动作 (action) | - |

### 输出动作 (10维)

10 个关节的目标位置:
- 0-4: 左腿 (yaw, roll, pitch, knee, ankle)
- 5-9: 右腿 (yaw, roll, pitch, knee, ankle)

**单位**: 弧度 (rad)

## 编译和部署

### 编译步骤

```bash
cd project
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

**生成的可执行文件**:
- `JetsonRLDeploy` - 主推理程序
- `calibration_tool` - 关节标定工具
- `gamepad_calibration` - 手柄键码获取工具
- `test_udp` - UDP 通信测试
- `test_motors` - 电机控制测试
- `test_trt_engine` - TensorRT 引擎测试
- `onnx_to_trt` - ONNX 转 TensorRT 工具

### 运行主程序

```bash
./JetsonRLDeploy ../model.engine --config ../robot.yaml --ip 192.168.5.159 --port 10000
```

**命令行选项**:
- `<engine_path>`: TensorRT 引擎文件路径 (必需)
- `--ip <IP>`: ODroid IP 地址 (默认: 192.168.5.159)
- `--port <N>`: UDP 端口 (默认: 10000)
- `--config <FILE>`: 标定文件路径 (默认: ../robot.yaml)

### 数据导出

程序退出时自动导出推理数据:
```
inference_data.csv
```

**CSV 格式**:
```
timestamp(s),action_0,action_1,...,action_9
0.002000,0.5,0.3,...
0.004000,0.51,0.31,...
```

## 性能指标

| 指标 | 值 |
|------|-----|
| 推理延迟 | 0.15 ms |
| 吞吐量 | 6666 FPS |
| 控制频率 | 500 Hz |
| 相比 LibTorch 加速 | 20-30x |

## 使用流程

### 第一步：模型转换

在 Jetson 上将 PyTorch 模型转换为 TensorRT 引擎:

```bash
cd scripts
python convert_to_trt.py ../model.pt ../model.engine --full
```

**注意**: TensorRT 引擎与 GPU 架构绑定，必须在目标 Jetson 设备上生成！

### 第二步：测试 TensorRT 引擎

```bash
cd build
./test_trt_engine ../model.engine
```

### 第三步：标定机器人

```bash
./calibration_tool
```

### 第四步：运行主程序

```bash
./JetsonRLDeploy ../model.engine --config ../robot.yaml
```

## 测试指南

### 1. UDP 通信测试

```bash
./test_udp [IP] [端口]
```

### 2. 电机控制测试

```bash
./test_motors --ip 192.168.5.159 --config ../robot.yaml
```

### 3. TensorRT 引擎测试

```bash
./test_trt_engine ../model.engine
```

### 4. 手柄键码获取

```bash
./gamepad_calibration /dev/input/js0
```

## 常见问题

### 1. TensorRT 找不到

确保 TensorRT 已安装，或设置路径:
```bash
cmake .. -DTENSORRT_ROOT=/usr/local/tensorrt
```

### 2. 引擎加载失败

- TensorRT 引擎与 GPU 架构绑定，必须在目标设备上生成
- 检查 TensorRT 版本是否匹配

### 3. CUDA 内存不足

- 减小 batch size
- 使用 FP16 精度

### 4. 推理结果异常

- 检查模型输入输出维度是否正确 (39 → 10)
- 验证 ONNX 导出是否正确

### 5. 标定文件加载失败

- 检查 `robot.yaml` 文件格式
- 确保包含 10 个标定值 (左腿 5 个 + 右腿 5 个)
- 使用 `calibration_tool` 重新生成标定文件

## Selection Examples（可选示例集）

本项目提供了一组可按需选择使用的示例工具，覆盖从模型转换到机器人预检的完整部署链路：

| 示例 | 可执行文件 | 功能说明 |
|------|-----------|---------|
| `test/test_trt_engine.cpp` | `test_trt_engine` | TensorRT 引擎加载与推理性能测试 |
| `test/test_trt_engine_csv.cpp` | `test_trt_engine_csv` | 与仿真黄金数据逐步对比，精度回归测试 |
| `test/test_udp.cpp` | `test_udp` | Jetson ↔ ODroid UDP 通信链路验证 |
| `test/test_motors.cpp` | `test_motors` | 正弦波电机控制测试，验证关节响应 |
| `test/test_init_pose.cpp` | `test_init_pose` | 线性插值平滑归零到初始姿态 |
| `test/gamepad_calibration.cpp` | `gamepad_calibration` | 手柄轴值与按钮键码实时获取 |
| `tools/onnx_to_trt.cpp` | `onnx_to_trt` | ONNX → TensorRT 引擎转换（C++ 版） |
| `scripts/convert_to_trt.py` | — | PyTorch → ONNX → TRT 完整转换脚本 |
| `scripts/convert_to_onnx.py` | — | PyTorch → ONNX 导出与数值验证脚本 |

详细说明（用途、场景、运行命令、输入输出格式）请参见：
👉 **[docs/selection_examples.md](docs/selection_examples.md)**

## 开发指南

详见 [CLAUDE.md](CLAUDE.md) 了解项目架构、常用命令和开发建议。

## 许可证

MIT License
