# Selection Examples（可选示例集）

## 概述

本仓库不存在名为 "Selection Examples" 的专属目录，但提供了一组**可供用户按需选择使用的示例工具**，分布在以下目录中：

| 目录 | 类型 | 说明 |
|------|------|------|
| [`test/`](../test/) | 测试程序（C++） | 验证各功能模块的独立测试工具 |
| [`tools/`](../tools/) | 转换工具（C++） | ONNX → TensorRT 模型转换 |
| [`scripts/`](../scripts/) | 脚本工具（Python） | PyTorch 模型导出与转换 |

这些示例各自独立、覆盖不同功能面，用户可以**按照部署流程或调试需求选择运行**。

---

## 示例详细说明

### 1. `test/test_trt_engine.cpp` — TensorRT 引擎功能验证

**用途**：验证 TensorRT 引擎是否能正常加载和推理，并测量推理性能。

**验证功能点**：
- CUDA 设备可用性检测
- TensorRT 引擎反序列化加载
- 输入/输出绑定信息打印（兼容 TensorRT 8.2–8.5+）
- 推理延迟与吞吐量测量（100 次预热 + 正式计时）
- 10 维动作输出的正确性

**适用场景**：首次在 Jetson 设备上部署，快速确认 GPU 和引擎工作正常。

**运行方法**：
```bash
cd build
./test_trt_engine ../model/model_fp32.engine
```

**预期输出**：
```
TensorRT引擎测试
GPU: NVIDIA Jetson Nano
平均推理时间: 0.150 ms (6666 FPS)
输出 (10维动作): [...]
```

---

### 2. `test/test_trt_engine_csv.cpp` — CSV 黄金数据对比测试

**用途**：将 TensorRT 引擎推理结果与仿真器输出的黄金数据逐步对比，量化误差。

**验证功能点**：
- TensorRT 引擎输出与 Python 仿真输出的数值一致性
- 多步（最多 100 步）逐帧推理误差统计（最大误差 / 平均误差）
- CSV 数据解析，支持完整的输入结构（`t0` 39维 + 私有隐变量 44维 + 扫描点 187维 + 历史 390维 + 动作 10维）

**适用场景**：模型部署后的精度回归测试，确保 Jetson 上的 TensorRT 输出与仿真训练环境一致。

**输入文件**：[`sim_inputs_outputs_100steps.csv`](../sim_inputs_outputs_100steps.csv)（仿真100步的输入输出黄金数据）

**CSV 列结构**：
| 列索引 | 维度 | 字段 |
|--------|------|------|
| 0–38 | 39 | `t0`：当前帧本体感知（角速度、欧拉角、指令、关节位置/速度/动作） |
| 39–82 | 44 | 私有隐变量（`priv_latent_*`） |
| 83–269 | 187 | 扫描点（`scan_dot_*`） |
| 270–659 | 390 | 历史帧（`t-10` 到 `t-1`，每帧 39 维） |
| 660–669 | 10 | 黄金动作输出（`act_0` 到 `act_9`） |

**运行方法**：
```bash
cd build
./test_trt_engine_csv ../model/model_fp32.engine ../sim_inputs_outputs_100steps.csv
```

**预期输出**（每步）：
```
========== 第 1 步 ==========
推理输出 (TensorRT): 0.081311 -0.012259 ...
仿真输出 (Golden):   0.081311 -0.012259 ...
最大误差: 0.000001
平均误差: 0.000000
```

---

### 3. `test/test_udp.cpp` — UDP 通信测试

**用途**：验证 Jetson 与 ODroid 之间的 UDP 通信链路是否正常，不涉及模型推理。

**验证功能点**：
- UDP socket 创建、绑定与数据收发
- `MsgRequest`（机器人状态）和 `MsgResponse`（电机指令）消息结构体的序列化/反序列化
- 收发包数量统计与丢包检测

**适用场景**：在运行主程序前，先单独验证网络连接是否正常。

**运行方法**：
```bash
cd build
./test_udp [IP] [端口]
# 示例：
./test_udp 192.168.5.159 10000
```

---

### 4. `test/test_motors.cpp` — 电机正弦波控制测试

**用途**：通过向所有关节发送正弦波位置指令，验证电机响应和完整数据流（观测输入 → 电机指令输出）。

**验证功能点**：
- 从 `robot.yaml` 加载初始姿态
- 正弦波生成：幅值 0.09 rad（约 5°），周期 10 秒
- 完整观测信息打印（39 维：角速度、欧拉角、关节位置/速度/力矩）
- 500 Hz 控制循环（2ms 周期）
- 收发包丢包率统计

**适用场景**：上电后验证所有关节电机是否响应正常，在运行 RL 策略前做安全性预检。

**运行方法**：
```bash
cd build
./test_motors --ip 192.168.5.159 --port 10000 --config ../robot.yaml
```

> ⚠️ **安全提示**：运行前请确保机器人处于安全悬挂或支撑状态，正弦波会使所有关节同步运动。

---

### 5. `test/test_init_pose.cpp` — 初始姿态测试

**用途**：上电后通过线性插值平滑将机器人从当前位置移动到标定的初始姿态，并保持不动。

**验证功能点**：
- 从 `robot.yaml` 读取 10 个关节的初始位置
- 线性插值平滑移动（默认 500 步，1 秒）
- 到达初始姿态后保持位置并持续输出反馈

**适用场景**：每次部署前的上电归零动作，确认机器人能安全到达初始站立姿态。

**运行方法**：
```bash
cd build
./test_init_pose --ip 192.168.5.159 --port 10000 --config ../robot.yaml --steps 500
```

---

### 6. `test/gamepad_calibration.cpp` — 手柄键码获取工具

**用途**：实时读取 Linux 输入设备事件，显示手柄摇杆轴值和按钮键码，用于手柄键码映射标定。

**验证功能点**：
- 读取 `/dev/input/event*` 设备的 `EV_ABS`（轴）和 `EV_KEY`（按钮）事件
- 实时显示轴值及归一化值（`value / 32768.0`）
- 自动列出可用输入设备

**适用场景**：配置手柄控制命令映射时，先运行此工具确认各摇杆和按钮对应的事件码。

**运行方法**：
```bash
cd build
./gamepad_calibration /dev/input/event2
```

---

### 7. `tools/onnx_to_trt.cpp` — ONNX 转 TensorRT 工具（C++ 版）

**用途**：在 Jetson 设备上将 ONNX 模型文件转换为 TensorRT `.engine` 文件。

**验证功能点**：
- ONNX 模型结构解析与打印
- 动态维度（batch）优化配置
- FP16 / FP32 精度选择
- 引擎序列化并保存

**适用场景**：在目标 Jetson 设备上本地生成 TensorRT 引擎（引擎与 GPU 架构绑定，必须在目标设备上生成）。

**运行方法**：
```bash
cd build
./onnx_to_trt ../model/model_fp32.onnx ../model/model_fp32.engine        # 默认 FP16
./onnx_to_trt ../model/model_fp32.onnx ../model/model_fp32.engine --fp32 # FP32
```

---

### 8. `scripts/convert_to_trt.py` — 模型转换脚本（Python 版）

**用途**：在 PC 或 Jetson 上完成 PyTorch → ONNX → TensorRT 的完整模型转换流程。

**验证功能点**：
- PyTorch JIT 模型加载与 ONNX 导出
- ONNX 模型结构验证
- TensorRT 引擎构建（FP16/FP32 可选）

**适用场景**：从 PyTorch 训练好的模型开始，完成完整部署准备链路。

**运行方法**：
```bash
cd scripts
# 仅导出 ONNX
python convert_to_trt.py model.pt model.onnx --to-onnx

# 仅转换为 TensorRT（需在 Jetson 上执行）
python convert_to_trt.py model.onnx model.engine --to-trt

# 完整流程 PT → ONNX → TRT（需在 Jetson 上执行）
python convert_to_trt.py model.pt model.engine --full
```

**依赖**：
```bash
pip install torch onnx onnxruntime tensorrt
```

---

### 9. `scripts/convert_to_onnx.py` — PyTorch → ONNX 导出脚本

**用途**：将自定义 PyTorch 模型导出为 ONNX 格式，并进行数值一致性验证。

**验证功能点**：
- 模型加载（state_dict 或整个模型两种方式均支持）
- ONNX 模型结构完整性检查（`onnx.checker`）
- PyTorch 输出与 ONNX Runtime 输出数值对比（atol=1e-5）

**适用场景**：将自定义网络结构的模型导出为 ONNX，为后续 TensorRT 转换做准备。

**运行方法**：
> 需要在 `convert_to_onnx.py` 中替换 `SimpleModel` 为你的实际模型类，并修改 `input_model_path` 和 `output_onnx_path` 路径。

```bash
cd scripts
python convert_to_onnx.py
```

---

## 推荐使用顺序

按照完整部署流程，建议按以下顺序使用这些示例：

```
第一步: 模型转换 (PC 端)
  └─ scripts/convert_to_onnx.py   (PyTorch → ONNX)

第二步: 引擎生成 (Jetson 端)
  └─ tools/onnx_to_trt             (ONNX → TRT Engine)
  └─ scripts/convert_to_trt.py    (一键完整转换)

第三步: 引擎验证 (Jetson 端)
  ├─ test/test_trt_engine          (性能测试)
  └─ test/test_trt_engine_csv      (精度回归测试)

第四步: 通信验证 (Jetson 端)
  └─ test/test_udp                 (UDP 链路测试)

第五步: 机器人预检 (Jetson 端，需连接 ODroid)
  ├─ test/test_init_pose           (初始姿态归零)
  └─ test/test_motors              (正弦波电机测试)

第六步: 手柄配置 (可选)
  └─ test/gamepad_calibration      (手柄键码标定)

第七步: 运行主程序
  └─ JetsonRLDeploy               (500Hz 实时推理控制)
```

---

## 相关文件路径

| 示例文件 | 构建产物 |
|----------|----------|
| [`test/test_trt_engine.cpp`](../test/test_trt_engine.cpp) | `build/test_trt_engine` |
| [`test/test_trt_engine_csv.cpp`](../test/test_trt_engine_csv.cpp) | `build/test_trt_engine_csv` |
| [`test/test_udp.cpp`](../test/test_udp.cpp) | `build/test_udp` |
| [`test/test_motors.cpp`](../test/test_motors.cpp) | `build/test_motors` |
| [`test/test_init_pose.cpp`](../test/test_init_pose.cpp) | `build/test_init_pose` |
| [`test/gamepad_calibration.cpp`](../test/gamepad_calibration.cpp) | `build/gamepad_calibration` |
| [`tools/onnx_to_trt.cpp`](../tools/onnx_to_trt.cpp) | `build/onnx_to_trt` |
| [`scripts/convert_to_trt.py`](../scripts/convert_to_trt.py) | — |
| [`scripts/convert_to_onnx.py`](../scripts/convert_to_onnx.py) | — |
| [`sim_inputs_outputs_100steps.csv`](../sim_inputs_outputs_100steps.csv) | 黄金数据（CSV 对比测试输入） |

> 完整构建方法详见 [README.md](../README.md#编译和部署)。
