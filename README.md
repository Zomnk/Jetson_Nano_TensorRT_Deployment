# Jetson RL 部署项目 (TensorRT版)

本项目用于在 Jetson 平台上使用 TensorRT 部署强化学习策略，实现双足机器人的实时控制。

## 项目结构

```
project/
├── CMakeLists.txt              # CMake 构建配置
├── README.md                   # 本文档
├── robot.yaml                  # 机器人标定配置文件
├── include/
│   ├── types.h                 # 消息结构体定义
│   ├── communication.h         # UDP 通信类头文件
│   └── trt_inference.h         # TensorRT 推理类头文件
├── src/
│   ├── main.cpp                # 主程序入口
│   ├── communication.cpp       # UDP 通信实现
│   ├── trt_inference.cpp       # TensorRT 推理实现
│   └── calibration_tool.cpp    # 关节标定工具
├── test/
│   ├── test_udp.cpp            # UDP通信测试
│   ├── test_motors.cpp         # 电机控制测试
│   └── test_trt_engine.cpp     # TensorRT引擎测试
└── scripts/
    └── convert_to_trt.py       # 模型转换脚本
```

## 开发环境

| 组件 | 版本 |
|------|------|
| 设备 | Jetson Nano B01 |
| 系统 | Ubuntu 18.04.6 LTS |
| CUDA | 10.2 |
| TensorRT | 8.2.1.8 |
| GCC | 7.5.0 |
| CMake | 3.14+ |

## 系统要求

- Jetson 平台 (Nano/Xavier/Orin)
- CUDA 10.2+
- TensorRT 8.2+
- CMake 3.14+
- GCC 7.5+

## 模型转换

在部署前，需要将 PyTorch 模型转换为 TensorRT 引擎。

### 安装依赖

```bash
pip install torch onnx tensorrt
```

### 转换步骤

```bash
cd project/scripts

# 方式1: 完整转换 (推荐)
python convert_to_trt.py model.pt model.engine --full

# 方式2: 分步转换
python convert_to_trt.py model.pt model.onnx --to-onnx
python convert_to_trt.py model.onnx model.engine --to-trt

# 使用FP32精度 (如果FP16精度不够)
python convert_to_trt.py model.pt model.engine --full --fp32
```

**注意**: TensorRT引擎与GPU架构绑定，必须在目标Jetson设备上生成！

## 编译步骤

```bash
cd project
mkdir build && cd build
cmake ..
make -j$(nproc)
```

编译完成后会生成以下可执行文件：
- `JetsonRLDeploy` - 主推理程序 (TensorRT)
- `calibration_tool` - 关节标定工具
- `test_udp` - UDP通信测试
- `test_motors` - 电机控制测试
- `test_trt_engine` - TensorRT引擎测试

## 使用流程

**重要：首次使用必须先进行标定！**

### 第一步：转换模型

在Jetson上将PyTorch模型转换为TensorRT引擎：

```bash
cd scripts
python convert_to_trt.py ../model.pt ../model.engine --full
```

### 第二步：测试TensorRT引擎

```bash
cd build
./test_trt_engine ../model.engine
```

输出示例：
```
CUDA设备数: 1
GPU: NVIDIA Tegra X1
引擎加载成功!
平均推理时间: 0.15 ms (6666 FPS)
测试通过!
```

### 第三步：标定机器人

```bash
./calibration_tool
```

标定工具选项：
```
--joint <ID>    只标定指定关节 (0-9)
--output <FILE> 输出文件路径 (默认: ../robot.yaml)
--ip <IP>       ODroid IP地址
```

### 第四步：运行主程序

```bash
./JetsonRLDeploy ../model.engine --config ../robot.yaml
```

选项：
```
--ip <IP>       ODroid IP地址 (默认: 192.168.5.159)
--port <N>      UDP端口 (默认: 10000)
--config <FILE> 标定文件路径 (默认: ../robot.yaml)
```

## 测试指南

### 1. UDP通信测试

```bash
./test_udp [IP] [端口]
```

### 2. 电机控制测试

```bash
./test_motors --ip 192.168.5.159 --config ../robot.yaml
```

### 3. TensorRT引擎测试

```bash
./test_trt_engine ../model.engine
```

测试内容：
- 检测CUDA设备
- 加载TensorRT引擎
- 打印绑定信息 (输入/输出维度)
- 执行推理并计时
- 打印输出结果

## 性能对比

| 框架 | 推理时间 | 吞吐量 |
|------|----------|--------|
| LibTorch (FP32) | ~5ms | ~200 FPS |
| LibTorch (FP16) | ~3ms | ~333 FPS |
| TensorRT (FP16) | ~0.15ms | ~6666 FPS |

TensorRT 相比 LibTorch 可提升 **20-30倍** 推理速度！

## 数据流说明

```
STM32 -> ODroid -> [UDP] -> Jetson (TensorRT推理) -> ODroid -> STM32
```

### 输入观测 (39维)

| 索引 | 维度 | 说明 |
|------|------|------|
| 0-2 | 3 | 角速度 (rad/s) |
| 3-5 | 3 | 欧拉角 (rad) |
| 6-8 | 3 | 控制指令 (vx, vy, yaw_rate) |
| 9-18 | 10 | 关节位置偏差 |
| 19-28 | 10 | 关节速度 |
| 29-38 | 10 | 上次动作 |

### 输出动作 (10维)

10个关节的目标位置 (左腿5个 + 右腿5个)

## 常见问题

### 1. TensorRT 找不到

确保 TensorRT 已安装，或设置路径：
```bash
cmake .. -DTENSORRT_ROOT=/usr/local/tensorrt
```

### 2. 引擎加载失败

- TensorRT引擎与GPU架构绑定，必须在目标设备上生成
- 检查TensorRT版本是否匹配

### 3. CUDA 内存不足

- 减小batch size
- 使用FP16精度

### 4. 推理结果异常

- 检查模型输入输出维度是否正确 (39 -> 10)
- 验证ONNX导出是否正确

## 许可证

MIT License
