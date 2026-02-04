#!/usr/bin/env python3
"""
PyTorch模型转TensorRT引擎工具

使用方法:
    python convert_to_trt.py model.pt model.onnx --to-onnx
    python convert_to_trt.py model.onnx model.engine --to-trt
    python convert_to_trt.py model.pt model.engine --full

依赖:
    pip install torch onnx onnxruntime tensorrt
"""

import argparse
import os

def export_to_onnx(pt_path, onnx_path, input_dim=39, output_dim=10):
    """将PyTorch模型导出为ONNX格式"""
    import torch

    print(f"加载PyTorch模型: {pt_path}")
    model = torch.jit.load(pt_path)
    model.eval()

    # 创建示例输入
    dummy_input = torch.randn(1, input_dim)

    print(f"导出ONNX: {onnx_path}")
    torch.onnx.export(
        model,
        dummy_input,
        onnx_path,
        input_names=['input'],
        output_names=['output'],
        dynamic_axes={
            'input': {0: 'batch'},
            'output': {0: 'batch'}
        },
        opset_version=11
    )

    print("ONNX导出成功!")

    # 验证ONNX
    import onnx
    onnx_model = onnx.load(onnx_path)
    onnx.checker.check_model(onnx_model)
    print("ONNX模型验证通过!")

def convert_to_tensorrt(onnx_path, engine_path, fp16=True):
    """将ONNX模型转换为TensorRT引擎"""
    import tensorrt as trt

    TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

    print(f"加载ONNX: {onnx_path}")

    builder = trt.Builder(TRT_LOGGER)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser = trt.OnnxParser(network, TRT_LOGGER)

    with open(onnx_path, 'rb') as f:
        if not parser.parse(f.read()):
            for i in range(parser.num_errors):
                print(f"ONNX解析错误: {parser.get_error(i)}")
            return False

    print("ONNX解析成功!")

    # 配置构建器
    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 30)  # 1GB

    if fp16 and builder.platform_has_fast_fp16:
        print("启用FP16精度")
        config.set_flag(trt.BuilderFlag.FP16)

    # 构建引擎
    print("构建TensorRT引擎 (可能需要几分钟)...")
    serialized_engine = builder.build_serialized_network(network, config)

    if serialized_engine is None:
        print("引擎构建失败!")
        return False

    # 保存引擎
    print(f"保存引擎: {engine_path}")
    with open(engine_path, 'wb') as f:
        f.write(serialized_engine)

    print(f"TensorRT引擎构建成功! 大小: {os.path.getsize(engine_path) / 1024:.1f} KB")
    return True

def main():
    parser = argparse.ArgumentParser(description='PyTorch模型转TensorRT引擎')
    parser.add_argument('input', help='输入文件 (.pt 或 .onnx)')
    parser.add_argument('output', help='输出文件 (.onnx 或 .engine)')
    parser.add_argument('--to-onnx', action='store_true', help='仅转换为ONNX')
    parser.add_argument('--to-trt', action='store_true', help='仅转换为TensorRT')
    parser.add_argument('--full', action='store_true', help='完整转换 (PT -> ONNX -> TRT)')
    parser.add_argument('--fp32', action='store_true', help='使用FP32精度 (默认FP16)')
    parser.add_argument('--input-dim', type=int, default=39, help='输入维度')

    args = parser.parse_args()

    if args.to_onnx:
        export_to_onnx(args.input, args.output, args.input_dim)
    elif args.to_trt:
        convert_to_tensorrt(args.input, args.output, fp16=not args.fp32)
    elif args.full:
        # 完整转换
        onnx_path = args.input.replace('.pt', '.onnx')
        export_to_onnx(args.input, onnx_path, args.input_dim)
        convert_to_tensorrt(onnx_path, args.output, fp16=not args.fp32)
    else:
        # 自动检测
        if args.input.endswith('.pt'):
            export_to_onnx(args.input, args.output, args.input_dim)
        elif args.input.endswith('.onnx'):
            convert_to_tensorrt(args.input, args.output, fp16=not args.fp32)
        else:
            print("无法识别输入文件类型，请使用 --to-onnx 或 --to-trt 指定")

if __name__ == '__main__':
    main()
