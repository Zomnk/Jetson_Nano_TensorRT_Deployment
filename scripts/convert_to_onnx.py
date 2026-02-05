import torch
import torch.nn as nn
import torch.onnx
import onnx
import onnxruntime
import numpy as np


# ==========================================
# 1. 定义或导入你的模型结构
# ==========================================
# 注意：你需要在这里定义你的模型类，或者从其他文件 import 进来
# 这里的 'SimpleModel' 只是一个示例占位符
class SimpleModel(nn.Module):
    def __init__(self):
        super(SimpleModel, self).__init__()
        self.conv1 = nn.Conv2d(3, 16, 3, 1, 1)
        self.relu = nn.ReLU()
        self.fc = nn.Linear(16 * 224 * 224, 10)

    def forward(self, x):
        x = self.conv1(x)
        x = self.relu(x)
        x = x.view(x.size(0), -1)
        x = self.fc(x)
        return x


def convert():
    # ==========================================
    # 2. 配置路径和参数
    # ==========================================
    # Windows 路径建议使用 r'' 防止转义，或者使用双斜杠
    input_model_path = r'D:\Jetson_TensorRT_Deploy\project\model\model.pt'  # 你的 .pt 文件路径
    output_onnx_path = r'D:\Jetson_TensorRT_Deploy\project\model\model.onnx'  # 输出的 .onnx 文件名

    # 设定运行设备 (通常转换时用 CPU 更稳定)
    device = torch.device('cpu')

    # ==========================================
    # 3. 加载模型
    # ==========================================
    print(f"正在加载模型: {input_model_path}...")

    # 初始化模型结构
    model = SimpleModel()
    model.to(device)

    # 加载权重 (根据你的保存方式，可能需要调整 map_location)
    # 如果你的 .pt 文件只包含 state_dict (推荐方式):
    try:
        model.load_state_dict(torch.load(input_model_path, map_location=device))
    except Exception as e:
        print("直接加载 state_dict 失败，尝试加载整个模型...")
        # 如果你的 .pt 文件包含整个模型结构:
        model = torch.load(input_model_path, map_location=device)

    # 务必设置为评估模式 (影响 Dropout, BatchNorm 等层)
    model.eval()

    # ==========================================
    # 4. 创建虚拟输入 (Dummy Input)
    # ==========================================
    # 输入形状必须与模型训练时的输入一致
    # 格式通常为: (Batch_Size, Channels, Height, Width)
    batch_size = 1
    channels = 3
    height = 224
    width = 224
    dummy_input = torch.randn(batch_size, channels, height, width, device=device)

    # ==========================================
    # 5. 执行转换 (Export)
    # ==========================================
    print("正在转换为 ONNX...")

    torch.onnx.export(
        model,  # 运行的模型
        dummy_input,  # 模型输入 (元组或张量)
        output_onnx_path,  # 保存路径
        export_params=True,  # 是否在模型文件中存储训练好的参数权重
        opset_version=11,  # ONNX 版本 (11 或 13 兼容性较好)
        do_constant_folding=True,  # 是否执行常量折叠优化
        input_names=['input'],  # 输入节点的名称 (方便后续调用)
        output_names=['output'],  # 输出节点的名称
        # 动态轴 (如果你的 batch_size 是可变的，必须设置这个)
        dynamic_axes={
            'input': {0: 'batch_size'},  # 0 轴是动态的
            'output': {0: 'batch_size'}
        }
    )
    print(f"转换完成！模型已保存至: {output_onnx_path}")

    # ==========================================
    # 6. 验证模型 (可选但推荐)
    # ==========================================
    verify_onnx_model(output_onnx_path, dummy_input, model)


def verify_onnx_model(onnx_path, dummy_input, torch_model):
    print("\n开始验证 ONNX 模型...")

    # 1. 检查模型结构是否完整
    onnx_model = onnx.load(onnx_path)
    onnx.checker.check_model(onnx_model)
    print("ONNX 模型结构检查通过。")

    # 2. 比较 PyTorch 和 ONNX Runtime 的输出数值
    # 获取 PyTorch 输出
    with torch.no_grad():
        torch_out = torch_model(dummy_input)

    # 获取 ONNX Runtime 输出
    ort_session = onnxruntime.InferenceSession(onnx_path)

    # 将 tensor 转换为 numpy
    def to_numpy(tensor):
        return tensor.detach().cpu().numpy() if tensor.requires_grad else tensor.cpu().numpy()

    # 计算 ONNX 输出
    ort_inputs = {ort_session.get_inputs()[0].name: to_numpy(dummy_input)}
    ort_outs = ort_session.run(None, ort_inputs)

    # 比较结果 (允许极小的误差)
    np.testing.assert_allclose(to_numpy(torch_out), ort_outs[0], rtol=1e-03, atol=1e-05)
    print("数值验证通过！PyTorch 输出与 ONNX 输出一致。")


if __name__ == '__main__':
    # 为了演示，我们先创建一个假的 .pt 文件
    # 在实际使用中，请删除这两行，直接使用你已有的文件
    # dummy_model = SimpleModel()
    # torch.save(dummy_model.state_dict(), "model.pt")

    # 运行转换
    convert()