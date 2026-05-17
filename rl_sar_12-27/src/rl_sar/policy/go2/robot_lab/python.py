import torch

# 读取保存的 depth tensor
depth_camera = torch.load("depth_camera.pth")

# 打印信息
print("Depth tensor type:", type(depth_camera))
print("Depth tensor shape:", depth_camera.shape)
print("Depth tensor device:", depth_camera.device)
print("Depth tensor dtype:", depth_camera.dtype)

# 如果你想看具体数据（可选，可能很大）
print("Depth tensor data (slice):", depth_camera[0, :5, :5])  # 只看前5x5

