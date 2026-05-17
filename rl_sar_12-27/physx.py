import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# =========================
# 路径配置
# =========================
csv_path = "./obs_log.csv"   # 改成你的路径

# =========================
# 读取 CSV
# =========================
df = pd.read_csv(csv_path, header=None)

# 只取前 53 维
df = df.iloc[:, :53]

num_steps, obs_dim = df.shape
print(f"Loaded obs shape: {df.shape}")

# =========================
# 列名定义（与你 C++ obs 完全一致）
# =========================
col_names = []

for i in range(3):
    col_names.append(f"ang_vel[{i}]")

for i in range(2):
    col_names.append(f"rpy[{i}]")

col_names += [
    "placeholder",
    "delta_yaw",
    "delta_next_yaw",
    "cmd_1",
    "cmd_2",
    "cmd_0",
    "terrain_0",
    "terrain_1",
]

for i in range(12):
    col_names.append(f"dof_pos[{i}]")

for i in range(12):
    col_names.append(f"dof_vel[{i}]")

for i in range(12):
    col_names.append(f"last_action[{i}]")

for i in range(4):
    col_names.append(f"foot_contact[{i}]")

assert len(col_names) == obs_dim

# =========================
# 1️⃣ 逐维 obs 曲线（53 维）
# =========================
for i in range(obs_dim):
    plt.figure(figsize=(10, 4))
    plt.plot(df.iloc[:, i], alpha=0.8)
    plt.title(col_names[i])
    plt.xlabel("Step")
    plt.ylabel("Value")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

# =========================
# 2️⃣ last_action（12 维）
# =========================
for i in range(12):
    plt.figure(figsize=(10, 4))
    plt.plot(df.iloc[:, 37 + i], alpha=0.8)
    plt.title(f"last_action[{i}]")
    plt.xlabel("Step")
    plt.ylabel("Action")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

# =========================
# 3️⃣ Δaction L2 范数（最关键）
# =========================
action = df.iloc[:, 37:49].to_numpy()
delta_action = np.linalg.norm(action[1:] - action[:-1], axis=1)

plt.figure(figsize=(10, 4))
plt.plot(delta_action, alpha=0.9)
plt.title("||Δ last_action||")
plt.xlabel("Step")
plt.ylabel("L2 Norm")
plt.grid(True)
plt.tight_layout()
plt.show()

# =========================
# 4️⃣ 各关节 Δaction
# =========================
for i in range(12):
    da = action[1:, i] - action[:-1, i]

    plt.figure(figsize=(10, 4))
    plt.plot(da, alpha=0.8)
    plt.title(f"Δ last_action[{i}]")
    plt.xlabel("Step")
    plt.ylabel("Δ Action")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

# =========================
# 5️⃣ foot_contact（4 维）
# =========================
for i in range(4):
    plt.figure(figsize=(10, 4))
    plt.plot(df.iloc[:, 49 + i], alpha=0.8)
    plt.title(f"foot_contact[{i}]")
    plt.xlabel("Step")
    plt.ylabel("Contact")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

# =========================
# 6️⃣ foot_contact 抖动强度
# =========================
contact = df.iloc[:, 49:53].to_numpy()
d_contact = np.abs(contact[1:] - contact[:-1])

plt.figure(figsize=(10, 4))
plt.plot(d_contact.mean(axis=1), alpha=0.9)
plt.title("Foot contact jitter strength")
plt.xlabel("Step")
plt.ylabel("Mean |Δ|")
plt.grid(True)
plt.tight_layout()
plt.show()

