# tsne_visualize.py —— 无颜色/单点版
import torch
import numpy as np
from sklearn.manifold import TSNE
from sklearn.decomposition import PCA
import matplotlib.pyplot as plt

# ---------------------------------------
# 1. 加载 latent
# ---------------------------------------
latent_path = "latents/z_tm_batch.pt"
tensor_model = torch.jit.load(latent_path).cpu()#.numpy()      # [N, latent_dim]
z = list(tensor_model.parameters())[0]
print("final z shape:", z)

# ---------------------------------------
# 2. PCA 降维
# ---------------------------------------
n_samples, latent_dim = z.shape
pca_dim = min(50, n_samples, latent_dim)
pca = PCA(n_components=pca_dim)
z_pca = pca.fit_transform(z)

# ---------------------------------------
# 3. t-SNE
# ---------------------------------------
perplexity = min(30, max(5, n_samples // 3))
tsne = TSNE(n_components=2,
            perplexity=perplexity,
            learning_rate="auto",
            init="pca")
z_2d = tsne.fit_transform(z_pca)

# ---------------------------------------
# 4. 绘图（无颜色）
# ---------------------------------------
plt.figure(figsize=(6, 6))
plt.scatter(z_2d[:, 0], z_2d[:, 1], s=15, c='k')   # 全黑小点
plt.title("Latent t-SNE (no label)")
plt.tight_layout()
plt.show()
