"""Vẽ biểu đồ phân tích thí nghiệm Sweep RANSAC.

Chạy từ gốc repo:
    python -m src.plot_obstacle_sweep
"""
from pathlib import Path
import matplotlib.pyplot as plt
import pandas as pd

csv_path = Path("results/obstacle_benchmark.csv")
if not csv_path.exists():
    raise FileNotFoundError("Chưa tìm thấy results/obstacle_benchmark.csv. Hãy chạy exp_obstacle_sweep trước!")

df = pd.read_csv(csv_path, dtype={"frame": str})

fig, axes = plt.subplots(1, 3, figsize=(16, 5))

# Plot 1: Số điểm vật cản (Obstacle Points)
ax1 = axes[0]
for frame, group in df.groupby("frame"):
    ax1.plot(group["dist_thresh_m"], group["obstacle_pts"], marker="o", linewidth=1.8, label=f"Frame {frame}")
ax1.set_title("Số điểm vật cản (Obstacle Points)", fontsize=11, fontweight="bold")
ax1.set_xlabel("RANSAC distance_threshold (m)")
ax1.set_ylabel("Số lượng điểm")
ax1.grid(True, linestyle="--", alpha=0.5)
ax1.legend()

# Plot 2: Số cụm chướng ngại vật (Clusters)
ax2 = axes[1]
for frame, group in df.groupby("frame"):
    ax2.plot(group["dist_thresh_m"], group["num_clusters"], marker="s", linewidth=1.8, label=f"Frame {frame}")
ax2.set_title("Số lượng cụm phát hiện (DBSCAN Clusters)", fontsize=11, fontweight="bold")
ax2.set_xlabel("RANSAC distance_threshold (m)")
ax2.set_ylabel("Số lượng cụm")
ax2.grid(True, linestyle="--", alpha=0.5)
ax2.legend()

# Plot 3: Khoảng cách vật cản gần nhất (Min Distance)
ax3 = axes[2]
for frame, group in df.groupby("frame"):
    ax3.plot(group["dist_thresh_m"], group["min_dist_m"], marker="^", linewidth=1.8, label=f"Frame {frame}")
ax3.set_title("Khoảng cách tới vật cản gần nhất (m)", fontsize=11, fontweight="bold")
ax3.set_xlabel("RANSAC distance_threshold (m)")
ax3.set_ylabel("Min Obstacle Distance (m)")
ax3.grid(True, linestyle="--", alpha=0.5)
ax3.legend()

plt.tight_layout()
out_fig = Path("results/figures/obstacle_sweep.png")
out_fig.parent.mkdir(parents=True, exist_ok=True)
plt.savefig(str(out_fig), dpi=150)
plt.close()
print(f"-> Đã lưu biểu đồ: {out_fig}")