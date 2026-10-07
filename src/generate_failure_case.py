"""CP4: Tạo ảnh phân tích Failure Case - Lỗi Under-segmentation trên frame 000019.

Chạy từ gốc repo:
    python -m src.generate_failure_case --frame 000019
"""
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

from starter.datasets import load_frame
from src.obstacle_detector import run_obstacle_pipeline


def main():
    frame_id = "000019"
    fr = load_frame("data/kitti_mini", frame_id)
    pts_raw = fr["points"][:, :3]

    # Chạy pipeline với cấu hình chuẩn
    res = run_obstacle_pipeline(pts_raw, voxel_size=0.1, distance_threshold=0.2, eps=0.5, min_points=10)

    # Tìm cụm khổng lồ bị dính chùm (cluster có chiều dài lớn nhất)
    giant_cluster = None
    max_len = 0.0
    for c in res["clusters"]:
        length_x = c["max_bound"][0] - c["min_bound"][0]
        if length_x > max_len:
            max_len = length_x
            giant_cluster = c

    fig, axes = plt.subplots(1, 2, figsize=(15, 7))

    # Subplot 1: Toàn cảnh BEV & đánh dấu Bounding Box khổng lồ
    ax1 = axes[0]
    ax1.scatter(res["pts_ground"][:, 1], res["pts_ground"][:, 0], s=0.5, c="lightgray", alpha=0.3, label="Ground")
    
    # Vẽ các cụm bình thường
    pts_obs = res["pts_obstacle"]
    labels = res["labels"]
    clustered = (labels >= 0) & (labels != giant_cluster["label"])
    ax1.scatter(pts_obs[clustered, 1], pts_obs[clustered, 0], s=2, c="steelblue", alpha=0.6, label="Normal Clusters")
    
    # Vẽ cụm bị lỗi dính chùm
    giant_pts = pts_obs[labels == giant_cluster["label"]]
    ax1.scatter(giant_pts[:, 1], giant_pts[:, 0], s=4, c="crimson", alpha=0.9, label=f"Failed Cluster (ID: {giant_cluster['label']})")
    
    # Bounding box lỗi
    y_min, x_min = giant_cluster["min_bound"][1], giant_cluster["min_bound"][0]
    y_max, x_max = giant_cluster["max_bound"][1], giant_cluster["max_bound"][0]
    rect = plt.Rectangle((y_min, x_min), y_max - y_min, x_max - x_min,
                         fill=False, edgecolor="red", linewidth=2.5, linestyle="-")
    ax1.add_patch(rect)
    ax1.text(y_min - 0.5, (x_min + x_max) / 2, f"FAIL: L = {max_len:.1f} m\nMerged Wall + Cars",
             color="red", fontweight="bold", fontsize=10, ha="right", va="center")

    ax1.set_title(f"Toàn cảnh BEV - Frame {frame_id}\nPhát hiện cụm bất thường kích thước {max_len:.1f}m", fontsize=11, fontweight="bold")
    ax1.set_xlabel("Y (Trái <-> Phải) [m]")
    ax1.set_ylabel("X (Phía trước) [m]")
    ax1.set_xlim(-15, 15)
    ax1.set_ylim(0, 40)
    ax1.legend(loc="upper right")
    ax1.grid(True, linestyle="--", alpha=0.5)

    # Subplot 2: Zoom cận cảnh vùng lỗi (Vỉa hè làm cầu nối dính các vật thể)
    ax2 = axes[1]
    zoom_mask = (giant_pts[:, 0] >= 0.0) & (giant_pts[:, 0] <= 30.0) & (giant_pts[:, 1] >= -14.0) & (giant_pts[:, 1] <= -4.0)
    ax2.scatter(giant_pts[zoom_mask, 1], giant_pts[zoom_mask, 0], s=6, c="crimson", alpha=0.8)
    
    ax2.annotate("Vat can 1: Xe do / Tuong truoc", xy=(-6.5, 5.0), xytext=(-12.0, 5.0),
                 arrowprops=dict(facecolor="black", shrink=0.05, width=1, headwidth=6), fontsize=9, fontweight="bold")
    ax2.annotate("Vat can 2: Xe do noi duoi", xy=(-6.2, 16.0), xytext=(-12.0, 16.0),
                 arrowprops=dict(facecolor="black", shrink=0.05, width=1, headwidth=6), fontsize=9, fontweight="bold")
    ax2.annotate("Cau noi (Curb/Bia le duong < 0.5m)", xy=(-6.0, 10.5), xytext=(-2.0, 10.5),
                 arrowprops=dict(facecolor="blue", shrink=0.05, width=1, headwidth=6), fontsize=9, color="blue", fontweight="bold")

    ax2.set_title("Zoom Cận Cảnh: Cơ chế dính chùm qua vỉa hè/lề đường\n(DBSCAN eps=0.5m không tách được khoảng cách hẹp)", fontsize=11, fontweight="bold")
    ax2.set_xlabel("Y (m)")
    ax2.set_ylabel("X (m)")
    ax2.set_xlim(-14, -2)
    ax2.set_ylim(2, 28)
    ax2.grid(True, linestyle="--", alpha=0.5)

    plt.tight_layout()
    out_file = Path("results/figures/fail_01_undersegmentation_cluster.png")
    out_file.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(str(out_file), dpi=150)
    plt.close()
    print(f"-> Đã tạo ảnh Failure Case thành công: {out_file}")


if __name__ == "__main__":
    main()