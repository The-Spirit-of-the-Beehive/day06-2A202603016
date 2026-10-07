"""Bonus B1: So sánh RANSAC Plane Segmentation vs Fixed Height Thresholding (Z-cutoff).

Chạy từ gốc repo:
    python -m src.bonus_compare_ground --frame 000019
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

from starter.datasets import load_frame


def main():
    parser = argparse.ArgumentParser(description="Bonus B1: So sánh 2 phương pháp lọc mặt đất")
    parser.add_argument("--data-root", default="data/kitti_mini")
    parser.add_argument("--frame", default="000019")
    parser.add_argument("--z-ground-threshold", type=float, default=-1.4,
                        help="Ngưỡng z cố định để coi là mặt đất (mét)")
    args = parser.parse_args()

    fr = load_frame(args.data_root, args.frame)
    pts_raw = fr["points"][:, :3]

    # ROI filter + Voxel downsample
    in_roi = ((pts_raw[:, 0] >= 0.0) & (pts_raw[:, 0] <= 40.0) &
              (pts_raw[:, 1] >= -15.0) & (pts_raw[:, 1] <= 15.0) &
              (pts_raw[:, 2] >= -2.5) & (pts_raw[:, 2] <= 1.5))
    pts_roi = pts_raw[in_roi]
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts_roi)
    pcd_down = pcd.voxel_down_sample(voxel_size=0.1)
    pts = np.asarray(pcd_down.points)

    # 1. Thuật toán A: RANSAC Plane Fitting
    t0 = time.perf_counter()
    pcd_ransac = o3d.geometry.PointCloud()
    pcd_ransac.points = o3d.utility.Vector3dVector(pts)
    _, inliers_ransac = pcd_ransac.segment_plane(distance_threshold=0.2, ransac_n=3, num_iterations=800)
    obs_ransac = pcd_ransac.select_by_index(inliers_ransac, invert=True)
    lat_ransac = (time.perf_counter() - t0) * 1000

    labels_ransac = np.array(obs_ransac.cluster_dbscan(eps=0.5, min_points=10, print_progress=False))
    clusters_ransac = len([lbl for lbl in np.unique(labels_ransac) if lbl >= 0])

    # 2. Thuật toán B: Fixed Height Thresholding (Z < threshold là ground)
    t0 = time.perf_counter()
    ground_mask_z = pts[:, 2] <= args.z_ground_threshold
    obs_pts_z = pts[~ground_mask_z]
    lat_z = (time.perf_counter() - t0) * 1000

    pcd_z = o3d.geometry.PointCloud()
    pcd_z.points = o3d.utility.Vector3dVector(obs_pts_z)
    labels_z = np.array(pcd_z.cluster_dbscan(eps=0.5, min_points=10, print_progress=False))
    clusters_z = len([lbl for lbl in np.unique(labels_z) if lbl >= 0])

    print("=" * 65)
    print(f"KẾT QUẢ SO SÁNH 2 THUẬT TOÁN (FRAME {args.frame}):")
    print(f"{'Tiêu chí':<25} | {'RANSAC Plane':<18} | {'Fixed Z Cutoff':<18}")
    print("-" * 65)
    print(f"{'Thời gian xử lý (ms)':<25} | {lat_ransac:<18.2f} | {lat_z:<18.2f}")
    print(f"{'Số điểm mặt đất (Ground)':<25} | {len(inliers_ransac):<18} | {int(ground_mask_z.sum()):<18}")
    print(f"{'Số điểm vật cản (Obstacle)':<25} | {len(obs_ransac.points):<18} | {len(obs_pts_z):<18}")
    print(f"{'Số cụm phát hiện (Clusters)':<25} | {clusters_ransac:<18} | {clusters_z:<18}")
    print("=" * 65)

    # Lưu hình so sánh BEV
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    
    # Plot RANSAC
    axes[0].scatter(pts[inliers_ransac, 1], pts[inliers_ransac, 0], s=0.5, c="gray", alpha=0.3, label="Ground")
    axes[0].scatter(np.asarray(obs_ransac.points)[:, 1], np.asarray(obs_ransac.points)[:, 0], s=2, c="crimson", alpha=0.8, label="Obstacle")
    axes[0].set_title(f"Method 1: RANSAC Plane\nTime: {lat_ransac:.1f}ms | Clusters: {clusters_ransac}")
    axes[0].set_xlim(-15, 15); axes[0].set_ylim(0, 40); axes[0].legend(); axes[0].grid(True, linestyle="--", alpha=0.5)

    # Plot Fixed Z
    axes[1].scatter(pts[ground_mask_z, 1], pts[ground_mask_z, 0], s=0.5, c="gray", alpha=0.3, label="Ground")
    axes[1].scatter(obs_pts_z[:, 1], obs_pts_z[:, 0], s=2, c="blue", alpha=0.8, label="Obstacle")
    axes[1].set_title(f"Method 2: Fixed Height Threshold (z <= {args.z_ground_threshold}m)\nTime: {lat_z:.1f}ms | Clusters: {clusters_z}")
    axes[1].set_xlim(-15, 15); axes[1].set_ylim(0, 40); axes[1].legend(); axes[1].grid(True, linestyle="--", alpha=0.5)

    plt.tight_layout()
    out_file = Path("results/figures/bonus_compare_ground.png")
    out_file.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(str(out_file), dpi=150)
    plt.close()
    print(f"-> Đã lưu ảnh so sánh Bonus B1: {out_file}")


if __name__ == "__main__":
    main()