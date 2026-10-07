"""Pipeline phát hiện vật cản cho Topic D: Voxel -> RANSAC -> DBSCAN -> BEV visualizer.

Chạy từ gốc repo:
    python -m src.obstacle_detector --data-root data/kitti_mini --frame 000019
"""
from __future__ import annotations

import argparse
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

from starter.datasets import load_frame


def run_obstacle_pipeline(points_xyz: np.ndarray,
                          voxel_size: float = 0.1,
                          distance_threshold: float = 0.2,
                          eps: float = 0.5,
                          min_points: int = 10,
                          roi_x: tuple[float, float] = (0.0, 40.0),
                          roi_y: tuple[float, float] = (-15.0, 15.0),
                          roi_z: tuple[float, float] = (-2.5, 1.5)) -> dict:
    """Thực hiện lọc ROI, Voxel downsample, RANSAC ground removal, và DBSCAN clustering."""
    # 1. Lọc điểm NaN / vô hạn và ROI (vùng quan tâm phía trước robot)
    valid = np.isfinite(points_xyz).all(axis=1)
    pts = points_xyz[valid]
    
    in_roi = ((pts[:, 0] >= roi_x[0]) & (pts[:, 0] <= roi_x[1]) &
              (pts[:, 1] >= roi_y[0]) & (pts[:, 1] <= roi_y[1]) &
              (pts[:, 2] >= roi_z[0]) & (pts[:, 2] <= roi_z[1]))
    pts = pts[in_roi]
    
    # 2. Đưa vào Open3D PointCloud & Voxel Downsample
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    if voxel_size > 0:
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    pts_down = np.asarray(pcd.points)

    if len(pts_down) < 10:
        return {"num_clusters": 0, "min_dist": float("inf"), "clusters": []}

    # 3. Lọc mặt đất bằng RANSAC plane segmentation
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=3,
        num_iterations=1000
    )
    ground_pcd = pcd.select_by_index(inliers)
    obstacle_pcd = pcd.select_by_index(inliers, invert=True)
    
    pts_ground = np.asarray(ground_pcd.points)
    pts_obstacle = np.asarray(obstacle_pcd.points)

    # 4. Gom cụm vật cản bằng DBSCAN
    num_clusters = 0
    labels = np.array([])
    min_dist = float("inf")
    clusters_info = []

    if len(pts_obstacle) > min_points:
        labels = np.array(obstacle_pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
        unique_labels = [lbl for lbl in np.unique(labels) if lbl >= 0]
        num_clusters = len(unique_labels)
        
        for lbl in unique_labels:
            c_pts = pts_obstacle[labels == lbl]
            c_dist = np.min(np.linalg.norm(c_pts[:, :2], axis=1))
            if c_dist < min_dist:
                min_dist = c_dist
            clusters_info.append({
                "label": lbl,
                "num_pts": len(c_pts),
                "min_dist": c_dist,
                "center": c_pts.mean(axis=0),
                "min_bound": c_pts.min(axis=0),
                "max_bound": c_pts.max(axis=0)
            })

    return {
        "raw_count": len(points_xyz),
        "roi_count": len(pts),
        "down_count": len(pts_down),
        "ground_count": len(pts_ground),
        "obstacle_count": len(pts_obstacle),
        "num_clusters": num_clusters,
        "min_dist": min_dist if min_dist != float("inf") else 0.0,
        "pts_ground": pts_ground,
        "pts_obstacle": pts_obstacle,
        "labels": labels,
        "clusters": clusters_info,
    }


def visualize_bev(result: dict, frame_id: str, out_path: Path):
    """Vẽ Bird's Eye View (BEV) thể hiện mặt đất và các cluster vật cản."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 7))
    
    # Subplot 1: Tách Ground vs Obstacle
    ax1 = axes[0]
    if len(result["pts_ground"]) > 0:
        ax1.scatter(result["pts_ground"][:, 1], result["pts_ground"][:, 0], s=0.5, c="gray", alpha=0.3, label="Ground")
    if len(result["pts_obstacle"]) > 0:
        ax1.scatter(result["pts_obstacle"][:, 1], result["pts_obstacle"][:, 0], s=1.5, c="red", alpha=0.8, label="Obstacle")
    ax1.set_title(f"Ground Removal (RANSAC) - Frame {frame_id}\nGround: {result['ground_count']} | Obstacle: {result['obstacle_count']}")
    ax1.set_xlabel("Y (Trái <-> Phải) [m]")
    ax1.set_ylabel("X (Phía trước) [m]")
    ax1.set_xlim(-15, 15)
    ax1.set_ylim(0, 40)
    ax1.legend(loc="upper right")
    ax1.grid(True, linestyle="--", alpha=0.5)

    # Subplot 2: Gom cụm DBSCAN
    ax2 = axes[1]
    pts_obs = result["pts_obstacle"]
    labels = result["labels"]
    
    if len(pts_obs) > 0 and len(labels) == len(pts_obs):
        # Nhiễu (label = -1)
        noise_mask = (labels == -1)
        ax2.scatter(pts_obs[noise_mask, 1], pts_obs[noise_mask, 0], s=1, c="black", alpha=0.2, label="Noise")
        
        # Các cluster
        clustered_mask = ~noise_mask
        if np.any(clustered_mask):
            scatter = ax2.scatter(pts_obs[clustered_mask, 1], pts_obs[clustered_mask, 0],
                                  s=3, c=labels[clustered_mask], cmap="tab20", alpha=0.9)
            
        # Vẽ bounding box 2D BEV cho từng cluster
        for c in result["clusters"]:
            y_min, x_min = c["min_bound"][1], c["min_bound"][0]
            y_max, x_max = c["max_bound"][1], c["max_bound"][0]
            rect = plt.Rectangle((y_min, x_min), y_max - y_min, x_max - x_min,
                                 fill=False, edgecolor="blue", linewidth=1.2, linestyle="--")
            ax2.add_patch(rect)
            
    ax2.set_title(f"DBSCAN Clustering - Clusters: {result['num_clusters']}\nMin Obstacle Dist: {result['min_dist']:.2f} m")
    ax2.set_xlabel("Y (Trái <-> Phải) [m]")
    ax2.set_ylabel("X (Phía trước) [m]")
    ax2.set_xlim(-15, 15)
    ax2.set_ylim(0, 40)
    ax2.grid(True, linestyle="--", alpha=0.5)

    plt.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(str(out_path), dpi=150)
    plt.close()
    print(f"-> Đã lưu ảnh BEV: {out_path}")


def main():
    parser = argparse.ArgumentParser(description="Phát hiện vật cản bằng RANSAC và DBSCAN")
    parser.add_argument("--data-root", default="data/kitti_mini")
    parser.add_argument("--frame", default="000019", help="Frame ID KITTI (vd: 000019)")
    parser.add_argument("--voxel-size", type=float, default=0.1)
    parser.add_argument("--dist-thresh", type=float, default=0.2)
    parser.add_argument("--eps", type=float, default=0.5)
    parser.add_argument("--min-points", type=int, default=10)
    parser.add_argument("--out-dir", default="results/figures")
    args = parser.parse_args()

    fr = load_frame(args.data_root, args.frame)
    points = fr["points"][:, :3]

    res = run_obstacle_pipeline(
        points,
        voxel_size=args.voxel_size,
        distance_threshold=args.dist_thresh,
        eps=args.eps,
        min_points=args.min_points
    )

    print(f"Frame: {args.frame} | Raw: {res['raw_count']} | Downsampled: {res['down_count']} "
          f"| Clusters: {res['num_clusters']} | Min Dist: {res['min_dist']:.2f}m")

    out_file = Path(args.out_dir) / f"demo_obstacle_{args.frame}.png"
    visualize_bev(res, args.frame, out_file)


if __name__ == "__main__":
    main()