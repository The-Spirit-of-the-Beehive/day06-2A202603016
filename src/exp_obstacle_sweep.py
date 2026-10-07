"""Topic D: Benchmark ảnh hưởng của distance_threshold trong RANSAC đến phát hiện vật cản.

Đo: số cụm (clusters), số điểm vật cản, khoảng cách gần nhất (min_dist), và Latency p50/p95 (20 lần).
Chạy từ gốc repo:
    python -m src.exp_obstacle_sweep --data-root data/kitti_mini --frames 000019 000011 000008
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
import time
import numpy as np
import open3d as o3d

from starter.datasets import load_frame


def run_pipeline_single(pts_down: np.ndarray, distance_threshold: float,
                        eps: float = 0.5, min_points: int = 10) -> tuple[int, int, int, float]:
    """Thực hiện RANSAC + DBSCAN trên đám mây điểm đã voxelize."""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts_down)
    
    # 1. RANSAC plane segmentation
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=3,
        num_iterations=800
    )
    ground_count = len(inliers)
    obstacle_pcd = pcd.select_by_index(inliers, invert=True)
    pts_obs = np.asarray(obstacle_pcd.points)
    obstacle_count = len(pts_obs)
    
    # 2. DBSCAN clustering
    num_clusters = 0
    min_dist = 0.0
    if obstacle_count >= min_points:
        labels = np.array(obstacle_pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
        unique_labels = [lbl for lbl in np.unique(labels) if lbl >= 0]
        num_clusters = len(unique_labels)
        
        # Tìm khoảng cách vật cản gần nhất tới robot (gốc toạ độ 0, 0)
        dists = np.linalg.norm(pts_obs[:, :2], axis=1)
        if len(dists) > 0:
            min_dist = float(np.min(dists))
            
    return ground_count, obstacle_count, num_clusters, min_dist


def measure_benchmark(points_xyz: np.ndarray, distance_threshold: float,
                      voxel_size: float = 0.1, eps: float = 0.5, min_points: int = 10,
                      warmup_runs: int = 2, timed_runs: int = 20) -> dict:
    """Đo số liệu và đo latency p50/p95 chuẩn rubric."""
    # Lọc ROI
    valid = np.isfinite(points_xyz).all(axis=1)
    pts = points_xyz[valid]
    in_roi = ((pts[:, 0] >= 0.0) & (pts[:, 0] <= 40.0) &
              (pts[:, 1] >= -15.0) & (pts[:, 1] <= 15.0) &
              (pts[:, 2] >= -2.5) & (pts[:, 2] <= 1.5))
    pts_roi = pts[in_roi]
    
    pcd_roi = o3d.geometry.PointCloud()
    pcd_roi.points = o3d.utility.Vector3dVector(pts_roi)
    pcd_down = pcd_roi.voxel_down_sample(voxel_size=voxel_size)
    pts_down = np.asarray(pcd_down.points)

    # Warmup runs (bỏ khởi tạo đầu)
    for _ in range(warmup_runs):
        _ = run_pipeline_single(pts_down, distance_threshold, eps, min_points)

    # Đo thời gian lặp 20 lần
    latencies = []
    ground_cnt = obs_cnt = n_clusters = 0
    min_distance = 0.0
    for _ in range(timed_runs):
        t0 = time.perf_counter()
        ground_cnt, obs_cnt, n_clusters, min_distance = run_pipeline_single(
            pts_down, distance_threshold, eps, min_points
        )
        t1 = time.perf_counter()
        latencies.append((t1 - t0) * 1000.0)  # ms

    p50 = float(np.percentile(latencies, 50))
    p95 = float(np.percentile(latencies, 95))

    return {
        "downsampled_pts": len(pts_down),
        "ground_pts": ground_cnt,
        "obstacle_pts": obs_cnt,
        "num_clusters": n_clusters,
        "min_dist_m": round(min_distance, 2),
        "latency_p50_ms": round(p50, 2),
        "latency_p95_ms": round(p95, 2)
    }


def main():
    parser = argparse.ArgumentParser(description="Sweep RANSAC distance threshold")
    parser.add_argument("--data-root", default="data/kitti_mini")
    parser.add_argument("--frames", nargs="+", default=["000019", "000011", "000008"])
    parser.add_argument("--dist-thresholds", nargs="+", type=float, default=[0.15, 0.20, 0.25, 0.30, 0.35])
    parser.add_argument("--out", default="results/obstacle_benchmark.csv")
    args = parser.parse_args()

    # Cố định seed cho tính lặp lại (reproducibility)
    np.random.seed(42)

    rows = []
    print(f"{'Frame':<8} | {'Thresh':<6} | {'Obs Pts':<8} | {'Clusters':<8} | {'Min Dist':<8} | {'p50 (ms)':<8}")
    print("-" * 60)
    for frame in args.frames:
        fr = load_frame(args.data_root, frame)
        pts_raw = fr["points"][:, :3]
        for dist_th in args.dist_thresholds:
            metrics = measure_benchmark(pts_raw, distance_threshold=dist_th)
            row = {
                "dataset": Path(args.data_root).name,
                "frame": frame,
                "dist_thresh_m": dist_th,
                **metrics
            }
            rows.append(row)
            print(f"{frame:<8} | {dist_th:<6.2f} | {metrics['obstacle_pts']:<8} | "
                  f"{metrics['num_clusters']:<8} | {metrics['min_dist_m']:<8.2f} | {metrics['latency_p50_ms']:<8.2f}")

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"\n-> Đã lưu bảng kết quả benchmark: {out_path} ({len(rows)} cấu hình)")


if __name__ == "__main__":
    main()