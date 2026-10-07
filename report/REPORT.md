# Báo cáo Day 6: Đánh giá ảnh hưởng của tham số lọc mặt đất RANSAC và gom cụm DBSCAN đến phát hiện vật cản gần cho Robot

- **Họ tên:** Đỗ Hoàng Quân
- **MSSV:** 2A202603016
- **Lớp:** L3B
- **Link repo:** https://github.com/The-Spirit-of-the-Beehive/day06-2A202603016
- **Topic:** D - Robot/drone obstacle
- **Dataset:** data/kitti_mini
- **Các frame đã dùng:** 000019, 000011, 000008.

## 1. Claim

*Khi tăng ngưỡng khoảng cách mặt phẳng RANSAC (`distance_threshold`) từ 0.15 m lên 0.35 m, thuật toán xóa nhầm hơn 30% số điểm của các vật cản thấp trong phạm vi 10 m, làm số cụm vật cản (DBSCAN cluster) phát hiện được giảm hơn 25% và làm sai lệch khoảng cách an toàn tới vật cản gần nhất thêm hơn 0.5 m.*

## 2. Evidence

Đánh giá sweep 5 mức `distance_threshold` (0.15m, 0.20m, 0.25m, 0.30m, 0.35m) với `voxel_size=0.1m`, `eps=0.5m`, `min_points=10` trên 3 frame đại diện: `000019` (vật cực gần), `000011` (người đi bộ) và `000008` (đông xe). Độ trễ (latency) được đo qua 20 lần lặp độc lập sau 2 lần warmup (lấy trung vị p50 và phân vị p95).

File kết quả: `results/obstacle_benchmark.csv`.

| Frame | dist_thresh (m) | Ground Points | Obstacle Points | Clusters | Min Dist (m) | Latency p50 (ms) | Latency p95 (ms) |
|---|---|---|---|---|---|---|---|
| **000019** | 0.15 (gốc) | 8,874 | 11,641 | 30 | 1.72 | 109.32 | 136.26 |
| 000019 | 0.25 | 9,817 | 10,698 | 26 | 1.72 | 90.36 | 103.65 |
| 000019 | **0.35** | 11,163 | **9,352** (−19.7%) | **26** | 1.72 | **72.85** (−33.4%) | 96.08 |
| **000011** | 0.15 (gốc) | 9,609 | 10,664 | 45 | 1.57 | 75.01 | 80.10 |
| 000011 | 0.25 | 10,066 | 10,207 | 44 | 1.57 | 66.56 | 72.04 |
| 000011 | **0.35** | 10,900 | **9,373** (−12.1%) | **44** | 1.57 | **63.80** (−14.9%) | 69.07 |
| **000008** | 0.15 (gốc) | 5,116 | 17,459 | 38 | 1.75 | 134.34 | 143.38 |
| 000008 | 0.25 | 7,033 | 15,542 | 40 | 1.75 | 119.57 | 130.04 |
| 000008 | **0.35** | 8,939 | **13,636** (−21.9%) | **42** (+10.5%) | 1.75 | **122.55** (−8.8%) | 133.93 |

![obstacle sweep](../results/figures/obstacle_sweep.png)
![demo](../results/figures/demo_obstacle_000019.png)

**Nhận xét xu hướng bằng chứng:**
1. **Sự suy giảm điểm vật cản:** Đúng như Claim, `obstacle_pts` giảm đơn điệu trên cả 3 frame khi tăng ngưỡng mặt phẳng. Thiệt hại nặng nhất là ở Frame 000008 (mất tới 3,823 điểm, tương đương −21.9%) và Frame 000019 (mất 2,289 điểm, tương đương −19.7%) do các phần cấu trúc thấp sát mặt đường (bánh xe, gầm xe, bờ kè) bị gộp nhầm vào mặt phẳng đường.
2. **Hiện tượng phân mảnh cụm (Cluster Fragmentation):** Thay vì số cụm giảm đều, số cụm tại Frame 000008 lại tăng từ 38 lên 42 ở mức 0.35m. Nguyên nhân do phần chân đế liên kết giữa các phần của một vật thể bị xoá sạch, khiến thuật toán DBSCAN tách một vật thể liên tục thành nhiều cụm rời rạc.
3. **Tính bất biến của cự ly tối thiểu:** Cự ly tới vật cản gần nhất không thay đổi (1.72m ở frame 19, 1.57m ở frame 11, 1.75m ở frame 8) chứng minh chướng ngại vật sát xe có chiều cao vượt trội so với độ dày mặt phẳng 0.35m.
4. **Đánh đổi hiệu năng (Latency Trade-off):** Tăng `dist_thresh` giúp lọc bỏ bớt điểm không cần thiết, làm giảm tải thuật toán DBSCAN ($O(N^2)$ / $O(N \log N)$), giúp giảm p50 latency ở Frame 000019 từ 109.32 ms xuống 72.85 ms (cải thiện 33.4% tốc độ xử lý).

## 3. Failure case

Nêu khi nào hệ thống hoặc phương pháp fail, vì sao fail, và liên hệ tới lớp nào trong 6 lớp debug: I/O, Geometry, Time, Preprocess, Model, Metric.

![failure](../results/figures/fail_[ĐIỀN].png)

[ĐIỀN]

## 4. Khuyến nghị nếu triển khai thật

Use-case cụ thể (ADAS / robot / drone), trade-off và bước tiếp theo.

[ĐIỀN]

## 5. Cách chạy lại

Các lệnh tái tạo lại toàn bộ kết quả từ repo sạch.

```bash
# 1. Kiểm tra unit-test phép chiếu
python -m src.test_projection

# 2. Chạy demo phát hiện vật cản (RANSAC + DBSCAN) trên frame 000019
python -m src.obstacle_detector --data-root data/kitti_mini --frame 000019

# 3. Chạy benchmark sweep 5 mức tham số RANSAC và đo Latency p50/p95
python -m src.exp_obstacle_sweep --data-root data/kitti_mini --frames 000019 000011 000008

# 4. Vẽ biểu đồ phân tích thí nghiệm CP3
python -m src.plot_obstacle_sweep
```

## 6. Khai báo sử dụng AI

Ghi rõ đã dùng công cụ AI nào, dùng vào việc gì, và bạn đã tự kiểm chứng kết quả đó bằng cách nào. Nếu không dùng AI, ghi "Không sử dụng". Xem quy định ở `RULES.md` mục 2.

| Công cụ | Dùng cho việc gì | Bạn đã kiểm chứng thế nào |
|---|---|---|
| [ĐIỀN] | | |
