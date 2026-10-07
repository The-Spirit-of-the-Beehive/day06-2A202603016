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

Bảng hoặc plot số liệu, kèm ảnh/video demo. Ghi rõ đường dẫn file trong `results/`.

| Cấu hình / mức perturb | Metric 1 | Metric 2 | Ghi chú |
|---|---|---|---|
| [ĐIỀN] | | | |

![demo](../results/figures/[ĐIỀN].png)

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
[ĐIỀN]
```

## 6. Khai báo sử dụng AI

Ghi rõ đã dùng công cụ AI nào, dùng vào việc gì, và bạn đã tự kiểm chứng kết quả đó bằng cách nào. Nếu không dùng AI, ghi "Không sử dụng". Xem quy định ở `RULES.md` mục 2.

| Công cụ | Dùng cho việc gì | Bạn đã kiểm chứng thế nào |
|---|---|---|
| [ĐIỀN] | | |
