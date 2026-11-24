# THUYẾT MINH ĐỒ ÁN — Robot song song 5 khâu Pick & Place

Quyển thuyết minh chi tiết cho đề tài **Robot song song phẳng 5 khâu (five-bar) tích hợp trục Z, thực hiện Pick & Place**, nền tảng **ESP32-S3 / Arduino (PlatformIO)**.

Tài liệu được chia thành nhiều file Markdown theo chương, đọc theo thứ tự dưới đây.

## Mục lục file

| File | Nội dung |
|---|---|
| [00_trang-bia_muc-luc.md](00_trang-bia_muc-luc.md) | Trang bìa, lời nói đầu, mục lục, danh mục hình/bảng, từ viết tắt, tóm tắt |
| [01_tong-quan.md](01_tong-quan.md) | Chương 1 — Tổng quan đề tài |
| [02_thiet-ke-co-khi.md](02_thiet-ke-co-khi.md) | Chương 2 — Phân tích & thiết kế cơ khí |
| [03_thiet-ke-dien-dien-tu.md](03_thiet-ke-dien-dien-tu.md) | Chương 3 — Thiết kế hệ thống điện – điện tử |
| [04_dong-hoc-mo-phong.md](04_dong-hoc-mo-phong.md) | Chương 4 — Động học và mô phỏng |
| [05_dieu-khien-phan-mem.md](05_dieu-khien-phan-mem.md) | Chương 5 — Thiết kế hệ thống điều khiển & phần mềm |
| [06_giao-tiep-giam-sat.md](06_giao-tiep-giam-sat.md) | Chương 6 — Giao tiếp và giám sát |
| [07_che-tao-thuc-nghiem.md](07_che-tao-thuc-nghiem.md) | Chương 7 — Chế tạo, thực nghiệm và đánh giá |
| [08_ket-luan.md](08_ket-luan.md) | Chương 8 — Kết luận và hướng phát triển |
| [99_phu-luc.md](99_phu-luc.md) | Phụ lục A–E (thông số, đấu nối, mã nguồn, giao thức, tài liệu tham khảo) |

> **Ghi chú số liệu:** Toàn bộ hằng số trong quyển thuyết minh lấy theo **mã nguồn firmware hiện tại** (`ik_5bar.h`, `z_axis.h`, `z_home.h`, `main.cpp`, `gripper.h`, `vl53l0x.h`). Một số thông số cơ khí (bước vít-me, hành trình Z, offset cảm biến) **phải hiệu chuẩn theo phần cứng thực tế** — xem cảnh báo trong Chương 7, mục 7.7.

> Hình vẽ đặt trong thư mục `hinh/`. Sơ đồ khối/flowchart dùng khối ASCII (đồng bộ với `BAOCAO.md`) hoặc Mermaid.
