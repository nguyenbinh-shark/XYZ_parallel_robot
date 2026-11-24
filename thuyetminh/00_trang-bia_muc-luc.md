<div align="center">

**[TÊN TRƯỜNG ĐẠI HỌC]**
**KHOA [CƠ KHÍ / ĐIỆN – ĐIỆN TỬ / CƠ ĐIỆN TỬ]**

---

# THUYẾT MINH ĐỒ ÁN MÔN HỌC

## THIẾT KẾ VÀ CHẾ TẠO ROBOT SONG SONG 5 KHÂU THỰC HIỆN GẮP – ĐẶT (PICK & PLACE)

*Nền tảng điều khiển: ESP32-S3 · Framework Arduino (PlatformIO)*

---

|  |  |
|---|---|
| **Giảng viên hướng dẫn:** | [Học hàm, học vị – Họ và tên] |
| **Sinh viên thực hiện:** | Trần Nguyên Bình |
| **Mã số sinh viên:** | [MSSV] |
| **Lớp / Khóa:** | [Lớp – Khóa] |
| **Ngành:** | Kỹ thuật Cơ điện tử |

---

*[Địa danh], năm 2026*

</div>

---

## LỜI NÓI ĐẦU

Robot song song ngày càng được ứng dụng rộng rãi trong công nghiệp nhờ độ cứng vững cao, quán tính chuyển động thấp và tốc độ đáp ứng nhanh — đặc biệt phù hợp cho các thao tác gắp – đặt (pick and place) tốc độ cao. Cơ cấu **5 khâu phẳng (five-bar linkage)** là một trong những cấu hình song song điển hình, cho phép định vị đầu công tác trong mặt phẳng bằng hai động cơ đặt cố định trên đế.

Đồ án này trình bày quá trình **thiết kế, chế tạo và lập trình điều khiển** một mô hình robot song song 5 khâu, bổ sung **trục Z** để nâng/hạ và **giác hút chân không** để gắp vật, tạo thành một hệ thống Pick & Place hoàn chỉnh 3 bậc tự do. Hệ thống được điều khiển bằng vi điều khiển **ESP32-S3**, tích hợp cảm biến khoảng cách **VL53L0X**, màn hình **OLED**, giao thức truyền thông serial có khung + kiểm tra lỗi và một **giao diện điều khiển trên trình duyệt (Web HMI)**.

Quyển thuyết minh đi từ tổng quan, cơ sở lý thuyết động học, thiết kế cơ khí và điện – điện tử, thuật toán điều khiển, đến chế tạo và thực nghiệm đánh giá. Em xin chân thành cảm ơn thầy/cô **[GVHD]** đã tận tình hướng dẫn để đồ án được hoàn thành.

*Sinh viên thực hiện*

---

## MỤC LỤC

- **Chương 1. TỔNG QUAN** — [01_tong-quan.md](01_tong-quan.md)
  - 1.1 Đặt vấn đề và lý do chọn đề tài
  - 1.2 Tổng quan về robot song song và cơ cấu 5 khâu
  - 1.3 Ứng dụng Pick & Place trong công nghiệp
  - 1.4 Mục tiêu và nhiệm vụ của đề tài
  - 1.5 Phạm vi và giới hạn
  - 1.6 Phương pháp thực hiện và bố cục thuyết minh
- **Chương 2. PHÂN TÍCH VÀ THIẾT KẾ CƠ KHÍ** — [02_thiet-ke-co-khi.md](02_thiet-ke-co-khi.md)
  - 2.1 Yêu cầu thiết kế và thông số kỹ thuật mục tiêu
  - 2.2 Lựa chọn cấu hình cơ cấu 5 khâu
  - 2.3 Thiết kế cụm định vị XY
  - 2.4 Thiết kế trục Z
  - 2.5 Cơ cấu giác hút chân không
  - 2.6 Bố trí cảm biến trên đầu công tác
  - 2.7 Tính chọn và kiểm nghiệm động cơ
  - 2.8 Mô hình tổng thể và bảng vật tư
- **Chương 3. THIẾT KẾ HỆ THỐNG ĐIỆN – ĐIỆN TỬ** — [03_thiet-ke-dien-dien-tu.md](03_thiet-ke-dien-dien-tu.md)
  - 3.1 Sơ đồ khối phần cứng tổng thể
  - 3.2 Khối xử lý trung tâm ESP32-S3
  - 3.3 Khối động cơ XY (2 servo)
  - 3.4 Khối động cơ trục Z (mạch cầu H)
  - 3.5 Khối encoder quadrature
  - 3.6 Khối cảm biến VL53L0X và OLED (bus I2C)
  - 3.7 Khối giác hút (van + relay bơm)
  - 3.8 Công tắc giới hạn và bảo vệ
  - 3.9 Bảng phân công chân GPIO
  - 3.10 Nguồn cấp và ước tính công suất
- **Chương 4. ĐỘNG HỌC VÀ MÔ PHỎNG** — [04_dong-hoc-mo-phong.md](04_dong-hoc-mo-phong.md)
  - 4.1 Hệ tọa độ và mô hình hóa cơ cấu
  - 4.2 Bài toán động học ngược
  - 4.3 Giải phương trình lượng giác bằng tang nửa góc
  - 4.4 Chọn nghiệm và chế độ lắp
  - 4.5 Động học thuận
  - 4.6 Phân tích không gian làm việc
  - 4.7 Mô phỏng trên MATLAB
  - 4.8 Chọn điểm HOME và vùng cấm
  - 4.9 Kiểm chứng chéo
- **Chương 5. THIẾT KẾ HỆ THỐNG ĐIỀU KHIỂN VÀ PHẦN MỀM** — [05_dieu-khien-phan-mem.md](05_dieu-khien-phan-mem.md)
  - 5.1 Kiến trúc phần mềm tổng thể
  - 5.2 Chu trình setup() và loop()
  - 5.3 Module điều khiển XY
  - 5.4 Module điều khiển trục Z
  - 5.5 Module cảm biến và lọc tín hiệu
  - 5.6 Module homing và teach
  - 5.7 Module giác hút
  - 5.8 Máy trạng thái Pick & Place
  - 5.9 Module hiển thị OLED
  - 5.10 Xử lý lệnh người dùng
- **Chương 6. GIAO TIẾP VÀ GIÁM SÁT** — [06_giao-tiep-giam-sat.md](06_giao-tiep-giam-sat.md)
  - 6.1 Giao thức serial có khung
  - 6.2 Tập lệnh máy
  - 6.3 Telemetry và sự kiện
  - 6.4 Giao diện Web HMI
  - 6.5 Ví dụ host Python
- **Chương 7. CHẾ TẠO, THỰC NGHIỆM VÀ ĐÁNH GIÁ** — [07_che-tao-thuc-nghiem.md](07_che-tao-thuc-nghiem.md)
  - 7.1 Chế tạo và lắp ráp
  - 7.2 Nạp firmware và các môi trường build
  - 7.3 Hiệu chuẩn hệ thống
  - 7.4 Thực nghiệm từng module
  - 7.5 Thực nghiệm tích hợp Pick & Place
  - 7.6 Đánh giá kết quả
  - 7.7 Các vấn đề gặp phải và cách khắc phục
- **Chương 8. KẾT LUẬN VÀ HƯỚNG PHÁT TRIỂN** — [08_ket-luan.md](08_ket-luan.md)
- **PHỤ LỤC A–E** — [99_phu-luc.md](99_phu-luc.md)

---

## DANH MỤC HÌNH VẼ (dự kiến)

| Hình | Tên | Chương |
|---|---|---|
| 2.1 | Sơ đồ động học cơ cấu 5 khâu và ký hiệu các khâu | 2 |
| 2.2 | Mô hình lắp tổng thể robot (3D/ảnh) | 2 |
| 3.1 | Sơ đồ khối phần cứng tổng thể | 3 |
| 3.2 | Sơ đồ đấu nối chi tiết | 3 |
| 4.1 | Mô hình toán học và hệ tọa độ | 4 |
| 4.2 | Đồ thị không gian làm việc (MATLAB) | 4 |
| 4.3 | Lưu đồ thuật toán động học ngược | 4 |
| 5.1 | Kiến trúc phần mềm super-loop | 5 |
| 5.2 | Lưu đồ vòng điều khiển PD trục Z | 5 |
| 5.3 | Sơ đồ máy trạng thái Pick & Place | 5 |
| 6.1 | Giao diện Web HMI | 6 |
| 7.1 | Biểu đồ đáp ứng vị trí trục Z | 7 |

## DANH MỤC BẢNG (dự kiến)

| Bảng | Tên | Chương |
|---|---|---|
| 2.1 | Thông số kỹ thuật mục tiêu | 2 |
| 2.2 | Kích thước các khâu | 2 |
| 2.3 | Bảng vật tư (BOM) | 2 |
| 3.1 | Bảng phân công chân GPIO | 3 |
| 3.2 | Ước tính công suất tiêu thụ | 3 |
| 5.1 | Tham số bộ điều khiển PD | 5 |
| 6.1 | Bảng lệnh giao thức | 6 |
| 7.1 | Kết quả thực nghiệm độ chính xác | 7 |

## DANH MỤC TỪ VIẾT TẮT VÀ KÝ HIỆU

| Ký hiệu | Ý nghĩa |
|---|---|
| IK | Inverse Kinematics — Động học ngược |
| FK | Forward Kinematics — Động học thuận |
| DOF | Degree Of Freedom — Bậc tự do |
| PWM | Pulse Width Modulation — Điều chế độ rộng xung |
| PD | Proportional–Derivative — Bộ điều khiển tỉ lệ – đạo hàm |
| ToF | Time of Flight — Nguyên lý đo khoảng cách theo thời gian bay của ánh sáng |
| I2C | Inter-Integrated Circuit — Chuẩn giao tiếp 2 dây |
| LEDC | LED Control — Bộ tạo PWM phần cứng của ESP32 |
| ISR | Interrupt Service Routine — Trình phục vụ ngắt |
| HMI | Human–Machine Interface — Giao diện người – máy |
| PP | Pick & Place — Gắp và đặt |
| CCW / CW | Ngược / thuận chiều kim đồng hồ |
| L0…L4 | Chiều dài các khâu của cơ cấu 5 khâu |
| θ₁, θ₂ | Góc quay hình học của hai khâu chủ động |

## TÓM TẮT

Đồ án thiết kế và chế tạo một **robot song song 5 khâu 3 bậc tự do** phục vụ thao tác gắp – đặt vật. Cụm định vị mặt phẳng XY dùng cơ cấu five-bar với hai servo (kích thước khâu L0=55, L1=L4=70, L2=L3=130 mm); trục Z dùng động cơ DC giảm tốc GA25-370 kết hợp encoder quadrature và vít-me, điều khiển vòng kín bằng bộ **PD**; đầu công tác gắn cảm biến ToF **VL53L0X** và **giác hút chân không**. Toàn bộ được điều phối bởi một **máy trạng thái Pick & Place** phi chặn (non-blocking) trên ESP32-S3, có khả năng dò bề mặt, xác nhận vật và gắp lại khi rơi. Hệ thống hỗ trợ điều khiển qua **giao thức serial có kiểm tra lỗi** và **giao diện Web** thời gian thực. Kết quả cho thấy robot định vị và thực hiện chu trình gắp – đặt ổn định trong không gian làm việc thiết kế.

**Từ khóa:** robot song song, cơ cấu 5 khâu, động học ngược, Pick & Place, ESP32-S3, điều khiển PD, VL53L0X.
