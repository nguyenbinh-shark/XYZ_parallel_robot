# CHƯƠNG 7. CHẾ TẠO, THỰC NGHIỆM VÀ ĐÁNH GIÁ

## 7.1 Chế tạo và lắp ráp

- **Khâu cơ khí:** các khâu L1–L4 và giá đỡ được **in 3D** (PLA/PETG) hoặc cắt từ mica/nhôm tấm theo kích thước ở Chương 2. Khớp quay dùng bu-lông + vòng bi/bạc để giảm độ rơ.
- **Lắp cụm XY:** cố định hai servo lên đế cách nhau L0 = 55 mm; gắn khâu gần lên còi servo sao cho ở vị trí HOME, góc còi nằm giữa tầm hành trình (đây là bước liên quan trực tiếp đến `SERVO*_OFFSET`).
- **Lắp trục Z:** ghép động cơ GA25-370 với vít-me, thanh dẫn hướng chống xoay; gắn cụm đầu công tác (giác hút + VL53L0X) lên bàn trượt; đặt công tắc giới hạn ở đỉnh hành trình.
- **Đấu nối điện:** theo Bảng 3.1 (Chương 3). Chung GND toàn hệ; tách nguồn động lực khỏi nguồn logic; thêm tụ lọc gần servo và cầu H.

## 7.2 Nạp firmware và các môi trường build

Dự án dùng **PlatformIO** với **4 môi trường build**, phân biệt bằng `build_src_filter` (khai báo trong `platformio.ini`). Cách bố trí này cho phép **kiểm tra độc lập từng cụm** trước khi chạy toàn hệ.

| Môi trường (`-e`) | Mục đích | Thư viện | Nguồn biên dịch |
|---|---|---|---|
| `esp32s3usbotg` | **Firmware chính** (Pick & Place đầy đủ) | ESP32Servo, Adafruit_VL53L0X, Adafruit SSD1306 | toàn bộ `src/` trừ các `test_*.cpp` |
| `test_xy_servo` | Kiểm tra riêng XY + IK | ESP32Servo | `test_xy_servo.cpp` + `ik_5bar.c` |
| `test_vl53` | Kiểm tra riêng cảm biến VL53L0X | Adafruit_VL53L0X | `test_vl53.cpp` + `vl53l0x.cpp` |
| `test_z_axis` | Kiểm tra riêng trục Z + encoder | – | `test_z_axis.cpp` + `z_axis.cpp` |

Lệnh nạp và theo dõi, ví dụ với môi trường test trục Z:

```
pio run -e test_z_axis -t upload
pio device monitor -b 115200
```

## 7.3 Hiệu chuẩn hệ thống

Hiệu chuẩn là bước **bắt buộc** trước khi vận hành, để số liệu phần mềm khớp phần cứng thật.

### 7.3.1 Hiệu chuẩn offset servo XY

1. Nạp `test_xy_servo`, dùng lệnh `home` để đưa robot về HOME.
2. Quan sát dáng cơ cấu; chỉnh `offset o1,o2` (hoặc `invert`) đến khi cơ cấu về đúng tư thế HOME đối xứng.
3. Ghi giá trị offset tìm được vào `SERVO1_OFFSET/SERVO2_OFFSET` trong `main.cpp` **và** `SV1_OFF/SV2_OFF` trong `hmi/index.html` (phải trùng nhau).

### 7.3.2 Hiệu chuẩn độ phân giải trục Z (xung/mm)

1. Nạp `test_z_axis`. Dùng lệnh `raw <pwm>` chạy PWM thô để **kiểm tra chiều đếm encoder** (đi lên phải làm số đếm tăng/giảm nhất quán); nếu ngược → hoán đổi 2 chân encoder A/B hoặc đảo IN1/IN2.
2. Cho trục Z di chuyển một quãng, **đo quãng đường thực tế** bằng thước, rồi dùng lệnh `cal <mm>` để phần mềm tính lại `Z_SCREW_PITCH` / `Z_PULSE_PER_MM` cho khớp.
3. Cập nhật hằng số vào `z_axis.h`.

> Công thức: `Z_PULSE_PER_MM = Z_ENC_PPR × 4 × Z_GEAR_RATIO / Z_SCREW_PITCH`.

### 7.3.3 Hiệu chuẩn offset cảm biến VL53L0X

1. Đặt đầu hút chạm bề mặt (h = 0), đọc giá trị VL53L0X.
2. Giá trị đọc được chính là `Z_HOME_SENSOR_OFFSET_MM` (khoảng cách cảm biến → đầu hút).
3. Cập nhật vào `z_home.h`; kiểm tra lại các ngưỡng suy ra: `Z_HOME_NEAR = offset+3`, `Z_PICK_NEAR = offset+2`, `Z_GRIP_DETECT = offset+15`.

## 7.4 Thực nghiệm từng module

### 7.4.1 Cụm XY

- **Nội dung:** đưa robot đến các điểm biết trước (HOME, 4 góc demo), đo sai lệch vị trí đầu công tác; chạy `demo` (hình vuông 20×20 mm) đánh giá độ mượt và độ lặp lại.
- **Chỉ tiêu:** sai số định vị, độ lặp lại, không có hiện tượng giật/kêu (binding).

### 7.4.2 Trục Z

- **Nội dung:** ra lệnh đến nhiều vị trí Z, ghi đáp ứng vị trí theo thời gian (dùng `mon on` in mỗi 200 ms); đánh giá thời gian xác lập, độ vọt lố, sai số xác lập.
- **Chỉ tiêu:** bám vị trí trong vùng chết ±4 xung, không dao động quanh đích.

```
  Z(mm)
   │        ┌──────────  ← đích
   │       ╱
   │     ╱   (đáp ứng PD: lên nhanh, xác lập êm, không vọt lố)
   │   ╱
   └─┴───────────────► t
```
**Hình 7.1 — Dạng đáp ứng vị trí trục Z (minh họa; đồ thị thực đo bằng `mon`)**

### 7.4.3 Cảm biến VL53L0X

- **Nội dung:** đọc khoảng cách tại cự ly cố định, so sánh giữa `read_mm` (thô) và `read_median`/`read_filtered` (đã lọc); đánh giá độ ổn định, khả năng loại spike.
- **Chỉ tiêu:** độ lệch chuẩn của giá trị đã lọc nhỏ; không có nhảy giá trị bất thường.

**Bảng 7.1 — Mẫu bảng ghi kết quả thực nghiệm** *(điền số đo thực tế)*

| Hạng mục | Điều kiện | Kết quả đo | Nhận xét |
|---|---|---|---|
| Sai số định vị XY | 5 điểm mẫu | … mm | |
| Độ lặp lại XY | 10 lần về HOME | … mm | |
| Sai số xác lập Z | 5 vị trí | … mm | |
| Độ ổn định VL53L0X | 50 mẫu, cự ly cố định | σ = … mm | |

## 7.5 Thực nghiệm tích hợp Pick & Place

**Quy trình vận hành chuẩn:**

```
Bước 1: Cấp nguồn → homez            (chuẩn hóa Z bằng VL53L0X, ~105 ms)
Bước 2: 27.5,80 → teachpick          (dạy vị trí & cao độ lấy vật)
Bước 3: 42.5,80 → teachplace         (dạy vị trí & cao độ đặt vật)
Bước 4: ppinfo                       (kiểm tra toàn bộ thông số)
Bước 5: run                          (chạy chu trình); abort (dừng khẩn)
```

- **Nội dung:** chạy chu trình gắp – đặt lặp lại nhiều lần; đếm số lần thành công; thử tạo tình huống **rơi vật** để kiểm tra cơ chế `repick` và trạng thái `DROP_DETECTED`.
- **Chỉ tiêu:** tỉ lệ gắp – đặt thành công; xác nhận cơ chế dò vật, xác nhận gắp, giám sát rơi vật hoạt động đúng.

## 7.6 Đánh giá kết quả

- **Động học:** kết quả IK từ firmware khớp mô hình MATLAB và mô phỏng HMI tại các điểm kiểm chứng → thuật toán đúng.
- **Cơ cấu XY:** nội suy đồng bộ giúp chuyển động mượt, không sinh nội lực; định vị lặp lại tốt trong vùng làm việc.
- **Trục Z:** PD bám vị trí ổn định, phanh giữ vị trí chắc; homing/teach xác định cao độ bề mặt tin cậy.
- **Tích hợp:** chu trình Pick & Place tự động hoàn chỉnh, có khả năng phát hiện lỗi gắp và tự thử lại.
- **Giám sát:** telemetry + Web HMI cho phép theo dõi và điều khiển thời gian thực.

Các **hạn chế** quan sát được: nội suy XY theo không gian góc (đường cong, chưa thẳng Descartes); cụm XY điều khiển vòng hở (phụ thuộc độ chính xác servo và độ rơ khớp); độ chính xác Z phụ thuộc hiệu chuẩn xung/mm và độ rơ vít-me.

## 7.7 Các vấn đề gặp phải và cách khắc phục

### 7.7.1 Hiện tượng bơm kêu khi khởi động (GPIO45 strapping)

GPIO45 là chân strapping (VDD_SPI), ROM bootloader giữ mức khi cấp nguồn → relay bơm có thể kích nhẹ. **Khắc phục:** gọi `gripper_init()` sớm nhất trong `setup()`; triệt để hơn thì xử lý phần cứng (điện trở kéo đúng chiều active, hoặc đổi chân không-strapping).

### 7.7.2 Trục Z đếm ngược chiều / PID mất ổn định

Nếu encoder đếm ngược chiều quy ước, PD sẽ đẩy động cơ ra xa đích. **Khắc phục:** hoán đổi 2 chân encoder A/B **hoặc** đảo IN1/IN2 (chỉ một trong hai). Kiểm tra bằng `raw <pwm>` trong `test_z_axis`.

### 7.7.3 ⚠️ Rà soát các thông số cần hiệu chuẩn theo phần cứng thực tế

Trong quá trình phát triển, một số hằng số trong mã nguồn **chưa thống nhất** với tài liệu/chú thích. Các giá trị dưới đây lấy theo **mã nguồn firmware hiện tại**, nhưng **phải được đo và chốt lại** theo phần cứng lắp thực tế trước khi công bố số liệu chính thức:

**Bảng 7.2 — Các thông số cần rà soát/hiệu chuẩn**

| # | Thông số | Giá trị trong mã nguồn | Ghi chú/khác biệt | Cách chốt |
|---|---|---|---|---|
| 1 | `Z_SCREW_PITCH` (bước vít) → xung/mm | 2.0 mm → **468,6 xung/mm** | Chú thích ghi "8 mm/vòng" → 117,15 xung/mm | Đo bằng `test_z_axis` lệnh `cal` (mục 7.3.2) |
| 2 | Hành trình Z `Z_MIN_MM/Z_MAX_MM` | **−100 … +100 mm** | Tài liệu cũ ghi 0…50 mm | Đặt theo chiều dài vít-me thực tế |
| 3 | `Z_HOME_SENSOR_OFFSET_MM` | **50 mm** | Tài liệu cũ ghi 8 mm | Đo theo cách lắp cảm biến (mục 7.3.3) |
| 4 | Chân encoder A/B | `ENC_A=10, ENC_B=11` | Chú thích prose ghi "C1→11, C2→10" | Đối chiếu đấu nối thực tế, kiểm tra chiều đếm |

> **Khuyến nghị:** sau khi hiệu chuẩn, cập nhật đồng bộ các hằng số vào `z_axis.h`, `z_home.h` và **Phụ lục A** để toàn quyển thuyết minh dùng một bộ số thống nhất.
