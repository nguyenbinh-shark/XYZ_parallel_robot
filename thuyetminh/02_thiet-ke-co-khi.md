# CHƯƠNG 2. PHÂN TÍCH VÀ THIẾT KẾ CƠ KHÍ

## 2.1 Yêu cầu thiết kế và thông số kỹ thuật mục tiêu

Trước khi thiết kế, cần xác định các yêu cầu kỹ thuật của hệ thống. Bảng 2.1 tổng hợp các thông số mục tiêu.

**Bảng 2.1 — Thông số kỹ thuật mục tiêu**

| Thông số | Giá trị mục tiêu | Ghi chú |
|---|---|---|
| Số bậc tự do | 3 (X, Y, Z) | XY: cơ cấu 5 khâu; Z: vít-me |
| Không gian làm việc XY | vùng bao ≈ x∈[−40, 95] mm, y∈[55, 190] mm | Giao 2 vùng với hạn của 2 cánh tay |
| Hành trình trục Z | vài chục mm (hiệu chuẩn theo vít-me thực tế) | Đủ để hạ đầu hút chạm bề mặt |
| Cơ cấu gắp | Giác hút chân không | Vật nhẹ, mặt phẳng |
| Độ phân giải Z | cỡ ~0,01 mm/xung (theo encoder) | Xem Chương 5 |
| Tải trọng gắp | Nhẹ (< ~100 g) | Giới hạn bởi bơm hút và servo |
| Nguồn điều khiển | ESP32-S3 | 3.3V logic |

## 2.2 Lựa chọn cấu hình cơ cấu 5 khâu

### 2.2.1 Sơ đồ động học

Cơ cấu định vị XY là một **five-bar linkage** với hai khớp chủ động đặt cố định trên đế (Hình 2.1).

```
         E (x, y)  ← Đầu công tác
        / \
      L2   L3
      /     \
     B       D
     |       |
    L1       L4
     |       |
     A───────C
   (0,0)  (55,0)
```
**Hình 2.1 — Sơ đồ động học cơ cấu 5 khâu**

- **Động cơ 1** đặt tại A(0, 0), điều khiển khâu gần L1 (quay góc θ₁ so với trục +X).
- **Động cơ 2** đặt tại C(L0, 0), điều khiển khâu gần L4 (quay góc θ₂ so với trục +X).
- Hai khâu xa L2, L3 khép kín tại đầu công tác **E(x, y)** qua các khớp B và D.
- Quy ước: trục x nằm ngang, trục y hướng lên; **chế độ lắp "khuỷu hướng ra ngoài"**.

### 2.2.2 Chọn kích thước các khâu

Kích thước các khâu được lấy theo mô hình MATLAB tham chiếu (l0=5.5, l1=7, l2=13, l3=13, l4=7 cm) nhân 10 để quy về mm, khai báo trong `src/ik_5bar.h`.

**Bảng 2.2 — Kích thước các khâu**

| Ký hiệu | Tên khâu | Giá trị | Vai trò |
|---|---|---|---|
| L0 | Khoảng cách 2 trục động cơ | 55 mm | Base — khoảng cách A↔C |
| L1 | Khâu gần trái | 70 mm | Nối động cơ 1 |
| L2 | Khâu xa trái | 130 mm | Nối đầu công tác |
| L3 | Khâu xa phải | 130 mm | Nối đầu công tác |
| L4 | Khâu gần phải | 70 mm | Nối động cơ 2 |

Cơ cấu **đối xứng** (L1=L4, L2=L3) giúp không gian làm việc đối xứng qua trục thẳng đứng đi qua trung điểm A–C, đơn giản hóa việc chọn điểm HOME và hiệu chuẩn. Tỉ lệ khâu xa/khâu gần ≈ 130/70 ≈ 1,86 được chọn để mở rộng vùng làm việc theo phương y mà vẫn giữ được độ cứng vững.

## 2.3 Thiết kế cụm định vị XY

- **Bố trí động cơ:** hai servo đặt cố định trên đế, đồng trục A và C cách nhau L0 = 55 mm; trục quay của còi servo vuông góc với mặt phẳng làm việc.
- **Khâu chủ động (L1, L4):** gắn trực tiếp lên còi (horn) của servo. Cần cứng, nhẹ; góc lắp còi phải khớp với **offset hiệu chỉnh** trong phần mềm (xem 5.3).
- **Khâu bị động (L2, L3):** nối khâu gần với đầu công tác qua các khớp quay tự do (vòng bi/bạc). Yêu cầu khớp có **độ rơ nhỏ** để giảm sai số định vị.
- **Đầu công tác E:** điểm khép kín của hai khâu xa, mang cụm trục Z + giác hút + cảm biến.
- **Vật liệu đề xuất:** khâu in 3D (PLA/PETG) hoặc mica/nhôm tấm; khớp dùng bu-lông + vòng bi nhỏ.

> **Lưu ý cơ – điều khiển:** vì là chuỗi kín, nếu hai khâu chủ động không chuyển động **đồng bộ**, tỉ lệ θ₁/θ₂ sẽ lệch nghiệm động học ở quá độ, sinh **nội lực đối kháng** (binding) gây rung, mòn khớp. Do đó phần mềm nội suy hai servo theo **chung một tham số thời gian** để chúng luôn kết thúc cùng lúc (xem 5.3.2).

## 2.4 Thiết kế trục Z

Trục Z đảm nhiệm nâng/hạ đầu hút để gắp và đặt vật.

- **Động cơ:** DC giảm tốc **GA25-370** (12 VDC), tỉ số truyền hộp số **21,3:1**, kèm **encoder Hall 2 kênh AB** (11 xung/kênh/vòng trước hộp số).
- **Truyền động:** động cơ quay **vít-me (lead screw)** biến chuyển động quay thành tịnh tiến của đầu công tác; có **thanh dẫn hướng** chống xoay.
- **Cảm biến vị trí:** encoder quadrature đọc 4× (4 lần/chu kỳ) cho độ phân giải cao; kết hợp **công tắc hành trình giới hạn trên** làm mốc tham chiếu và bảo vệ.

> **Cần hiệu chuẩn:** độ phân giải trục Z (số xung/mm) phụ thuộc **bước vít-me thực tế**. Công thức: `xung/mm = PPR × 4 × tỉ_số_truyền / bước_vít`. Giá trị này **phải được đo và chốt** theo vít-me lắp thật (quy trình hiệu chuẩn ở mục 7.3.2). Trong firmware, hằng số `Z_SCREW_PITCH` và `Z_PULSE_PER_MM` khai báo tại `src/z_axis.h`.

## 2.5 Cơ cấu giác hút chân không

Đầu công tác sử dụng **giác hút chân không** để gắp vật phẳng, nhẹ:

- **Đầu hút (giác hút):** cao su mềm, tạo buồng kín khi áp lên bề mặt vật.
- **Bơm chân không:** hút không khí tạo áp âm để giữ vật; điều khiển bằng **relay**.
- **Van điện từ:** đóng/mở đường thông khí. Khi hút → giữ kín; khi nhả → mở van cho khí vào phá chân không, thả vật nhanh.
- **Trình tự nhả:** mở van trước, sau **~150 ms** (xả áp âm) mới tắt bơm để vật rời dứt khoát, tránh dính lại.

Cơ cấu này đơn giản, không cần cơ cấu kẹp cơ khí, phù hợp vật có bề mặt phẳng nhẵn.

## 2.6 Bố trí cảm biến trên đầu công tác

Trên cụm đầu công tác (di chuyển cùng trục Z) gắn **cảm biến khoảng cách VL53L0X** hướng **nhìn xuống** bề mặt làm việc.

```
  VL53L0X (gắn trên đầu công tác)
       │  ↕ Z_HOME_SENSOR_OFFSET  (khoảng cách cố định cảm biến → đầu hút)
  Đầu giác hút
       │  ↕ h  (chiều cao đầu hút so với bề mặt)
  ════════════════  Bề mặt làm việc

  Số đọc cảm biến  =  h + offset  ⟹  Z_vật_lý = số_đọc − offset
```
**Hình 2.2 — Quan hệ hình học giữa cảm biến và đầu hút**

Nhờ quan hệ này, robot xác định được cao độ bề mặt để **homing** và **dò vật** mà không cần chạm cơ khí (xem 5.6). Ngoài ra còn có **công tắc hành trình giới hạn trên** đặt ở đỉnh hành trình Z để làm mốc tham chiếu và ngắt an toàn.

> Khoảng cách lắp `Z_HOME_SENSOR_OFFSET` là một **hằng số hiệu chuẩn vật lý** — phải đo đúng theo cách lắp thực tế (khai báo tại `src/z_home.h`).

## 2.7 Tính chọn và kiểm nghiệm động cơ

### 2.7.1 Động cơ XY (servo)

Mô-men yêu cầu của mỗi servo phụ thuộc khối lượng khâu, đầu công tác và cánh tay đòn. Do tải nhẹ và tốc độ vừa phải, chọn **servo RC tiêu chuẩn** (mô-men cỡ vài kg·cm) là đủ. Servo được điều khiển bằng xung **50 Hz**, độ rộng **500–2500 µs** tương ứng góc **0–180°**. Ưu điểm: tích hợp sẵn mạch điều khiển vị trí, điều khiển đơn giản bằng góc lệnh (vòng hở).

### 2.7.2 Động cơ trục Z

Trục Z chịu tải trọng cụm đầu công tác + lực gắp. Động cơ **GA25-370** giảm tốc 21,3:1 cung cấp mô-men đủ lớn ở tốc độ thấp; encoder cho phép điều khiển **vòng kín** để định vị chính xác. Vít-me tự hãm (self-locking) giúp giữ vị trí khi mất điện động cơ.

### 2.7.3 Kiểm nghiệm sơ bộ

Với tải nhẹ, yếu tố quyết định không phải mô-men tối đa mà là **độ mịn điều khiển** và **độ rơ cơ khí**. Thực nghiệm (Chương 7) sẽ kiểm chứng khả năng bám vị trí và độ lặp lại.

## 2.8 Mô hình tổng thể và bảng vật tư

Robot gồm 3 cụm chính lắp trên một đế chung: **cụm XY (5 khâu)** → mang **cụm Z (vít-me)** → mang **đầu công tác (giác hút + VL53L0X)**. Mạch điều khiển và nguồn đặt cạnh đế.

**Bảng 2.3 — Bảng vật tư (BOM) chính**

| STT | Thành phần | Số lượng | Ghi chú |
|---|---|---|---|
| 1 | Vi điều khiển ESP32-S3 (board `esp32s3usbotg`) | 1 | Khối điều khiển trung tâm |
| 2 | Servo RC (định vị XY) | 2 | 50 Hz, 500–2500 µs |
| 3 | Động cơ DC GA25-370 + encoder | 1 | Giảm tốc 21,3:1, encoder 11 PPR |
| 4 | Mạch cầu H (L298N / TB6612FNG) | 1 | Điều khiển động cơ Z |
| 5 | Vít-me + thanh dẫn hướng | 1 bộ | Trục Z |
| 6 | Cảm biến ToF VL53L0X | 1 | I2C, đo khoảng cách |
| 7 | Màn hình OLED SSD1306 0.96" | 1 | I2C, hiển thị trạng thái |
| 8 | Bơm chân không + van + giác hút | 1 bộ | Cơ cấu gắp |
| 9 | Relay điều khiển bơm | 1 | Active-HIGH |
| 10 | Công tắc hành trình (giới hạn trên) | 1 | Endstop |
| 11 | Bộ khâu 5-bar (in 3D/gia công) + khớp | 1 bộ | L0..L4 |
| 12 | Nguồn tổ ong / adapter | 1 | Cấp cho động cơ, bơm, logic |

Chi tiết chân kết nối và sơ đồ điện được trình bày ở **Chương 3**.
