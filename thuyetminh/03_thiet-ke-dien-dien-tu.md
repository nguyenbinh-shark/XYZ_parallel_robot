# CHƯƠNG 3. THIẾT KẾ HỆ THỐNG ĐIỆN – ĐIỆN TỬ

## 3.1 Sơ đồ khối phần cứng tổng thể

Toàn bộ hệ thống được điều phối bởi vi điều khiển ESP32-S3. Sơ đồ khối tổng thể như Hình 3.1.

```
┌─────────────────────────────────────────────────────────────────┐
│                        ESP32-S3                                 │
│                                                                 │
│  Serial Input ──► process() ──► State Machine (Pick & Place)    │
│                                        │                        │
│            ┌───────────────────────────┼────────────────┐       │
│            ▼                           ▼                ▼       │
│   ┌─────────────────┐    ┌──────────────────┐  ┌─────────────┐  │
│   │  Chức năng 1    │    │  Chức năng 2     │  │ Chức năng 3 │  │
│   │  Robot Phẳng    │    │  Độ Cao (Trục Z) │  │ Sensor +    │  │
│   │  (5-bar XY)     │    │  DC Motor +      │  │ Giác Hút    │  │
│   │                 │    │  Encoder + PD    │  │             │  │
│   └────────┬────────┘    └────────┬─────────┘  └──────┬──────┘  │
│            │                      │                   │         │
└────────────┼──────────────────────┼───────────────────┼─────────┘
             │                      │                   │
     ┌───────┴───────┐      ┌───────┴──────┐   ┌────────┴─────┐
     │  Servo 1 & 2  │      │  H-bridge +  │   │ VL53L0X (I2C)│
     │  GPIO 48, 47  │      │  DC Motor    │   │ OLED   (I2C) │
     │               │      │  Encoder A,B │   │ Van GPIO 42  │
     └───────────────┘      └──────────────┘   │ Relay GPIO 45│
                                               └──────────────┘
```
**Hình 3.1 — Sơ đồ khối phần cứng tổng thể**

Hệ thống gồm ba cụm chức năng: **(1) định vị XY** bằng 2 servo; **(2) trục Z** bằng động cơ DC + encoder + mạch cầu H; **(3) cảm biến + giác hút**. Ngoài ra có màn OLED hiển thị trạng thái. Tất cả kết nối về ESP32-S3.

## 3.2 Khối xử lý trung tâm ESP32-S3

- **Board:** `esp32s3usbotg` (Espressif ESP32-S3-USB-OTG), framework Arduino.
- **Đặc điểm nổi bật:** lõi kép Xtensa LX7, nhiều GPIO, hỗ trợ **PWM phần cứng (LEDC)**, ngắt ngoài trên mọi GPIO, bộ I2C linh hoạt, **USB-CDC** tích hợp.
- **Lý do chọn:** đủ ngoại vi cho đồng thời 2 servo + 1 động cơ DC (PWM + 2 kênh encoder ngắt) + bus I2C (cảm biến + OLED) + 2 ngõ ra giác hút; nạp/giao tiếp trực tiếp qua **USB-CDC 115200 baud** không cần mạch USB-UART rời.
- **Mức logic:** 3.3V. Các tín hiệu ra driver, relay cần tương thích mức 3.3V hoặc qua tầng đệm.

> **Lưu ý USB-CDC:** khi biên dịch, có thể cần bật tùy chọn `USB CDC On Boot` để cổng Serial hoạt động qua USB (nêu trong `hmi/README.md`).

## 3.3 Khối động cơ XY (2 servo)

- **Servo 1** — GPIO **48** — động cơ tại A (trái).
- **Servo 2** — GPIO **47** — động cơ tại C (phải).
- Điều khiển bằng thư viện **ESP32Servo**: tần số **50 Hz**, độ rộng xung **500–2500 µs** ↔ góc **0–180°**.
- Servo được cấp nguồn riêng (5–6V, dòng đủ lớn) để tránh sụt áp làm reset ESP32; **chung GND** với vi điều khiển.

## 3.4 Khối động cơ trục Z (mạch cầu H)

Động cơ DC GA25-370 điều khiển qua **mạch cầu H (L298N / TB6612FNG)**:

| Tín hiệu | GPIO | Chức năng |
|---|---|---|
| MOTOR_IN1 | 4 | Chiều quay (cực 1) |
| MOTOR_IN2 | 5 | Chiều quay (cực 2) |
| MOTOR_PWM (ENA) | 6 | Tốc độ (PWM) |

- **PWM** tạo bằng **LEDC** kênh 4, tần số **20 kHz** (ngoài ngưỡng nghe → êm), độ phân giải **8-bit** (duty 0–255).
- Điều khiển kiểu **sign-magnitude**: IN1/IN2 quyết định chiều, ENA quyết định tốc độ. Khi dừng đúng đích, đặt **IN1=IN2=HIGH** để **phanh chủ động** (active brake), giữ vị trí chắc.

## 3.5 Khối encoder quadrature

- Encoder Hall 2 kênh **A (GPIO10)**, **B (GPIO11)**, cấu hình `INPUT_PULLUP`.
- Đọc **4× quadrature**: ngắt trên **cả hai kênh, sườn CHANGE** → 4 lần đếm mỗi chu kỳ tín hiệu, tăng độ phân giải.
- Trình phục vụ ngắt (ISR) đặt trong **IRAM** để chạy nhanh, dùng bảng tra Gray-code xác định chiều đếm (chi tiết ở 5.4.1).

> Nếu trục Z đếm **ngược chiều** (PID mất ổn định), chỉ cần **hoán đổi 2 chân encoder A/B** hoặc đảo IN1/IN2 — một trong hai là đủ.

## 3.6 Khối cảm biến VL53L0X và OLED (bus I2C)

Hai thiết bị dùng **chung một bus I2C**:

| Thiết bị | Địa chỉ | Chức năng |
|---|---|---|
| VL53L0X (ToF) | 0x29 | Đo khoảng cách bề mặt |
| OLED SSD1306 0.96" | 0x3C | Hiển thị trạng thái |

- Bus I2C: **SDA = GPIO40**, **SCL = GPIO41**, tốc độ **400 kHz** (Fast Mode).
- **VL53L0X:** nguyên lý Time-of-Flight (đo thời gian bay của tia laser hồng ngoại), dải đo ~30–2000 mm, **timing budget 33 ms** (cân bằng tốc độ/độ chính xác). Đọc kiểu **single-shot**.
- **OLED:** hiển thị bảng trạng thái (XY, Z, giác hút, khoảng cách, trạng thái Pick & Place), làm tươi mỗi **300 ms**.

## 3.7 Khối giác hút (van + relay bơm)

| Tín hiệu | GPIO | Trạng thái | Chức năng |
|---|---|---|---|
| PIN_SUCTION (van) | 42 | HIGH = hút, LOW = nhả | Điều khiển van khí |
| PIN_PUMP_RELAY (bơm) | 45 | active-HIGH | Bật/tắt bơm chân không |

> ⚠️ **GPIO45 là chân strapping (VDD_SPI):** lúc cấp nguồn, ROM bootloader giữ chân này ở một mức trong vài trăm ms → bơm có thể kêu nhẹ khi khởi động. Firmware gọi `gripper_init()` **sớm nhất** trong `setup()` để rút ngắn hiện tượng này, nhưng phần ROM boot không thể tránh hoàn toàn bằng phần mềm. Khắc phục triệt để bằng phần cứng: chọn điện trở kéo phù hợp chiều active ở chân IN của module relay, hoặc chuyển sang chân không-strapping.

## 3.8 Công tắc giới hạn và bảo vệ

- **Công tắc hành trình giới hạn trên** — GPIO **8**, cấu hình `INPUT_PULLUP`, **active-LOW** (nối GND, khi nhấn = mức LOW).
- Hai công dụng:
  1. **Dừng khẩn** khi đầu Z chạm đỉnh hành trình lúc đang đi lên (bảo vệ cơ khí).
  2. **Homing về đỉnh:** chạy lên chạm công tắc → chuẩn hóa `Z = Z_MAX` (mốc tham chiếu tin cậy hơn cảm biến ToF).
- Ngoài ra firmware còn có **giới hạn mềm** (software limit) kiểm tra vị trí encoder để không vượt hành trình.

## 3.9 Bảng phân công chân GPIO

**Bảng 3.1 — Bảng phân công chân GPIO tổng hợp**

| Chức năng | Tín hiệu | GPIO | Loại | Mô tả |
|---|---|---|---|---|
| Robot phẳng | SERVO1 | 48 | PWM 50 Hz | Động cơ 1 tại A(0,0) – trái |
| Robot phẳng | SERVO2 | 47 | PWM 50 Hz | Động cơ 2 tại C(55,0) – phải |
| Trục Z | MOTOR_IN1 | 4 | OUTPUT | Chiều quay H-bridge |
| Trục Z | MOTOR_IN2 | 5 | OUTPUT | Chiều quay H-bridge |
| Trục Z | MOTOR_PWM (ENA) | 6 | LEDC 20 kHz | Tốc độ PWM động cơ |
| Trục Z | ENC_A | 10 | INPUT ngắt | Encoder kênh A |
| Trục Z | ENC_B | 11 | INPUT ngắt | Encoder kênh B |
| Trục Z | LIMIT_TOP | 8 | INPUT_PULLUP | Công tắc giới hạn trên (active-LOW) |
| Cảm biến/Hiển thị | SDA | 40 | I2C | Dữ liệu I2C (VL53L0X + OLED) |
| Cảm biến/Hiển thị | SCL | 41 | I2C | Clock I2C (VL53L0X + OLED) |
| Giác hút | PIN_SUCTION | 42 | OUTPUT | Van hút/nhả |
| Giác hút | PIN_PUMP_RELAY | 45 | OUTPUT | Relay bơm ⚠️ strapping pin |

> **Ghi chú:** chú thích trong `z_axis.h` mô tả dây "C1→GPIO11, C2→GPIO10", nhưng khai báo `#define` là `Z_ENC_A=10, Z_ENC_B=11`. Cần **đối chiếu với đấu nối thực tế** và kiểm tra chiều đếm (mục 7.3.2, 7.7).

## 3.10 Nguồn cấp và ước tính công suất

Hệ thống dùng nhiều mức điện áp, cần tách nguồn hợp lý để tránh nhiễu và sụt áp:

**Bảng 3.2 — Ước tính công suất tiêu thụ (tham khảo)**

| Tải | Điện áp | Dòng điển hình | Ghi chú |
|---|---|---|---|
| ESP32-S3 (logic) | 3.3V (từ 5V USB) | ~0,1–0,25 A | Cấp qua USB hoặc LDO |
| 2× Servo RC | 5–6V | ~0,3–1 A mỗi cái (đỉnh khi tải) | Nguồn riêng, chung GND |
| Động cơ Z GA25-370 | 12V | ~0,3–0,6 A (đỉnh khởi động cao hơn) | Qua cầu H |
| Bơm chân không | 5–12V | tùy loại | Qua relay |
| VL53L0X + OLED | 3.3V | vài chục mA | Từ nguồn logic |

**Nguyên tắc thiết kế nguồn:**
- **Tách nguồn động lực** (servo, động cơ Z, bơm) khỏi **nguồn logic** để nhiễu đóng cắt không làm reset MCU.
- **Chung GND** toàn hệ thống.
- Bổ sung **tụ lọc** gần servo và cầu H để ổn định điện áp khi tải thay đổi đột ngột.
