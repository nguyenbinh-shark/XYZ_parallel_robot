# BÁO CÁO KỸ THUẬT LẬP TRÌNH
## Robot Song Song 5 Khâu – Hệ thống Pick & Place
**Nền tảng:** ESP32-S3 | **Framework:** Arduino (PlatformIO)

---

## 1. TỔNG QUAN HỆ THỐNG

### 1.1 Sơ đồ khối tổng thể

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
     │  GPIO 47, 48  │      │  DC Motor    │   │ Van GPIO 42  │
     │               │      │  Encoder A,B │   │ Relay GPIO 45│
     └───────────────┘      └──────────────┘   └──────────────┘
```

### 1.2 Bảng chân kết nối tổng hợp

| Chức năng | Tên tín hiệu | GPIO | Loại | Mô tả |
|---|---|---|---|---|
| Robot phẳng | SERVO1 | 47 | PWM 50Hz | Động cơ servo tại A(0,0) |
| Robot phẳng | SERVO2 | 48 | PWM 50Hz | Động cơ servo tại C(38,0) |
| Trục Z | MOTOR_IN1 | 4 | OUTPUT | Chiều quay H-bridge |
| Trục Z | MOTOR_IN2 | 5 | OUTPUT | Chiều quay H-bridge |
| Trục Z | MOTOR_PWM (ENA) | 6 | LEDC 20kHz | Tốc độ PWM động cơ |
| Trục Z | ENC_A (C1) | 11 | INPUT ngắt | Encoder kênh A |
| Trục Z | ENC_B (C2) | 10 | INPUT ngắt | Encoder kênh B |
| Trục Z | LIMIT_TOP | 8 | INPUT_PULLUP | Công tắc hành trình giới hạn trên (active-LOW) |
| Cảm biến | SDA | 40 | I2C | Bus I2C dữ liệu (VL53L0X + OLED) |
| Cảm biến | SCL | 41 | I2C | Bus I2C clock (VL53L0X + OLED) |
| Hiển thị | OLED SSD1306 | 40/41 | I2C 0x3C | Màn 0.96" – dùng chung bus I2C |
| Giác hút | PIN_SUCTION | 42 | OUTPUT | Van điều khiển hút/đẩy |
| Giác hút | PUMP_RELAY | 45 | OUTPUT | Relay bật/tắt bơm ⚠️ strapping pin |

---

## 2. CHỨC NĂNG 1: ROBOT PHẲNG 5 KHÂU (XY)

### 2.1 Mô tả cơ cấu

Robot song song 5 khâu (five-bar linkage) với 2 khớp chủ động:

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
   (0,0)  (38,0)

  Thông số:  L0 = 38 mm  (khoảng cách 2 trục)
             L1 = L2 = L3 = L4 = 50 mm
```

- **Motor 1** tại A(0,0): góc θ₁ đo từ trục +X
- **Motor 2** tại C(38,0): góc θ₂ đo từ trục +X
- **Vùng làm việc**: xấp xỉ 40 × 60 mm (Y > 0)

### 2.2 Sơ đồ khối phần mềm – Chức năng 1

```
┌──────────────┐     ┌────────────────────────┐     ┌──────────────┐
│  Input       │     │   Khối xử lý           │     │  Output      │
│  (x, y) mm   │────►│   Bài toán động học    │────►│  θ1, θ2 (°)  │
│              │     │   ngược IK             │     │              │
└──────────────┘     │   ik_5bar(x,y,&t1,&t2) │     └──────┬───────┘
                     └────────────────────────┘            │
                                                           ▼
                     ┌────────────────────────┐     ┌──────────────────────┐
                     │   Ánh xạ góc           │     │ move_to(): đặt ĐÍCH  │
                     │   map_servo1(θ1)       │────►│ a1_goal/a2_goal      │
                     │   map_servo2(θ2)       │     │ (KHÔNG ghi tức thì)  │
                     └────────────────────────┘     └──────────┬───────────┘
                                                                │
                            [loop() gọi servo_xy_update() mỗi 20ms]
                                                                │
                                                                ▼
                     ┌─────────────────────────────────┐  ┌──────────────┐
                     │ Nội suy ĐỒNG BỘ theo thời gian   │  │  Servo PWM   │
                     │ u = (t−t_start)/T_total ∈ [0,1]  │─►│  s1.write()  │
                     │ a = lerp(a_start, a_goal, u)      │  │  s2.write()  │
                     │ (2 trục CHUNG u → kết thúc cùng lúc)│ └──────────────┘
                     └─────────────────────────────────┘
```

> **Lưu ý quan trọng (cập nhật):** `move_to()` không còn ghi servo tức thì. Nó tính IK,
> đặt góc đích rồi để `servo_xy_update()` **nội suy dần** trong `loop()` (non-blocking).
> Hai servo dùng **chung tham số `u`** nên luôn **kết thúc cùng thời điểm** → triệt tiêu
> nội lực/binding trong chuỗi kín 5 khâu. Xem mục **2.6**.

### 2.3 Mô hình toán học – Động học ngược (IK)

**Với Motor 1 tại A(0,0):**

```
c₁ = x² + y² + L1² − L2²
d₁ = 2·L1·x
e₁ = 2·L1·y

Điều kiện tồn tại: d₁² + e₁² − c₁² ≥ 0

θ₁ = 2·atan2( e₁ + √(d₁² + e₁² − c₁²) , d₁ + c₁ )
```

**Với Motor 2 tại C(L0, 0):**

```
dx = x − L0
c₂ = dx² + y² + L4² − L3²
d₂ = 2·L4·dx
e₂ = 2·L4·y

θ₂ = 2·atan2( e₂ − √(d₂² + e₂² − c₂²) , d₂ + c₂ )   [nhánh âm]
```

**Chuẩn hóa:** Nếu θ < 0 → θ += 2π để đưa về [0°, 360°)

### 2.4 Bảng Input / Output

| | Tên | Kiểu dữ liệu | Đơn vị | Giá trị hợp lệ | Ghi chú |
|---|---|---|---|---|---|
| **INPUT** | x | float | mm | [0, 38] | Tọa độ X đầu công tác |
| **INPUT** | y | float | mm | [10, 90] | Tọa độ Y đầu công tác |
| **OUTPUT** | θ1 | float → int | độ | [0, 180] | Góc servo motor 1 |
| **OUTPUT** | θ2 | float → int | độ | [0, 180] | Góc servo motor 2 |
| **OUTPUT** | return | uint8_t | – | 0 hoặc 1 | 1=thành công, 0=ngoài workspace |

### 2.5 Flowchart – Thuật toán IK

```
            ┌─────────────────────┐
            │   Vào: (x, y) mm    │
            └──────────┬──────────┘
                       │
                       ▼
            ┌─────────────────────┐
            │ Tính c1, d1, e1     │
            │ (Motor 1 tại A)     │
            └──────────┬──────────┘
                       │
                       ▼
              ┌────────────────┐
              │ d1²+e1²−c1²≥0? │
              └───┬────────┬───┘
                 NO        YES
                  │         │
                  ▼         ▼
            ┌──────────┐  ┌──────────────────┐
            │ return 0 │  │ Tính θ1 = 2·atan2│
            │ (lỗi)    │  └────────┬─────────┘
            └──────────┘           │
                                   ▼
                        ┌─────────────────────┐
                        │ Tính c2, d2, e2     │
                        │ (Motor 2 tại C)     │
                        └──────────┬──────────┘
                                   │
                                   ▼
                          ┌────────────────┐
                          │d2²+e2²−c2²≥0?  │
                          └───┬────────┬───┘
                             NO        YES
                              │         │
                              ▼         ▼
                        ┌──────────┐  ┌──────────────────┐
                        │ return 0 │  │ Tính θ2 = 2·atan2│
                        └──────────┘  └────────┬─────────┘
                                               │
                                               ▼
                                     ┌─────────────────────┐
                                     │ Chuẩn hóa θ1,θ2     │
                                     │ về [0°, 360°)       │
                                     └──────────┬──────────┘
                                                │
                                                ▼
                                     ┌─────────────────────┐
                                     │ Ánh xạ → góc servo  │
                                     │ + clamp [0°, 180°]  │
                                     └──────────┬──────────┘
                                                │
                                                ▼
                                     ┌─────────────────────────┐
                                     │ Đặt đích a1_goal/a2_goal│
                                     │ T_total = Δgóc_max /     │
                                     │   SERVO_SPEED_DPS        │
                                     │ xy_moving = true; return1│
                                     └──────────┬──────────────┘
                                                │
                                  [servo_xy_update() nội suy trong loop()]
```

### 2.6 Điều khiển chuyển động servo – Nội suy đồng bộ (non-blocking)

**Vấn đề của cách ghi tức thì (trước đây):** `s1.write/s2.write` ngay sau IK khiến 2 servo
slew ở tốc độ riêng, **kết thúc lệch thời điểm**. Trong chuỗi kín 5 khâu, tỉ lệ θ₁/θ₂ không
khớp nghiệm IK ở quá độ → sinh **nội lực đối kháng** (binding, buzz, mòn gear, có thể mất bước).

**Giải pháp – nội suy tuyến tính theo thời gian, 2 trục chung tham số `u`:**

```
move_to(x,y):
    IK + map → a1_goal, a2_goal           (validate đích: ngoài workspace → return 0)
    a1_start = cur_a1;  a2_start = cur_a2  (xuất phát từ vị trí HIỆN TẠI)
    Δ = max(|a1_goal−a1_start|, |a2_goal−a2_start|)
    T_total = Δ × 1000 / SERVO_SPEED_DPS   (≥ 1 tick) → vận tốc góc ≤ giới hạn
    xy_t_start = millis();  xy_moving = true

servo_xy_update()  [loop(), mỗi 20ms]:
    u = (millis() − xy_t_start) / T_total,  kẹp [0,1]
    cur_a1 = a1_start + round((a1_goal − a1_start) × u)
    cur_a2 = a2_start + round((a2_goal − a2_start) × u)
    s1.write(cur_a1);  s2.write(cur_a2)
    nếu u ≥ 1 → xy_moving = false           (tín hiệu "đã tới đích" THẬT)
```

| Đặc tính | Cách cũ (ghi tức thì) | Cách mới (nội suy đồng bộ) |
|---|---|---|
| Đồng bộ 2 trục | Không (lệch thời điểm) | **Có** (chung `u` → cùng lúc) |
| Nội lực 5 khâu | Có nguy cơ binding | **Triệt tiêu** |
| Vận tốc | Không kiểm soát (giật) | Giới hạn `SERVO_SPEED_DPS` |
| Tín hiệu tới đích | Chờ mù `settle_ms` | `xy_is_moving()` = false (thật) |
| Chặn `loop()` | `delay()` trong demo | Non-blocking hoàn toàn |

> **Giới hạn còn lại:** nội suy trong *không gian góc* nên đầu cuối vẫn đi đường cong (không
> phải đường thẳng Descartes), nhưng là đường cong **xác định, lặp lại được**. Với tầm di
> chuyển ngắn của pick-place thì an toàn. Nếu cần đường thẳng/tránh va chạm → nội suy Cartesian
> (chạy IK mỗi bước, kiểm điểm trung gian trong workspace) — chưa cần ở phiên bản này.

---

## 3. CHỨC NĂNG 2: ĐIỀU KHIỂN ĐỘ CAO – TRỤC Z

### 3.1 Mô tả phần cứng

| Thành phần | Thông số |
|---|---|
| Động cơ | GA25-370, 12VDC, tỉ số truyền 21.3:1, 280 RPM |
| Encoder | Hall sensor 2 kênh AB, 11 xung/kênh/vòng |
| Vít me | Bước vít 2 mm/vòng |
| H-bridge | L298N / TB6612FNG |
| PWM | LEDC 20 kHz, 8-bit (0–255) |
| Độ phân giải | 11 × 4 × 21.3 / 2.0 ≈ **468.6 xung/mm** |

### 3.2 Sơ đồ khối phần mềm – Chức năng 2

```
                    ┌──────────────────────────────────────────────┐
                    │              z_axis_update()  (20ms)         │
                    │                                              │
  Lệnh (mm) ──────► │  target_cnt = z_mm × 468.6                   │
                    │         │                                    │
                    │         ▼                                    │
  Encoder ISR ─────►│  error = target_cnt − enc_pos                │
  (ngắt A, B)       │  deriv = error − prev_error                  │
                    │         │                                    │
                    │         ▼                                    │
                    │  output = Kp×error + Kd×deriv                │
                    │         │                                    │
                    │         ▼                                    │
                    │  PWM = clamp(|output|, 55, 220)              │
                    │         │                                    │
                    └─────────┼────────────────────────────────────┘
                              │
                              ▼
                    ┌─────────────────────┐
                    │   motor_set(±PWM)   │
                    │   H-bridge IN1, IN2 │
                    │   LEDC duty cycle   │
                    └─────────────────────┘
```

### 3.3 Bộ giải mã Encoder – Quadrature 4x

Ngắt CHANGE trên **cả 2 kênh A và B** → 4 lần mỗi chu kỳ:

```
Bảng trạng thái QEM (Gray-code 2-bit):
┌───────┬────┬────┬────┬────┐
│Trước\ │ 00 │ 01 │ 10 │ 11 │  ← Trạng thái hiện tại (A<<1 | B)
│Hiện   │    │    │    │    │
├───────┼────┼────┼────┼────┤
│  00   │  0 │ -1 │ +1 │  0 │
│  01   │ +1 │  0 │  0 │ -1 │
│  10   │ -1 │  0 │  0 │ +1 │
│  11   │  0 │ +1 │ -1 │  0 │
└───────┴────┴────┴────┴────┘
enc_pos += QEM[(last_ab << 2) | ab]
```

### 3.4 Bộ điều khiển PD

| Tham số | Ký hiệu | Giá trị | Vai trò |
|---|---|---|---|
| Hệ số tỉ lệ | Kp | 2.5 | Tốc độ tiếp cận đích |
| Hệ số đạo hàm | Kd | 0.08 | Giảm dao động khi đến đích |
| Vùng chết | Deadband | ±4 xung | Dừng khi sai số nhỏ (~0.009mm) |
| PWM tối thiểu | PWM_MIN | 55/255 | Thắng ma sát tĩnh |
| PWM tối đa | PWM_MAX | 220/255 | Bảo vệ dòng điện |
| Chu kỳ | T | 20 ms | Tần số vòng lặp PD |

### 3.5 Bảng Input / Output

| | Tên | Kiểu | Đơn vị | Giá trị hợp lệ | Ghi chú |
|---|---|---|---|---|---|
| **INPUT** | z_mm | float | mm | [0.0, 50.0] | Vị trí tuyệt đối cần đến |
| **INPUT** | enc_A | digital | – | 0 / 1 | Tín hiệu encoder kênh A |
| **INPUT** | enc_B | digital | – | 0 / 1 | Tín hiệu encoder kênh B |
| **OUTPUT** | IN1, IN2 | digital | – | HIGH/LOW | Chiều quay H-bridge |
| **OUTPUT** | PWM duty | uint8_t | – | [0, 255] | Tốc độ động cơ (LEDC) |
| **OUTPUT** | enc_pos | int32_t | xung | – | Vị trí hiện tại (volatile) |
| **OUTPUT** | _moving | bool | – | true/false | Trạng thái đang di chuyển |

### 3.6 Flowchart – Vòng điều khiển PD (z_axis_update)

```
              ┌────────────────────────┐
              │  z_axis_update()       │
              │  Gọi mỗi 20ms từ loop  │
              └───────────┬────────────┘
                          │
                          ▼
                  ┌───────────────┐
                  │ _moving=true? │──NO──► return
                  └───────┬───────┘
                          │YES
                          ▼
              ┌────────────────────────┐
              │ pos = enc_pos (snapshot│
              │ error = target − pos   │
              │ deriv = error−prev_err │
              │ prev_error = error     │
              └───────────┬────────────┘
                          │
                          ▼
              ┌────────────────────────┐
              │ Kiểm tra giới hạn mềm  │──vi phạm──► stop(), cảnh báo
              │ Z_MIN ≤ pos ≤ Z_MAX    │
              └───────────┬────────────┘
                          │
                          ▼
                ┌─────────────────┐
                │|error| ≤ 4 xung?│──YES──► motor_set(0), _moving=false
                └────────┬────────┘
                         │NO
                         ▼
              ┌────────────────────────┐
              │ output=Kp×err+Kd×deriv │
              └───────────┬────────────┘
                          │
                          ▼
              ┌────────────────────────┐
              │ pwm=clamp(|out|,55,220)│
              └───────────┬────────────┘
                          │
                          ▼
              ┌────────────────────────┐
              │ out≥0 → motor tiến     │
              │ out<0 → motor lùi      │
              │ motor_set(±pwm)        │
              └────────────────────────┘
```

### 3.7 Flowchart – Ngắt Encoder (ISR)

```
    ┌─────────────────────────────┐
    │ Ngắt CHANGE trên A hoặc B   │
    └─────────────┬───────────────┘
                  │
                  ▼
    ┌─────────────────────────────┐
    │ ab = (digitalRead(A)<<1)    │
    │      | digitalRead(B)       │
    └─────────────┬───────────────┘
                  │
                  ▼
    ┌─────────────────────────────┐
    │ idx = (last_ab << 2) | ab   │
    │ enc_pos += QEM[idx]         │
    │ last_ab = ab                │
    └─────────────────────────────┘
```

---

## 4. CHỨC NĂNG 3: CẢM BIẾN VL53L0X + GIÁC HÚT

### 4.1 Mô tả phần cứng

**Cảm biến khoảng cách VL53L0X:**
- Giao tiếp: I2C (400kHz, **SDA=40, SCL=41**), địa chỉ 0x29
- Nguyên lý: Time-of-Flight (phát–thu laser)
- Dải đo: 30–2000 mm, timing budget 33ms
- Lắp đặt: Trên đầu công tác, **nhìn xuống**

**Màn hình OLED SSD1306 0.96":**
- Giao tiếp: I2C 0x3C, **dùng chung bus với VL53L0X** (SDA=40, SCL=41)
- Hiển thị: vị trí XY/Z, trạng thái giác hút, khoảng cách VL53L0X, state Pick&Place
- Cập nhật: mỗi 300ms trong `loop()`; **không đọc VL53L0X khi PP đang chạy** (tránh
  double-read 33ms×2 làm trễ vòng PID trục Z)

**Giác hút chân không:**
- Van điện từ: GPIO 42 (HIGH = hút, LOW = nhả/đẩy)
- Relay bơm: GPIO 45 (active-LOW theo mặc định) ⚠️ **strapping pin** – xem ghi chú dưới
- Delay nhả bơm: 150ms (xả áp âm trước khi tắt bơm)

> ⚠️ **GPIO45 là chân strapping (VDD_SPI):** lúc cấp nguồn ROM bootloader giữ chân
> này ở mức LOW (~vài trăm ms). Với relay **active-LOW**, bơm có thể kêu trong khoảng
> thời gian boot. Firmware gọi `gripper_init()` sớm nhất trong `setup()` để rút ngắn,
> nhưng phần ROM boot không thể tránh bằng phần mềm. Khắc phục triệt để bằng phần cứng:
> thêm điện trở **kéo lên (pull-up) 10kΩ** ở chân IN của module relay, hoặc dùng relay
> **active-HIGH** (đặt `PUMP_RELAY_ACTIVE_HIGH = true`), hoặc đổi sang chân không-strapping.

### 4.2 Hệ tọa độ VL53L0X – Nguyên lý homing

```
┌─────────────────────────────────────────┐
│                                         │
│  VL53L0X (gắn trên đầu công tác)        │
│       │  ↕ Z_HOME_SENSOR_OFFSET_MM      │
│  Đầu giác hút                           │
│       │  ↕ h (chiều cao thực)           │
│  ════════════════ Bề mặt làm việc       │
│                                         │
│  VL53L0X đọc = h + SENSOR_OFFSET        │
│  ⟹  Z_vật_lý = đọc − SENSOR_OFFSET     │
└─────────────────────────────────────────┘
```

### 4.3 Sơ đồ khối – Chức năng 3

```
┌─────────────────────────────────────────────────────┐
│                   Khối VL53L0X                      │
│                                                     │
│  I2C Bus ──► vl53l0x_read_mm()  ──► khoảng cách mm  │
│             vl53l0x_read_avg(3) ──► TB 3 lần đọc    │
└──────────────────────────┬──────────────────────────┘
                           │
              ┌────────────┴─────────────┐
              ▼                          ▼
┌──────────────────────┐    ┌────────────────────── ───┐
│   Homing / Teach     │    │   Phát hiện vật          │
│                      │    │                          │
│  Instant:            │    │  z_grip_has_object()     │
│  Z = đọc − offset    │    │  true nếu đọc <          │
│  z_axis_set_pos_mm() │    │  Z_GRIP_DETECT_MM (23mm) │
│                      │    │                          │
│  Slow: hạ 1mm/bước   │    │  Gọi sau PICK_GRIP       │
│  đến khi phát hiện   │    │  để xác nhận hút thành   │
│  bề mặt              │    │  công                    │
└──────────────────────┘    └───────────────────────── ┘

┌──────────────────────────────────────────────────── ─┐
│                   Khối Giác Hút                      │
│                                                      │
│  gripper_suck()       → van HIGH + relay ON          │
│  gripper_valve_open() → van LOW   (bơm vẫn chạy)     │
│  gripper_pump_off()   → relay OFF (sau 150ms)        │
│  gripper_release()    → van LOW + delay + relay OFF  │
└───────────────────────────────────────────────────── ┘
```

### 4.4 Bảng Input / Output

**VL53L0X:**

|          |Tên       |Kiểu   |Đơn vị|Giá trị      | Ghi chú                     |
|--        |--        |--     |---   |---          |---                          |
|**INPUT** |SDA/SCL   |I2C    | –    | –           | Bus I2C 400kHz              |
|**OUTPUT**|distance  |int16_t|mm    |[0, 2000]or-1| -1 = ngoài tầm đo           |
|**OUTPUT**|is_near   |bool   | –    |true/false   | true nếu < Z_HOME_NEAR_MM   |
|**OUTPUT**|has_object|bool   | –    |true/false   | true nếu < Z_GRIP_DETECT_MM |

**Giác hút:**

|          | Tên          | Kiểu  |GPIO| Trạng thái       | Ghi chú           |
|---       |---           |---    |--- |---               |---                |
|**INPUT** |lệnh          |enum   | –  | suck/drop        | Từ state machine  |
|**OUTPUT**|PIN_SUCTION   |digital|42  | HIGH=hút,LOW=nhả | Điều khiển van    |
|**OUTPUT**|PIN_PUMP_RELAY|digital|45  | active-LOW       | Relay bơm (strap) |
|**STATE** |_sucking      |bool   | –  | true/false       | Trạng thái nội bộ |

### 4.5 Flowchart – Homing tức thì (z_home_instant)

```
          ┌──────────────────────────┐
          │  Lệnh: homez             │
          └───────────┬──────────────┘
                      │
                      ▼
          ┌──────────────────────────┐
          │ Đọc VL53L0X 3 lần        │
          │ (delay 35ms giữa mỗi lần)│
          │ Lấy trung bình           │
          └───────────┬──────────────┘
                      │
                      ▼
            ┌─────────────────┐
            │ Giá trị hợp lệ? │──NO──► Chuyển sang Slow Homing
            └────────┬────────┘
                     │YES
                     ▼
          ┌──────────────────────────┐
          │ Z_mm = đọc − OFFSET(8mm) │
          │ clamp [0, Z_MAX]         │
          └───────────┬──────────────┘
                      │
                      ▼
          ┌──────────────────────────┐
          │ z_axis_set_pos_mm(Z_mm)  │
          │ (force-set encoder pos)  │
          └───────────┬──────────────┘
                      │
                      ▼
          ┌──────────────────────────┐
          │ Hoàn thành – Z đã chuẩn  │
          └──────────────────────────┘
```

### 4.6 Flowchart – Homing chậm (z_home_slow_start + z_home_update)

```
       ┌───────────────────────────  ──┐
       │ z_home_slow_start()           │
       │ Giả sử robot ở vị trí cao nhất│
       │ set_pos_mm(Z_MAX=50mm)        │
       │ move_to(49mm)                 │
       └──────────────┬──────────────  ┘
                      │
                  [loop() gọi z_home_update()]
                      │
                      ▼
       ┌─────────────────────────────┐
       │ State: STEPPING             │
       │ z_axis_is_moving()?         │──YES──► chờ
       └──────────────┬──────────────┘
                      │NO (PID đã ổn định)
                      ▼
       ┌─────────────────────────────┐
       │ Đọc VL53L0X                 │
       └──────────────┬──────────────┘
                      │
          ┌───────────┴──────────────┐
         YES                        NO
    đọc < Z_HOME_NEAR_MM?       pos > 0 mm?
          │                          │
          ▼                         YES
   ┌────────────┐                    │
   │  FOUND:    │          ┌─────────────────┐
   │ set_zero() │          │ Hạ thêm 1mm     │
   │ move_to(20)│          │ move_to(pos−1mm)│
   └─────┬──────┘          └─────────────────┘
         │
    [LIFTING: chờ Z đến 20mm]
         │
         ▼
   ┌────────────┐
   │  DONE:     │
   │ Z=0 = bề   │
   │ mặt làm    │
   │ việc       │
   └────────────┘
```

---

## 5. TÍCH HỢP: CHUỖI PICK & PLACE

### 5.1 Sơ đồ khối tích hợp 3 chức năng

```
┌──────────────────────────────────────────────────────────────────┐
│                    State Machine Pick & Place                    │
│                                                                  │
│  Input: lệnh "run" + tọa độ pick(x,y,z) + place(x,y,z)           │
│                                                                  │
│   ┌──────┐   ┌──────────┐   ┌──────────┐   ┌──────────────────┐  │
│   │CF1:  │   │CF2:      │   │CF3:      │   │ Kiểm soát trạng  │  │
│   │XY    │◄──│Z-axis    │◄──│VL53L0X   │   │ thái tổng thể    │  │ 
│   │servo │   │PD ctrl   │   │+ Gripper │   │ (pp_update)      │  │
│   └──────┘   └──────────┘   └──────────┘   └──────────────────┘  │
└──────────────────────────────────────────────────────────────────┘
```

### 5.2 Sơ đồ chuyển trạng thái Pick & Place

```
                        ┌──────────┐
     lệnh "run"         │          │
   ─────────────────►   │   IDLE   │ ◄── hoàn thành / abort
                        └────┬─────┘
                             │ z_move(safe_z) + xy_move(pick)
                             ▼
                        ┌──────────┐
                        │ PICK_XY  │ đk: !z_moving && !xy_moving && elapsed≥500ms
                        └────┬─────┘
                             │ z_move(pick_z)
                             ▼
                       ┌───────────┐
                       │PICK_LOWER │ điều kiện: !z_moving
                       └─────┬─────┘
                             │ gripper_suck()
                             ▼
                       ┌───────────┐
                       │ PICK_GRIP │ điều kiện: elapsed ≥ grip_ms
                       └─────┬─────┘
                             │ [đọc VL53L0X]
                             ▼
                      ┌────────────┐
                      │PICK_VERIFY │ điều kiện: elapsed ≥ 80ms
                      └─────┬──────┘
                            │ Có vật → OK │ Không → cảnh báo, vẫn tiếp
                            │ z_move(safe_z)
                            ▼
                       ┌───────────┐
                       │PICK_RAISE │ điều kiện: !z_moving
                       └─────┬─────┘
                             │ xy_move(place)
                             ▼
                       ┌───────────┐
                       │ PLACE_XY  │ đk: !xy_moving && elapsed ≥ 500ms
                       └─────┬─────┘
                             │ z_move(place_z)
                             ▼
                      ┌────────────┐
                      │PLACE_LOWER │ điều kiện: !z_moving
                      └─────┬──────┘
                            │ gripper_valve_open() ← non-blocking
                            ▼
                      ┌────────────┐   t≥150ms → pump_off()
                      │ PLACE_DROP │   điều kiện: elapsed ≥ drop_ms
                      └─────┬──────┘
                            │ z_move(safe_z)
                            ▼
                      ┌────────────┐
                      │PLACE_RAISE │ điều kiện: !z_moving
                      └─────┬──────┘
                            │
                            ▼
                        ┌──────────┐
                        │   DONE   │ ──────► IDLE
                        └──────────┘
```

### 5.3 Bảng Input / Output – Pick & Place tổng thể

| Giai đoạn | Input (điều kiện chuyển)     | Hành động Output                 |
|--        -|                              |---                               |
| PICK_XY   |`!z_moving`AND`!xy_moving`AND`elapsed≥500ms`| `z_move(pick_z)`   |
| PICK_LOWER|`!z_moving`                   | `gripper_suck()`→van HIGH, bơm ON|
| PICK_GRIP |`elapsed ≥ grip_ms` (400ms)   | –                                |
|PICK_VERIFY|`elapsed ≥ 80ms`              | Đọc VL53L0X, in kết quả          |
| PICK_RAISE|`!z_moving`                   | `xy_move(place_x, place_y)`      |
| PLACE_XY  |`!xy_moving`AND`elapsed ≥500ms`| `z_move(place_z)`               |
|PLACE_LOWER|`!z_moving`                   | `gripper_valve_open()` → van LOW |
|PLACE_DROP |`elapsed ≥ 150ms`             | `pump_off()`                     |
|PLACE_DROP |`elapsed ≥ drop_ms` (300ms)   | `z_move(safe_z)`                 |
|PLACE_RAISE|`!z_moving`                   | → DONE                           |

### 5.4 Workflow sử dụng tiêu chuẩn

```
Bước 1: Cấp nguồn
  → homez              [Chuẩn hóa Z bằng VL53L0X, ~105ms]

Bước 2: Đặt vị trí XY lấy hàng
  → 19,45              [Di chuyển XY đến vị trí lấy]
  → teachpick          [Hạ Z tự động đến bề mặt, ghi pick_z]

Bước 3: Đặt vị trí XY đặt hàng
  → 34,45              [Di chuyển XY đến vị trí đặt]
  → teachplace         [Hạ Z tự động đến bề mặt, ghi place_z]

Bước 4: Xác nhận thông số
  → ppinfo             [Kiểm tra toàn bộ tọa độ và thời gian]

Bước 5: Thực hiện
  → run                [Bắt đầu chuỗi Pick & Place]
  → abort              [Dừng khẩn cấp bất kỳ lúc nào]
```

---

## 6. BẢNG TỔNG HỢP THÔNG SỐ CÀI ĐẶT

| Thông số | File cài đặt | Giá trị | Ý nghĩa |
|---|---|---|---|
| L0, L1–L4 | ik_5bar.h | 38, 50 mm | Chiều dài các khâu |
| Servo offset | main.cpp | 0° | Hiệu chỉnh góc lắp đặt |
| Servo speed | main.cpp | 150 °/s | Tốc độ góc tối đa khi nội suy XY |
| Servo XY tick | main.cpp | 20 ms | Chu kỳ cập nhật nội suy servo |
| Z gear ratio | z_axis.h | 21.3 | Tỉ số truyền hộp số |
| Z screw pitch | z_axis.h | 2.0 mm | Bước vít me |
| Z_MAX_MM | z_axis.h | 50 mm | Hành trình tối đa |
| Kp, Kd | z_axis.h | 2.5, 0.08 | Hệ số PD controller |
| Sensor offset | z_home.h | 8 mm | Khoảng cách VL53L0X–đầu hút |
| Grip detect | z_home.h | 23 mm | Ngưỡng phát hiện vật |
| Settle time | main.cpp | 500 ms | Thời gian ổn định servo XY |
| Grip time | main.cpp | 400 ms | Thời gian bám giác hút |
| Drop time | main.cpp | 300 ms | Thời gian nhả vật |
| Pump delay | gripper.h | 150 ms | Delay tắt bơm sau mở van |

---

## 7. GIAO THỨC GIAO TIẾP SERIAL (host ↔ ESP32)

Ngoài lệnh text gõ tay, firmware hỗ trợ **giao thức máy có khung + checksum** để
GUI/PC/MCU điều khiển tin cậy. Hai đường chạy **song song**: dòng bắt đầu bằng `$`
được định tuyến sang bộ giải mã giao thức (`proto_handle()`), còn lại vào `process()`.

```
Host -> ESP :  $<seq>,<CMD>,<arg...>*<CC>      vd: $1,MOVE,19.0,45.0*05
ESP  -> Host:  #<seq>,OK|ERR,...*<CC>          ACK/NAK (echo seq để khớp lệnh)
               @<STATE>,X,Y,Z,GRIP,DIST,...*CC telemetry định kỳ (lệnh TLM)
               !<EV>,...*<CC>                  sự kiện bất đồng bộ (đổi state…)
```

- **`<CC>`** = XOR (2 hex) mọi byte giữa marker đầu và `*` (kiểu NMEA). Chiều
  Host→ESP có thể bỏ `*CC` (tiện gõ tay); ESP kiểm tra khi có và trả `ERR,CRC` nếu sai.
- **Mã lỗi:** `CRC, ARGS, WORKSPACE, RANGE, BUSY, CMD`.
- **Bảng lệnh đầy đủ, telemetry, sự kiện và ví dụ host Python:** xem [`PROTOCOL.md`](PROTOCOL.md).

| Nhóm lệnh | CMD chính |
|---|---|
| Truy vấn/chung | `PING, VER, STATUS, TLM, EVENTS, DIST, GRIPQ, LIMQ, PPINFO, ABORT` |
| Cấu hình PP | `PICK, PLACE, SAFEZ, SETTLE, GRIPMS, DROPMS, RUN` |
| Homing/Teach | `HOMEZ, HOMEZS, HOMETOP, TEACHPICK, TEACHPLACE` |
| Thủ công | `MOVE, HOME, DEMO, Z, ZR, ZSTOP, ZZERO, SUCK, DROP, PUMP` |
