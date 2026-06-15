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
| Robot phẳng | SERVO1 | 48 | PWM 50Hz | Động cơ 1 tại A(0,0) – trái |
| Robot phẳng | SERVO2 | 47 | PWM 50Hz | Động cơ 2 tại C(55,0) – phải |
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
   (0,0)  (55,0)

  Thông số:  L0 = 55 mm  (khoảng cách 2 trục)
             L1 = L4 = 70 mm   (khâu gần – nối với động cơ)
             L2 = L3 = 130 mm  (khâu xa – nối đầu công tác)
             (khớp file MATLAB Kinematic_and_workspace.m: l0=5.5 l1=7 l2=13 cm ×10)
```

- **Motor 1** tại A(0,0): góc θ₁ đo từ trục +X (gốc tọa độ = tâm động cơ trái)
- **Motor 2** tại C(55,0): góc θ₂ đo từ trục +X
- Quy ước: x sang ngang, y hướng lên; chiều dương động cơ = **thuận chiều kim đồng hồ**
- **Vùng làm việc**: giao của 2 vành khuyên (bán kính trong |L2−L1| = **60 mm**, ngoài
  L1+L2 = **200 mm**) quanh A và C; thực tế bị thu hẹp thêm bởi giới hạn góc servo [0,180]

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
                     │ Nội suy ĐỒNG BỘ theo thời gian  │  │  Servo PWM   │
                     │ u = (t−t_start)/T_total ∈ [0,1] │─►│  s1.write()  │
                     │ a = lerp(a_start, a_goal, u)    │  │  s2.write()  │
                     │(2 trục CHUNG u → kết thúc cùng )│  └──────────────┘
                     └─────────────────────────────────┘
```

> **Lưu ý quan trọng (cập nhật):** `move_to()` không còn ghi servo tức thì. Nó tính IK,
> đặt góc đích rồi để `servo_xy_update()` **nội suy dần** trong `loop()` (non-blocking).
> Hai servo dùng **chung tham số `u`** nên luôn **kết thúc cùng thời điểm** → triệt tiêu
> nội lực/binding trong chuỗi kín 5 khâu. Xem mục **2.6**.

### 2.3 Mô hình toán học – Động học ngược (IK)

Mỗi nhánh quy về giải phương trình `A·cosθ + B·sinθ = C`, nghiệm Weierstrass:
`θ = 2·atan( (B ± √(A²+B²−C²)) / (A+C) )`. Triển khai **bám đúng file MATLAB**.

**Nhánh trái – Motor 1 tại A(0,0):**

```
A = 2·L1·x ;  B = 2·L1·y ;  C = x² + y² + L1² − L2²
Điều kiện tồn tại (trong vùng làm việc): A² + B² − C² ≥ 0   (nếu < 0 → ngoài workspace)
θ1_(+) = 2·atan( (B + √disc)/(A+C) ) ;  θ1_(−) = 2·atan( (B − √disc)/(A+C) )
```

**Nhánh phải – Motor 2 tại C(L0,0):**

```
A = 2·L4·(x − L0) ;  B = 2·L4·y ;  C = (L0 − x)² + y² + L4² − L3²
θ2_(+), θ2_(−)  tính tương tự
```

**Chọn nghiệm theo dấu y (chọn 1 chế độ lắp – khuỷu hướng ra ngoài):**

```
nếu y ≥ 0:  θ1 = θ1_(+) ,  θ2 = θ2_(−)
nếu y < 0:  θ1 = θ1_(−) ,  θ2 = θ2_(+)
```

`ik_5bar()` trả về **góc hình học θ (CCW so với +X)** – thuần toán học, **chưa có offset**.
Việc đổi sang góc servo (offset + đảo chiều + chuẩn hóa) làm ở lớp `map_servo` (mục 2.7).

### 2.4 Bảng Input / Output

| | Tên | Kiểu dữ liệu | Đơn vị | Giá trị hợp lệ | Ghi chú |
|---|---|---|---|---|---|
| **INPUT** | x | float | mm | ~[−40, 95] | Tọa độ X (đo từ tâm động cơ trái) |
| **INPUT** | y | float | mm | ~[55, 190] | Tọa độ Y (>0), trong giao 2 vành khuyên |
| **OUTPUT** | θ1 | float | độ | góc hình học | Góc khâu L1 (CCW so với +X), CHƯA offset |
| **OUTPUT** | θ2 | float | độ | góc hình học | Góc khâu L4 (CCW so với +X), CHƯA offset |
| **OUTPUT** | return | uint8_t | – | 0 hoặc 1 | 1=giải được, 0=ngoài workspace (disc<0) |

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
                                     ┌──────────────────────── ─┐
                                     │ Đặt đích a1_goal/a2_goal │
                                     │ T_total = Δgóc_max /     │
                                     │   SERVO_SPEED_DPS        │
                                     │ xy_moving = true; return1│
                                     └──────────┬────────────── ┘
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

### 2.7 Ánh xạ góc hình học → góc servo (offset, đảo chiều, chuẩn hóa)

`ik_5bar()` cho góc hình học θ (CCW). Servo cần góc vật lý [0,180°] theo chiều CW. Chuỗi:

```
a = 180 − (θ + OFFSET)          // reverse_dir: CCW → CW; OFFSET = hiệu chỉnh lắp đặt
nếu SERVO2_INVERT: a = 180 − a  // chỉ servo 2 nếu lắp đối xứng gương
a = wrap360(a)                  // đưa về [0,360): fmod + cộng 360 nếu âm
hợp lệ nếu a ≤ 180, ngược lại (180,360) → ngoài hành trình servo → trả −1
```

- **OFFSET** (`SERVO1_OFFSET=−45`, `SERVO2_OFFSET=+45`): chọn để HOME nằm trong tầm servo.
  Đây là hiệu chuẩn **vật lý** – phải khớp cách lắp còi servo.
- **`wrap360()` rất quan trọng:** `atan` (và cả `atan2`) có thể trả góc lệch ±360° →
  nếu so trực tiếp `[0,180]` sẽ bị **"quá giới hạn ảo"** (từ chối điểm hợp lệ). Wrap rồi
  mới so giúp `atan`/`atan2` cho **cùng kết quả đúng**, loại trừ lỗi này.
- 3 nơi offset phải **đồng bộ**: còi servo (vật lý) ↔ `SERVO*_OFFSET` (firmware) ↔
  `SV*_OFF` (web HMI). Lệch nhau → vị trí thật / hình vẽ sai.

> "Giới hạn vùng làm việc" thực tế = giao của: **(1)** điều kiện hình học `disc≥0` trong
> `ik_5bar()`, và **(2)** góc servo (sau wrap) phải ≤180° trong `map_servo`. Cả hai chốt
> tại `move_to()` (trả false → host nhận `ERR,WORKSPACE`).

---

## 3. CHỨC NĂNG 2: ĐIỀU KHIỂN ĐỘ CAO – TRỤC Z

### 3.1 Mô tả phần cứng

| Thành phần | Thông số |
|---|---|
| Động cơ | GA25-370, 12VDC, tỉ số truyền 21.3:1, 280 RPM |
| Encoder | Hall sensor 2 kênh AB, 11 xung/kênh/vòng |
| Vít me | Bước vít (lead) **8 mm/vòng** |
| H-bridge | L298N / TB6612FNG |
| PWM | LEDC 20 kHz, 8-bit (0–255) |
| Độ phân giải | 11 × 4 × 21.3 / 8.0 ≈ **117.15 xung/mm** |

### 3.2 Sơ đồ khối phần mềm – Chức năng 2

```
                    ┌──────────────────────────────────────────────┐
                    │              z_axis_update()  (20ms)         │
                    │                                              │
  Lệnh (mm) ──────► │  target_cnt = z_mm × 117.15                  │
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
| Vùng chết | Deadband | ±4 xung | Dừng khi sai số nhỏ (~0.034mm) |
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
- Relay bơm: GPIO 45, **active-HIGH** (`PUMP_RELAY_ACTIVE_HIGH = true`): HIGH=bơm chạy,
  LOW=tắt ⚠️ **strapping pin** – xem ghi chú dưới
- Delay nhả bơm: 150ms (xả áp âm trước khi tắt bơm)

> ⚠️ **GPIO45 là chân strapping (VDD_SPI):** lúc cấp nguồn ROM bootloader giữ chân
> này ở mức nhất định (~vài trăm ms) → bơm có thể kêu nhẹ lúc boot. Firmware gọi
> `gripper_init()` sớm nhất trong `setup()` để rút ngắn, nhưng phần ROM boot không
> tránh được bằng phần mềm. Khắc phục triệt để bằng phần cứng (điện trở kéo phù hợp
> chiều active ở chân IN module relay), hoặc đổi sang chân không-strapping.

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
|**OUTPUT**|PIN_PUMP_RELAY|digital|45  | active-HIGH      | Relay bơm (strap) |
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
  → 27.5,80            [Di chuyển XY đến vị trí lấy (trong workspace)]
  → teachpick          [Hạ Z tự động đến bề mặt, ghi pick_z]

Bước 3: Đặt vị trí XY đặt hàng
  → 42.5,80            [Di chuyển XY đến vị trí đặt]
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
| L0 | ik_5bar.h | 55 mm | Khoảng cách 2 trục động cơ |
| L1, L4 | ik_5bar.h | 70 mm | Khâu gần (nối động cơ) |
| L2, L3 | ik_5bar.h | 130 mm | Khâu xa (nối đầu công tác) |
| HOME (X,Y) | main.cpp | (27.5, 80) mm | Vị trí home (giữa 2 ĐC, trong workspace) |
| Servo offset 1/2 | main.cpp | −45° / +45° | Hiệu chỉnh góc lắp đặt (PHẢI khớp web `SV*_OFF`) |
| Servo invert 2 | main.cpp | false | Đảo chiều servo 2 (lắp đối xứng) |
| Servo speed | main.cpp | 150 °/s | Tốc độ góc tối đa khi nội suy XY |
| Servo XY tick | main.cpp | 20 ms | Chu kỳ cập nhật nội suy servo |
| Z gear ratio | z_axis.h | 21.3 | Tỉ số truyền hộp số |
| Z screw pitch | z_axis.h | 8.0 mm | Bước vít me (lead) |
| Z pulses/mm | z_axis.h | ≈117.15 | 11×4×21.3/8.0 |
| Z_MAX_MM | z_axis.h | 50 mm | Hành trình tối đa |
| Kp, Kd | z_axis.h | 2.5, 0.08 | Hệ số PD controller |
| Sensor offset | z_home.h | 8 mm | Khoảng cách VL53L0X–đầu hút |
| Grip detect | z_home.h | 23 mm | Ngưỡng phát hiện vật |
| Settle time | main.cpp | 500 ms | Thời gian ổn định servo XY |
| Grip time | main.cpp | 400 ms | Thời gian bám giác hút |
| Drop time | main.cpp | 300 ms | Thời gian nhả vật |
| Pump delay | gripper.h | 150 ms | Delay tắt bơm sau mở van |
| Pump relay | gripper.h | active-HIGH | `PUMP_RELAY_ACTIVE_HIGH=true` |

---

## 7. GIAO THỨC GIAO TIẾP SERIAL (host ↔ ESP32)

Ngoài lệnh text gõ tay, firmware hỗ trợ **giao thức máy có khung + checksum** để
GUI/PC/MCU điều khiển tin cậy. Hai đường chạy **song song**: dòng bắt đầu bằng `$`
được định tuyến sang bộ giải mã giao thức (`proto_handle()`), còn lại vào `process()`.

```
Host -> ESP :  $<seq>,<CMD>,<arg...>*<CC>      vd: $1,MOVE,27.5,80.0*05
ESP  -> Host:  #<seq>,OK|ERR,...*<CC>          ACK/NAK (echo seq để khớp lệnh)
               @<STATE>,X,Y,Z,GRIP,DIST,XYMV,ZMV,LIM,S1,S2*CC  telemetry định kỳ
               !<EV>,...*<CC>                  sự kiện bất đồng bộ (đổi state…)
```

> Telemetry có thêm **S1, S2** = góc servo thật (0–180°) đang gửi tới động cơ → web dùng
> để hiển thị góc + vẽ cơ cấu theo động học thuận.

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

---

## 8. GIAO DIỆN ĐIỀU KHIỂN TRÊN WEB (HMI)

File `hmi/index.html` là một **giao diện điều khiển chạy thẳng trên trình duyệt**, kết nối
ESP32 qua **Web Serial API** (không cần cài đặt, không cần server). Mở bằng **Microsoft
Edge** hoặc **Google Chrome** (Firefox không hỗ trợ Web Serial).

### 8.1 Kiến trúc

```
┌─────────────────────────┐   Web Serial (USB-CDC, 115200)   ┌──────────────┐
│   Trình duyệt (HMI)     │ ◄──────────────────────────────► │   ESP32-S3   │
│                         │   $seq,CMD,..*CC   /   #,@,!      │  (firmware)  │
│  • Gửi khung lệnh       │ ───────────────────────────────► │              │
│  • Đọc ACK/telemetry/EV │ ◄─────────────────────────────── │              │
│  • Vẽ trạng thái + sơ đồ│                                  └──────────────┘
└─────────────────────────┘
```

- Dùng **đúng giao thức khung + checksum** ở mục 7 (`send()` tự thêm `seq` và `*CC`).
- Khi kết nối, HMI tự gửi `VER`, bật sự kiện `EVENTS,1`, và bật telemetry `TLM,200`
  (stream mỗi 200ms). Có watchdog báo nếu mở COM nhưng không nhận được dữ liệu.

### 8.2 Bảng trạng thái (đọc từ telemetry)

| Mục hiển thị | Nguồn (telemetry) | Ghi chú |
|---|---|---|
| X, Y (mm) | X, Y | Tọa độ đầu công tác (lệnh) |
| Z (mm) | Z | Vị trí trục Z |
| VL53L0X (mm) | DIST | Khoảng cách cảm biến (−1 = lỗi) |
| Động cơ 1 / 2 (°) | S1, S2 | **Góc servo thật** đang gửi động cơ |
| X / Y từ servo (mm) | tính từ S1,S2 | Động học **thuận** (kiểm tra khớp X/Y) |
| State P&P | STATE | IDLE / PICK_* / PLACE_* … |
| Đèn báo | GRIP, LIM, XYMV, ZMV | Giác hút/Bơm, giới hạn trên, XY chạy, Z chạy |

- **Sơ đồ cơ cấu (canvas):** vẽ 5 khâu **từ góc servo thật** (S1,S2) → đảo offset
  (`servoToTheta`) ra góc hình học → động học thuận (giao 2 đường tròn `circInt`) ra
  đầu công tác. Có đánh dấu điểm pick (P) và place (D).
- **Quan trọng:** hằng số `SV1_OFF/SV2_OFF` trong web **phải bằng** `SERVO*_OFFSET`
  của firmware, nếu không hình + "X/Y từ servo" sẽ lệch.

### 8.3 Khối điều khiển (gửi lệnh xuống ESP32)

| Khu vực | Nút / thao tác | Lệnh gửi |
|---|---|---|
| **Jog XY** | bước nhảy 0.5/1/5/10 mm + pad X±/Y± + HOME | `MOVE x,y` / `HOME` |
| | ô X,Y + GO | `MOVE x,y` |
| | phím mũi tên ← → ↑ ↓ | jog XY (tương đương pad) |
| **Trục Z** | bước 0.5/2/5 + Z▲/Z▼ | `ZR ±step` |
| | ô Z tuyệt đối + GO | `Z mm` |
| | Z STOP / Z = 0 | `ZSTOP` / `ZZERO` |
| | PageUp / PageDown | jog Z |
| **Giác hút / Bơm** | HÚT / NHẢ | `SUCK` / `DROP` |
| | BẬT BƠM / TẮT BƠM | `PUMP 1` / `PUMP 0` |
| **Homing/Teach** | HOMEZ / HOMEZ chậm / HOME ĐỈNH | `HOMEZ` / `HOMEZS` / `HOMETOP` |
| | TEACH pick / place / Đọc PPINFO | `TEACHPICK` / `TEACHPLACE` / `PPINFO` |
| **Pick & Place** | nhập pick/place (x,y,z) + SET | `PICK` / `PLACE` |
| | SafeZ/Settle/Grip/Drop + ÁP DỤNG | `SAFEZ/SETTLE/GRIPMS/DROPMS` |
| | ▶ RUN / ■ ABORT | `RUN` / `ABORT` |
| **E-STOP** (nút đỏ lớn) | dừng khẩn cấp | `ABORT` |

### 8.4 Console

Khung console cuối trang log mọi khung gửi/nhận (TX màu xanh, ACK xanh lá, lỗi đỏ,
telemetry xám, sự kiện vàng) – tiện debug giao thức và theo dõi phản hồi robot.

### 8.5 Lưu ý vận hành

- Mỗi lần đổi offset servo ở firmware → **đổi luôn `SV*_OFF` trong `index.html`** cho khớp.
- Telemetry dùng giá trị DIST cache khi Pick&Place đang chạy (tránh đọc VL53L0X 33ms gây
  trễ vòng PID trục Z).
- Web **không tự giới hạn** vùng làm việc; mọi điểm được gửi xuống, firmware quyết định
  hợp lệ hay trả `ERR,WORKSPACE`.
