# PHỤ LỤC

> Các giá trị số trong phụ lục lấy theo **mã nguồn firmware hiện tại**. Những thông số đánh dấu ⚠️ cần hiệu chuẩn theo phần cứng thực tế (xem mục 7.7).

## PHỤ LỤC A — Bảng tổng hợp thông số cài đặt

| Thông số | File | Giá trị (mã nguồn) | Ý nghĩa |
|---|---|---|---|
| L0 | ik_5bar.h | 55 mm | Khoảng cách 2 trục động cơ |
| L1, L4 | ik_5bar.h | 70 mm | Khâu gần (nối động cơ) |
| L2, L3 | ik_5bar.h | 130 mm | Khâu xa (nối đầu công tác) |
| HOME (X, Y) | main.cpp | (27.5, 80) mm | Vị trí home |
| SERVO1/2_OFFSET | main.cpp | −45° / +45° | Hiệu chỉnh lắp (PHẢI khớp `SV*_OFF` HMI) |
| SERVO2_INVERT | main.cpp | false | Đảo chiều servo 2 |
| SERVO_SPEED_DPS | main.cpp | 150 °/s | Vận tốc góc tối đa nội suy XY |
| SERVO_XY_INTERVAL_MS | main.cpp | 20 ms | Chu kỳ nội suy servo |
| SERVO_HZ / US_MIN / US_MAX | main.cpp | 50 Hz / 500 / 2500 µs | Cấu hình xung servo |
| Z_ENC_PPR | z_axis.h | 11 | Xung/kênh/vòng (trước hộp số) |
| Z_GEAR_RATIO | z_axis.h | 21.3 | Tỉ số truyền hộp số |
| Z_SCREW_PITCH | z_axis.h | ⚠️ 2.0 mm | Bước vít-me (hiệu chuẩn) |
| Z_PULSE_PER_MM | z_axis.h | ⚠️ ≈ 468,6 xung/mm | = PPR·4·Gear/Pitch |
| Z_MIN_MM / Z_MAX_MM | z_axis.h | ⚠️ −100 / +100 mm | Giới hạn hành trình |
| Z_KP / Z_KD | z_axis.h | 2.5 / 0.08 | Hệ số PD |
| Z_DEADBAND | z_axis.h | ±4 xung | Vùng chết |
| Z_PWM_MIN / MAX | z_axis.h | 55 / 220 (của 255) | Giới hạn PWM |
| Z_PID_INTERVAL_MS | z_axis.h | 20 ms | Chu kỳ PD |
| Z_LEDC_FREQ / RES | z_axis.h | 20 kHz / 8-bit | PWM động cơ Z |
| Z_HOME_SENSOR_OFFSET_MM | z_home.h | ⚠️ 50 mm | Khoảng cách VL53L0X → đầu hút |
| Z_HOME_NEAR_MM | z_home.h | offset+3 (=53) | Ngưỡng "chạm bề mặt" |
| Z_PICK_NEAR_MM | z_home.h | offset+2 (=52) | Ngưỡng "gần vật" khi hạ |
| Z_GRIP_DETECT_MM | z_home.h | offset+15 (=65) | Ngưỡng "có vật" |
| Z_HOME_STEP_MM / LIFT_MM | z_home.h | 1 / 20 mm | Bước hạ / nâng sau homing |
| Z_HOME_TOP_PWM / TIMEOUT | z_home.h | 110 / 6000 ms | Homing về đỉnh |
| VL53L0X timing budget | vl53l0x.cpp | 33 ms | Thời gian đo 1 mẫu |
| VL_EMA_ALPHA | vl53l0x.cpp | 0.40 | Hệ số làm mượt EMA |
| I2C SDA/SCL/FREQ | vl53l0x.h | 40 / 41 / 400 kHz | Bus I2C chung |
| pp_task (mặc định) | main.cpp | pick(27.5,80,5), place(42.5,80,5), safe_z=20 | Cấu hình Pick & Place |
| xy_settle / grip / drop | main.cpp | 500 / 400 / 300 ms | Các mốc thời gian PP |
| max_pick_retry | main.cpp | 3 | Số lần thử gắp lại |
| GRIPPER_RELEASE_PUMP_DELAY_MS | gripper.h | 150 ms | Delay tắt bơm sau mở van |
| PUMP_RELAY_ACTIVE_HIGH | gripper.h | true | Relay bơm active-HIGH |

## PHỤ LỤC B — Sơ đồ đấu nối và bảng chân GPIO

Xem **Bảng 3.1** (Chương 3) cho bảng phân công chân đầy đủ. Tóm tắt nhanh:

```
ESP32-S3
 ├─ GPIO48 ── Servo 1 (XY, động cơ A/trái)
 ├─ GPIO47 ── Servo 2 (XY, động cơ C/phải)
 ├─ GPIO4,5,6 ── Cầu H (IN1, IN2, ENA/PWM) ── Động cơ Z (GA25-370)
 ├─ GPIO10,11 ── Encoder A, B (Z)
 ├─ GPIO8  ── Công tắc giới hạn trên (INPUT_PULLUP, active-LOW)
 ├─ GPIO40,41 ── I2C (SDA, SCL) ── VL53L0X (0x29) + OLED SSD1306 (0x3C)
 ├─ GPIO42 ── Van giác hút (HIGH=hút)
 └─ GPIO45 ── Relay bơm (active-HIGH, ⚠️ strapping)
```

## PHỤ LỤC C — Trích mã nguồn chính

### C.1 Động học ngược 5 khâu (`src/ik_5bar.c`)

```c
#define RAD2DEG 57.29577951f

/* Giải A*cos(t) + B*sin(t) = C  →  t = 2*atan((B ± s)/(A+C)),  s = sqrt(A²+B²−C²) */
static uint8_t angle(float A, float B, float C, float *t1, float *t2) {
    float disc = A*A + B*B - C*C;
    if (disc < 0.0f) return 0;                 // ngoài workspace
    float s = sqrtf(disc);
    float denom = A + C;
    *t1 = 2.0f * atanf((B + s) / denom);
    *t2 = 2.0f * atanf((B - s) / denom);
    return 1;
}

uint8_t ik_5bar(float x, float y, float *theta1_deg, float *theta2_deg) {
    float theta1_t1, theta1_t2, theta2_t1, theta2_t2;

    /* Nhánh trái (động cơ 1 tại A) */
    float a = 2.0f * x * IK_L1;
    float b = 2.0f * y * IK_L1;
    float c = x*x + y*y + IK_L1*IK_L1 - IK_L2*IK_L2;
    if (!angle(a, b, c, &theta1_t1, &theta1_t2)) return 0;

    /* Nhánh phải (động cơ 2 tại C) */
    float d = 2.0f * (x - IK_L0) * IK_L4;
    float e = 2.0f * y * IK_L4;
    float f = (IK_L0 - x)*(IK_L0 - x) + y*y + IK_L4*IK_L4 - IK_L3*IK_L3;
    if (!angle(d, e, f, &theta2_t1, &theta2_t2)) return 0;

    /* Chọn nghiệm theo dấu y (chế độ lắp "khuỷu ra ngoài") */
    float theta1, theta2;
    if (y >= 0.0f) { theta1 = theta1_t1; theta2 = theta2_t2; }
    else           { theta1 = theta1_t2; theta2 = theta2_t1; }

    *theta1_deg = theta1 * RAD2DEG;            // góc hình học CCW, chưa offset
    *theta2_deg = theta2 * RAD2DEG;
    return 1;
}
```

### C.2 Vòng điều khiển PD trục Z (rút gọn, `src/z_axis.cpp`)

```c
void z_axis_update() {
    if (!_moving) return;
    if (millis() - last_pid_ms < Z_PID_INTERVAL_MS) return;   // 20 ms
    last_pid_ms = millis();

    long pos   = enc_pos;                       // snapshot vị trí
    long error = target_cnt - pos;
    long deriv = error - prev_error;  prev_error = error;

    // 1) Ưu tiên công tắc đỉnh khi đi lên → dừng, chuẩn hóa Z = Z_MAX
    // 2) Giới hạn mềm Z_MIN/Z_MAX → dừng
    if (labs(error) <= Z_DEADBAND) { motor_set(0); _moving = false; return; }  // đã tới

    float out = Z_KP * error + Z_KD * deriv;
    int pwm = constrain((int)fabs(out), Z_PWM_MIN, Z_PWM_MAX);
    motor_set(out >= 0 ? pwm : -pwm);
}
```

### C.3 Máy trạng thái Pick & Place (khai báo, `src/main.cpp`)

```cpp
enum class PPState : uint8_t {
    IDLE, PICK_XY, PICK_LOWER, PICK_GRIP, PICK_VERIFY, PICK_RAISE,
    PLACE_XY, PLACE_LOWER, PLACE_DROP, PLACE_RAISE, DROP_DETECTED, DONE
};

struct PPTask {
    float pick_x, pick_y, pick_z;
    float place_x, place_y, place_z;
    float safe_z;
    uint16_t xy_settle_ms, grip_ms, drop_ms;
    uint8_t  max_pick_retry;
};
```

## PHỤ LỤC D — Đặc tả giao thức serial

Đặc tả đầy đủ (khung bản tin, bảng lệnh, telemetry, sự kiện, mã lỗi, ví dụ host Python và phiên làm việc mẫu) được trình bày trong file [`PROTOCOL.md`](../PROTOCOL.md) của dự án. Tóm tắt các loại khung:

```
$<seq>,<CMD>,<args>*<CC>      Host → ESP  (lệnh)
#<seq>,OK|ERR,...*<CC>        ESP → Host  (ACK/NAK)
@<11 trường>*<CC>            ESP → Host  (telemetry định kỳ)
!<EV>,...*<CC>               ESP → Host  (sự kiện bất đồng bộ)
```

## PHỤ LỤC E — Tài liệu tham khảo

1. Espressif Systems, *ESP32-S3 Technical Reference Manual & Datasheet*.
2. STMicroelectronics, *VL53L0X Datasheet — World's smallest Time-of-Flight ranging sensor*.
3. Adafruit, *Adafruit_VL53L0X* và *Adafruit SSD1306 / GFX* library documentation.
4. K. Kozak, *ESP32Servo Library* (madhephaestus/ESP32Servo).
5. J. J. Craig, *Introduction to Robotics: Mechanics and Control*, Pearson.
6. L. Tsai, *Robot Analysis: The Mechanics of Serial and Parallel Manipulators*, Wiley.
7. Tài liệu nội bộ dự án: `BAOCAO.md`, `PROTOCOL.md`, `Kinematic_and_workspace.m`, `hmi/README.md`.
8. NMEA 0183 checksum (XOR) — tham khảo định dạng khung kiểm tra lỗi.

---

*— Hết —*
