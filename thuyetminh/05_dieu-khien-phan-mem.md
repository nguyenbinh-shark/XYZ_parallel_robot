# CHƯƠNG 5. THIẾT KẾ HỆ THỐNG ĐIỀU KHIỂN VÀ PHẦN MỀM

## 5.1 Kiến trúc phần mềm tổng thể

Firmware tổ chức theo mô hình **siêu vòng lặp hợp tác (cooperative super-loop)**, **phi chặn (non-blocking)**: mỗi module có một hàm `..._update()` thực hiện một phần việc nhỏ rồi trả điều khiển ngay, để `loop()` gọi lần lượt tất cả module ở tần suất cao. Nhờ đó nhiều tác vụ (nội suy servo, vòng PID trục Z, đọc cảm biến, máy trạng thái Pick & Place, truyền thông) chạy **"song song" trên một luồng** mà không cản trở nhau.

```
        ┌───────────────── loop() ─────────────────┐
        │  z_axis_update()     ← vòng PID trục Z    │
        │  servo_xy_update()   ← nội suy servo XY   │
        │  z_home_update()     ← homing/teach       │
        │  demo_update()       ← demo hình vuông    │
        │  pp_update()         ← máy trạng thái PP  │
        │  [OLED 300ms] [Telemetry] [Serial RX]     │
        └───────────────────────────────────────────┘
```
**Hình 5.1 — Kiến trúc phần mềm super-loop**

**Nguyên tắc cốt lõi:** **tuyệt đối tránh `delay()`** trong các đường điều khiển. `delay()` sẽ làm treo cả vòng lặp, khiến vòng PID trục Z trễ nhịp, servo giật, và lệnh `abort` không kịp phản hồi. Mọi việc "chờ" đều thay bằng so sánh mốc thời gian `millis()`.

Các module chính (mỗi module một cặp `.h/.cpp` trong `src/`): `ik_5bar` (động học ngược), `z_axis` (trục Z), `z_home` (homing/teach), `vl53l0x` (cảm biến), `gripper` (giác hút), `oled` (hiển thị), và `main.cpp` (điều phối + máy trạng thái + giao thức).

## 5.2 Chu trình setup() và loop()

### 5.2.1 setup() — khởi tạo

Thứ tự khởi tạo được sắp đặt có chủ đích:

1. `Serial.begin(115200)` — mở cổng giao tiếp.
2. **`gripper_init()` — gọi SỚM NHẤT.** Vì GPIO45 (relay bơm) là **chân strapping**, khởi tạo sớm để đưa relay về trạng thái tắt càng nhanh càng tốt, giảm hiện tượng bơm kêu lúc boot.
3. `delay(500)` — chờ nguồn ổn định (chỉ ở setup, không ảnh hưởng vòng điều khiển).
4. Gắn (attach) hai servo XY (GPIO48, GPIO47) ở 50 Hz, 500–2500 µs; đặt góc ban đầu 90°.
5. In banner thông tin (kích thước khâu L0–L4, vị trí HOME).
6. `vl53l0x_init()` (tùy chọn — cảnh báo nếu thiếu), `z_axis_init()`, `oled_init()`.
7. In trợ giúp lệnh, rồi `move_to(HOME_X, HOME_Y)` đưa robot về HOME.

### 5.2.2 loop() — vòng lặp chính

`loop()` gọi tuần tự (theo đúng thứ tự để ưu tiên điều khiển trước, hiển thị/truyền thông sau):

- `z_axis_update()` — vòng PID trục Z (phải chạy liên tục, ưu tiên cao).
- `servo_xy_update()` — nội suy servo XY (nhịp 20 ms).
- `z_home_update()` — máy trạng thái homing/teach.
- `demo_update()` — máy trạng thái demo.
- `pp_update()` — máy trạng thái Pick & Place.
- **OLED** làm tươi mỗi 300 ms (chỉ đọc VL53L0X khi PP rảnh, tránh phép đọc 33 ms làm trễ PID).
- **Telemetry** phát khung `@` theo chu kỳ (khi bật).
- **Serial RX** đọc theo dòng: dòng bắt đầu `$` → `proto_handle()` (giao thức máy); còn lại → `process()` (lệnh văn bản).

## 5.3 Module điều khiển XY

### 5.3.1 Ánh xạ góc hình học → góc servo

`ik_5bar()` cho góc hình học θ (CCW). Servo cần góc vật lý [0, 180°] theo chiều CW. Chuỗi biến đổi (trong `main.cpp`):

```
a = 180 − (θ + OFFSET)          // reverse_dir: CCW → CW; OFFSET = hiệu chỉnh lắp đặt
nếu SERVO2_INVERT: a = 180 − a  // chỉ servo 2 nếu lắp đối xứng gương
a = wrap360(a)                  // đưa về [0,360): fmod rồi cộng 360 nếu âm
hợp lệ nếu a ≤ 180, ngược lại (180,360) → ngoài hành trình → trả −1
```

Tham số (khai báo trong `main.cpp`):
- `SERVO1_OFFSET = −45°`, `SERVO2_OFFSET = +45°`: chọn sao cho HOME nằm trong tầm servo. Đây là **hiệu chuẩn vật lý** — phải khớp cách lắp còi servo.
- `SERVO2_INVERT = false`: hai động cơ cùng chiều.

> **`wrap360()` rất quan trọng:** hàm `atan`/`atan2` có thể trả góc lệch ±360°. Nếu so trực tiếp với [0,180] sẽ **từ chối nhầm** điểm hợp lệ ("quá giới hạn ảo"). Chuẩn hóa về [0,360) trước khi so mới cho kết quả đúng.

> **Đồng bộ 3 nơi:** offset ở **còi servo (vật lý)** ↔ `SERVO*_OFFSET` (firmware) ↔ `SV*_OFF` (Web HMI) phải **bằng nhau**, nếu không vị trí thật/hình vẽ sẽ lệch.

Hàm `move_to(x, y)` thực hiện: IK → ánh xạ servo → kiểm tra hợp lệ → **đặt góc đích** và khởi động nội suy (không ghi servo tức thì). Nếu điểm ngoài vùng làm việc, trả về `false` (host nhận `ERR,WORKSPACE`).

### 5.3.2 Nội suy đồng bộ hai servo

**Vấn đề của cách ghi tức thì:** ghi thẳng `s1.write/s2.write` ngay sau IK khiến hai servo slew ở tốc độ riêng, **kết thúc lệch thời điểm**. Trong chuỗi kín 5 khâu, tỉ lệ θ₁/θ₂ không khớp nghiệm IK ở quá độ → sinh **nội lực đối kháng** (binding, kêu, mòn khớp).

**Giải pháp:** nội suy tuyến tính theo thời gian, hai trục **chung một tham số `u ∈ [0,1]`**:

```
move_to(x, y):
    IK + ánh xạ → a1_goal, a2_goal
    a1_start = cur_a1;  a2_start = cur_a2            (xuất phát từ vị trí HIỆN TẠI)
    Δ = max(|a1_goal − a1_start|, |a2_goal − a2_start|)
    T_total = Δ × 1000 / SERVO_SPEED_DPS            (≥ 1 nhịp) → giới hạn vận tốc góc
    xy_t_start = millis();  xy_moving = true

servo_xy_update()   [gọi trong loop(), mỗi 20 ms]:
    u = (millis() − xy_t_start) / T_total,  kẹp [0,1]
    cur_a1 = a1_start + round((a1_goal − a1_start) × u)
    cur_a2 = a2_start + round((a2_goal − a2_start) × u)
    s1.write(cur_a1);  s2.write(cur_a2)
    nếu u ≥ 1 → xy_moving = false                   (tín hiệu "đã tới đích" thật)
```

Vì cả hai trục dùng chung `u`, chúng luôn **kết thúc cùng lúc** → triệt tiêu nội lực. Tham số: `SERVO_SPEED_DPS = 150 °/s` (vận tốc góc tối đa), `SERVO_XY_INTERVAL_MS = 20 ms` (khớp nhịp 50 Hz của servo).

| Đặc tính | Ghi tức thì (cũ) | Nội suy đồng bộ (dùng) |
|---|---|---|
| Đồng bộ 2 trục | Không | **Có** (chung `u`) |
| Nội lực 5 khâu | Nguy cơ binding | **Triệt tiêu** |
| Vận tốc | Không kiểm soát | Giới hạn `SERVO_SPEED_DPS` |
| Tín hiệu tới đích | Chờ mù | `xy_is_moving()`=false (thật) |

> **Giới hạn:** nội suy trong *không gian góc* nên đầu công tác đi đường **cong** (không thẳng Descartes), nhưng là đường cong **xác định, lặp lại được** — chấp nhận được với tầm di chuyển ngắn của Pick & Place.

## 5.4 Module điều khiển trục Z

Trục Z điều khiển **vòng kín**: encoder đo vị trí, bộ PD tính tín hiệu điều khiển, cầu H + PWM ra động cơ.

### 5.4.1 Giải mã encoder quadrature 4×

Ngắt sườn **CHANGE trên cả hai kênh A, B** → 4 lần đếm mỗi chu kỳ. Chiều đếm xác định bằng **bảng tra Gray-code** `QEM[16]` từ trạng thái (trước, hiện tại):

```
Bảng QEM (2-bit ab = A<<1 | B):
Trước\Hiện   00   01   10   11
    00        0   −1   +1    0
    01       +1    0    0   −1
    10       −1    0    0   +1
    11        0   +1   −1    0

ISR:  ab = (digitalRead(A) << 1) | digitalRead(B);
      enc_pos += QEM[(last_ab << 2) | ab];
      last_ab = ab;
```

ISR đặt trong **IRAM** (`IRAM_ATTR`) để chạy nhanh, `enc_pos` khai báo `volatile`. Vị trí quy đổi: `z_mm = enc_pos / Z_PULSE_PER_MM`.

### 5.4.2 Bộ điều khiển PD

Vòng điều khiển chạy mỗi **20 ms**. Sai số tính theo **số xung** (không cần quy đổi mm mỗi vòng):

```
error  = target_cnt − enc_pos            (target_cnt = z_mm × Z_PULSE_PER_MM)
deriv  = error − prev_error
output = Kp × error + Kd × deriv
pwm    = clamp(|output|, Z_PWM_MIN, Z_PWM_MAX)   // 55 … 220
motor_set( output ≥ 0 ? +pwm : −pwm )
```

**Bảng 5.1 — Tham số bộ điều khiển PD (`z_axis.h`)**

| Tham số | Ký hiệu | Giá trị | Vai trò |
|---|---|---|---|
| Hệ số tỉ lệ | Kp | 2.5 | Tốc độ tiếp cận đích |
| Hệ số đạo hàm | Kd | 0.08 | Giảm dao động/vọt lố khi đến đích |
| Vùng chết | Deadband | ±4 xung | Coi như "đã tới" khi sai số nhỏ |
| PWM tối thiểu | PWM_MIN | 55/255 | Thắng ma sát tĩnh |
| PWM tối đa | PWM_MAX | 220/255 | Bảo vệ dòng (~80% duty) |
| Chu kỳ | T | 20 ms | Nhịp vòng PD |

Không dùng khâu tích phân (I) vì tải nhẹ và có vít-me tự hãm; PD đủ để bám vị trí mà không tích lũy sai số. Khi `|error| ≤ deadband`, động cơ **phanh** (IN1=IN2=HIGH) và đặt cờ `_moving=false`.

```
              z_axis_update()  (20 ms)
                    │
         _moving? ──NO──► return
                    │YES
        error = target − enc_pos;  deriv = error − prev
                    │
   [ưu tiên] chạm công tắc đỉnh khi đi lên? ──► stop, Z=Z_MAX
                    │
        vi phạm giới hạn mềm Z_MIN/Z_MAX? ──► stop
                    │
        |error| ≤ 4 xung? ──YES──► phanh, _moving=false ("đã tới")
                    │NO
        output = Kp·error + Kd·deriv
        pwm = clamp(|output|, 55, 220)
        motor_set(±pwm)
```
**Hình 5.2 — Lưu đồ vòng điều khiển PD trục Z**

### 5.4.3 Chống vượt hành trình và an toàn

- **Công tắc đỉnh (ưu tiên cao nhất):** khi đang đi **lên** mà chạm công tắc → dừng ngay, chuẩn hóa `Z = Z_MAX`.
- **Giới hạn mềm:** kiểm tra `Z_MIN_MM ≤ vị trí ≤ Z_MAX_MM`; vượt → dừng.
- **Phanh chủ động** khi đến đích để giữ vị trí.
- Hàm phụ trợ: `z_axis_move_by` (tương đối), `z_axis_set_zero`, `z_axis_set_pos_mm` (đặt vị trí không chuyển động — dùng cho homing), `z_axis_raw_pwm` (điều khiển PWM trực tiếp, bỏ qua PID — dùng để kiểm tra chiều/hiệu chuẩn).

> **Hiệu chuẩn độ phân giải:** `Z_PULSE_PER_MM = PPR × 4 × Gear / Pitch`. Giá trị phụ thuộc bước vít-me thực tế — xem cảnh báo mục 7.3.2.

## 5.5 Module cảm biến và lọc tín hiệu

VL53L0X đọc kiểu **single-shot** (~33 ms/lần). Do tín hiệu ToF có thể có gai nhiễu (spike), module `vl53l0x` cung cấp nhiều mức lọc phù hợp từng ngữ cảnh:

| Hàm | Cơ chế | Dùng cho |
|---|---|---|
| `vl53l0x_read_mm()` | 1 lần đọc thô | Truy vấn nhanh, debug |
| `vl53l0x_read_avg(n=3)` | Trung bình n lần (delay 35 ms) | Ít dùng (nhạy spike) |
| `vl53l0x_read_median(n=5)` | **Trung vị** n lần | Homing, teach, dò vật (loại spike) |
| `vl53l0x_read_filtered()` | Trung vị 3 mẫu → **EMA** (α=0.40) | Giám sát liên tục (OLED) |
| `vl53l0x_filter_reset()` | Xóa bộ lọc | Đầu mỗi chu trình vận chuyển |

- **Trung vị (median)** loại bỏ giá trị nhảy bất thường → tốt cho đo tại vị trí cố định.
- **EMA (trung bình trượt mũ)** làm mượt số hiển thị; khi mất tín hiệu (−1) tạm thời thì **giữ giá trị đã lọc** trước đó.
- Trả về **−1** khi ngoài tầm đo/lỗi (`RangeStatus == 4`).

## 5.6 Module homing và teach

Trục Z không có mốc "0" tuyệt đối lúc bật máy → cần **homing** để chuẩn hóa vị trí. Quan hệ hình học: `Z_vật_lý = số_đọc_VL53L0X − Z_HOME_SENSOR_OFFSET`. Có ba chiến lược homing + chế độ teach, dùng chung máy trạng thái `z_home_update()`.

### 5.6.1 Homing tức thì (`z_home_instant`)

Nhanh nhất (~105 ms), **không chuyển động cơ khí**: đọc `median(5)`, tính `Z = số_đọc − offset` (kẹp ≥ 0), rồi **force-set** encoder bằng `z_axis_set_pos_mm(Z)`. Yêu cầu: bề mặt phía dưới nằm trong tầm nhìn cảm biến. Nếu cảm biến không đọc được → chuyển sang homing chậm.

### 5.6.2 Homing chậm theo bề mặt

Máy trạng thái: `IDLE → STEPPING → FOUND → LIFTING → DONE`.

```
z_home_slow_start():  giả sử robot ở đỉnh hành trình → set_pos_mm(Z_MAX)
                      → move_to(Z_MAX − 1mm), vào STEPPING

STEPPING (mỗi vòng, khi PID đã ổn định !z_moving):
    d = median(5); pos = vị trí hiện tại
    nếu d < Z_HOME_NEAR_MM        → FOUND (chạm bề mặt)
    ngược lại nếu pos ≤ 0.1        → DONE (hết hành trình, KHÔNG thấy bề mặt → lỗi)
    ngược lại                     → hạ thêm 1 mm: move_to(pos − 1)

FOUND (homing): set_zero() → move_to(Z_HOME_LIFT_MM=20) → LIFTING → DONE
```

Tham số (`z_home.h`): `Z_HOME_STEP_MM=1`, `Z_HOME_LIFT_MM=20`, `Z_HOME_NEAR_MM = offset+3`.

> **Cảnh báo an toàn:** homing chậm giả định robot đang ở **đỉnh hành trình**. Gọi khi Z ở giữa hành trình có thể khiến cơ cấu đâm xuống.

### 5.6.3 Homing theo công tắc đỉnh (`z_home_top_start`)

Tin cậy hơn ToF cho mốc tham chiếu: chạy **PWM thô** (bỏ qua PID) đi lên ở tốc độ nhẹ `Z_HOME_TOP_PWM=110` đến khi **chạm công tắc** → dừng, đặt `Z = Z_MAX`. Có **timeout 6 s** báo lỗi nếu không chạm.

### 5.6.4 Teach pick_z / place_z

Giống homing chậm về cách hạ dò, nhưng khi FOUND thì **ghi lại vị trí** vào biến (`pick_z` hoặc `place_z`) — **không** zero, **không** nâng. Dùng để "dạy" robot cao độ lấy/đặt chính xác.

## 5.7 Module giác hút

Giác hút được điều khiển **phi chặn** để không cản vòng PID:

- `gripper_suck()` — van HIGH + bật bơm (hút).
- `gripper_valve_open()` — mở van (LOW) tức thì, bơm **vẫn chạy** (dùng trong máy trạng thái).
- `gripper_pump_off()` — tắt bơm; gọi sau `GRIPPER_RELEASE_PUMP_DELAY_MS = 150 ms` (xả áp âm).
- `gripper_release()` — bản **chặn ~150 ms** cho lệnh thủ công (không dùng trong máy trạng thái).

**Phát hiện vật** `z_grip_has_object()`: đọc `median(3)`, trả `true` nếu `số_đọc < Z_GRIP_DETECT_MM (= offset+15)`. Nếu cảm biến lỗi → trả `true` (**fail-safe**: giả định có vật, tránh làm rơi nhầm).

## 5.8 Máy trạng thái Pick & Place

Đây là "bộ não" điều phối cả ba cụm chức năng. Cấu hình một tác vụ chứa trong `struct PPTask`: `pick(x,y,z)`, `place(x,y,z)`, `safe_z`, các mốc thời gian `xy_settle_ms`, `grip_ms`, `drop_ms`, và số lần thử lại `max_pick_retry`. Mặc định: pick=(27.5, 80, 5), place=(42.5, 80, 5), safe_z=20, times 500/400/300 ms, retry 3.

Máy trạng thái (`enum class PPState`) gồm 12 trạng thái:
`IDLE, PICK_XY, PICK_LOWER, PICK_GRIP, PICK_VERIFY, PICK_RAISE, PLACE_XY, PLACE_LOWER, PLACE_DROP, PLACE_RAISE, DROP_DETECTED, DONE`.

```
   "run" → IDLE
             │ nâng safe_z + move XY tới pick
             ▼
        PICK_XY   (chờ !z_moving && !xy_moving && settle)
             │ hạ Z từng bước, đọc median(5)
             ▼
        PICK_LOWER  ── d ≤ Z_PICK_NEAR (52mm) hoặc chạm pick_z → hút
             ▼
        PICK_GRIP   (chờ grip_ms)
             ▼
        PICK_VERIFY ── z_grip_has_object()? có → tiếp; không → repick/ NOOBJ
             │ nâng safe_z
             ▼
        PICK_RAISE  → move XY tới place
             ▼
        PLACE_XY   (giám sát rơi vật mỗi 300ms; !xy_moving && settle)
             ▼
        PLACE_LOWER → mở van (non-blocking)
             ▼
        PLACE_DROP  (tắt bơm sau 150ms; chờ drop_ms)
             ▼
        PLACE_RAISE → nâng safe_z
             ▼
          DONE → IDLE
   (lỗi gắp / rơi vật quá số lần) → DROP_DETECTED
```
**Hình 5.3 — Sơ đồ máy trạng thái Pick & Place**

**Các cơ chế thông minh:**
- **Dò vật khi hạ:** thay vì hạ mù đến `pick_z`, robot hạ **từng bước 1 mm**, đọc `median(5)`, dừng và hút khi cảm biến báo gần vật (`≤ Z_PICK_NEAR_MM`); `pick_z` chỉ là **giới hạn an toàn dưới**.
- **Xác nhận gắp:** sau khi hút, kiểm tra `z_grip_has_object()` để chắc chắn đã bám vật.
- **Giám sát rơi vật:** trong lúc vận chuyển (`PLACE_XY`), cứ 300 ms kiểm tra vật còn không; nếu rơi → gọi `pp_repick()` thử lại (tối đa `max_pick_retry` lần), quá số lần → `DROP_DETECTED`.
- Mỗi lần đổi trạng thái phát sự kiện `!ST,<state>` cho host (xem Chương 6).

Toàn bộ được điều khiển hợp tác từ `loop()` qua `pp_update()`, **không dùng `delay()`**, nên có thể `abort` bất cứ lúc nào.

## 5.9 Module hiển thị OLED

OLED SSD1306 hiển thị **một bảng trạng thái cố định** (không phải menu), làm tươi mỗi 300 ms:

```
=  5-Bar Robot v1  =
XY:  +27.5  +80.0 mm
Z :  +12.30 mm  ZERO
Grip:HUT   VL:  142mm
PP : IDLE
────────────────────
```

Nội dung: tọa độ XY, vị trí Z (kèm nhãn `ZERO` khi Z≈0), trạng thái giác hút (`HUT`/`NHA`), khoảng cách VL53L0X (`---` khi lỗi), và tên trạng thái Pick & Place. Để tránh phép đọc cảm biến 33 ms làm trễ vòng PID, OLED **chỉ đọc VL53L0X khi PP rảnh**; khi PP đang chạy thì dùng giá trị cache.

## 5.10 Xử lý lệnh người dùng

Hàm `process()` phân tích lệnh văn bản gõ tay qua Serial:

- **XY:** `x,y` (di chuyển), `home`, `demo`, `pos`, `dist`.
- **Z:** `z<mm>`, `z+<mm>`, `z-<mm>`, `z0`, `zstop`, `zpos`, `zenc`, `zraw <pwm>`.
- **Giác hút:** `suck`, `drop`, `pump on|off`.
- **Homing/Teach:** `homez`, `homez slow`, `hometop`, `lim?`, `teachpick`, `teachplace`, `grip?`.
- **Pick & Place:** `pick x,y,z`, `place x,y,z`, `safez`, `settle`, `grip`, `drop`, `retry`, `run`, `abort`, `ppinfo`.
- **Chung:** `help`.

Song song với `process()`, các dòng bắt đầu bằng `$` được định tuyến sang bộ giải mã **giao thức máy** (Chương 6) để điều khiển từ GUI/PC một cách tin cậy.
