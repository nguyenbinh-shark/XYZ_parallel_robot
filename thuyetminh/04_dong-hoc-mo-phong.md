# CHƯƠNG 4. ĐỘNG HỌC VÀ MÔ PHỎNG

## 4.1 Hệ tọa độ và mô hình hóa cơ cấu

Đặt hệ tọa độ Oxy trong mặt phẳng làm việc với **gốc tọa độ trùng tâm động cơ 1** tại A(0, 0); động cơ 2 tại C(L0, 0). Các điểm và khâu (Hình 4.1):

- A(0, 0), C(L0, 0): trục của hai động cơ chủ động.
- B: khớp nối L1–L2 (khuỷu trái); D: khớp nối L4–L3 (khuỷu phải).
- E(x, y): đầu công tác — giao điểm của hai khâu xa.
- θ₁: góc khâu L1 (A→B) so với trục +X; θ₂: góc khâu L4 (C→D) so với trục +X.

```
         E (x, y)
        / \
      L2   L3
      /     \
     B       D
      \     /
      L1   L4
        \ /
     A───────C
   (0,0)   (L0,0)
```
**Hình 4.1 — Mô hình toán học và hệ tọa độ**

- **Động học ngược (IK):** cho tọa độ E(x, y) → tìm góc θ₁, θ₂. Đây là bài toán **chính** dùng trong điều khiển.
- **Động học thuận (FK):** cho θ₁, θ₂ → tìm E(x, y). Dùng để **kiểm chứng và mô phỏng** (vẽ cơ cấu trên HMI).

## 4.2 Bài toán động học ngược

Ý tưởng: mỗi khuỷu (B, D) đồng thời cách **tâm động cơ** một đoạn (L1 hoặc L4) và cách **đầu công tác E** một đoạn (L2 hoặc L3). Từ đó viết ràng buộc khoảng cách và rút ra phương trình cho từng khâu.

### 4.2.1 Nhánh trái — Động cơ 1 tại A(0, 0)

Khớp B nằm trên khâu L1: `B = (L1·cosθ₁, L1·sinθ₁)`. Ràng buộc |EB| = L2:

$$(x - L_1\cos\theta_1)^2 + (y - L_1\sin\theta_1)^2 = L_2^2$$

Khai triển và rút gọn (chú ý `cos² + sin² = 1`), thu được phương trình tuyến tính theo cosθ₁, sinθ₁:

$$\underbrace{2 L_1 x}_{A}\cos\theta_1 + \underbrace{2 L_1 y}_{B}\sin\theta_1 = \underbrace{x^2 + y^2 + L_1^2 - L_2^2}_{C}$$

Vậy hệ số:

```
A = 2·L1·x
B = 2·L1·y
C = x² + y² + L1² − L2²
```

### 4.2.2 Nhánh phải — Động cơ 2 tại C(L0, 0)

Tương tự với khớp D trên khâu L4 quanh tâm C và ràng buộc |ED| = L3:

```
A = 2·L4·(x − L0)
B = 2·L4·y
C = (L0 − x)² + y² + L4² − L3²
```

Cả hai nhánh đưa về cùng dạng phương trình lượng giác `A·cosθ + B·sinθ = C`.

## 4.3 Giải phương trình lượng giác bằng tang nửa góc

Phương trình `A·cosθ + B·sinθ = C` được giải bằng phép thế **tang nửa góc (Weierstrass)**: đặt `t = tan(θ/2)`, khi đó `cosθ = (1−t²)/(1+t²)`, `sinθ = 2t/(1+t²)`. Thay vào và rút gọn được phương trình bậc hai theo t, cho nghiệm:

$$\theta = 2\cdot\arctan\!\left(\frac{B \pm \sqrt{A^2 + B^2 - C^2}}{A + C}\right)$$

Đặt **định thức (discriminant)**: `disc = A² + B² − C²`.

- Nếu **disc < 0**: không có nghiệm thực → điểm (x, y) **nằm ngoài không gian làm việc** của nhánh đó → trả về lỗi.
- Nếu **disc ≥ 0**: có hai nghiệm ứng với dấu `+` và `−` của căn, tương ứng **hai chế độ lắp** (khuỷu gập vào hoặc duỗi ra).

Đoạn mã lõi (`src/ik_5bar.c`):

```c
static uint8_t angle(float A, float B, float C, float *t1, float *t2) {
    float disc = A*A + B*B - C*C;
    if (disc < 0.0f) return 0;              // ngoài workspace
    float s = sqrtf(disc);
    float denom = A + C;
    *t1 = 2.0f * atanf((B + s) / denom);    // nghiệm +
    *t2 = 2.0f * atanf((B - s) / denom);    // nghiệm −
    return 1;
}
```

## 4.4 Chọn nghiệm và chế độ lắp

Mỗi nhánh cho 2 nghiệm → tổ hợp 4 dáng cơ cấu. Để cơ cấu hoạt động **nhất quán** (không "lật" khuỷu giữa chừng), phải cố định **một chế độ lắp** — ở đây chọn kiểu **"khuỷu hướng ra ngoài"** bằng quy tắc theo dấu của y:

```
nếu y ≥ 0:  θ₁ = θ₁(+) ,  θ₂ = θ₂(−)
nếu y < 0:  θ₁ = θ₁(−) ,  θ₂ = θ₂(+)
```

Hàm `ik_5bar()` trả về **góc hình học θ (đo ngược chiều kim đồng hồ, CCW, so với +X)** — thuần toán học, **chưa có offset lắp đặt**. Việc chuyển sang góc servo vật lý (đảo chiều, offset, chuẩn hóa) được xử lý ở lớp ánh xạ servo trong firmware (mục 5.3.1). Hàm trả về `1` nếu giải được, `0` nếu một trong hai nhánh ngoài không gian làm việc.

## 4.5 Động học thuận

Động học thuận dùng để **vẽ mô phỏng** và kiểm chứng: cho góc θ₁, θ₂ tìm E(x, y).

1. Tính hai khuỷu:
   - `B = (L1·cosθ₁, L1·sinθ₁)`
   - `D = (L0 + L4·cosθ₂, L4·sinθ₂)`
2. Đầu công tác E là **giao của hai đường tròn**: tâm B bán kính L2 và tâm D bán kính L3.
3. Chọn nghiệm giao điểm phù hợp chế độ lắp (phía trên đường BD).

Trong giao diện Web (`hmi/index.html`), hàm `circInt(B, L2, D, L3)` thực hiện phép giao hai đường tròn này để vẽ cơ cấu 5 khâu theo thời gian thực từ góc servo phản hồi. Đây là cách **kiểm tra chéo** trực quan giữa lệnh (x, y) và dáng cơ cấu thực.

## 4.6 Phân tích không gian làm việc

Không gian làm việc là tập các điểm E(x, y) mà **cả hai nhánh** đều giải được (disc ≥ 0) **và** góc servo nằm trong hành trình [0°, 180°].

Với mỗi cánh tay, điểm E phải nằm trong **vành khăn (annulus)** giới hạn bởi:

- Bán kính trong: `|L2 − L1| = |130 − 70| = 60 mm` (khi khâu gần và xa thẳng hàng, gập lại).
- Bán kính ngoài: `L1 + L2 = 70 + 130 = 200 mm` (khi hai khâu duỗi thẳng).

Không gian làm việc của cơ cấu = **giao của hai vành khăn** tâm A và tâm C:

```
điều kiện trái :  60 ≤ √(x² + y²)         ≤ 200
điều kiện phải :  60 ≤ √((x − L0)² + y²)  ≤ 200
vùng làm việc  =  (điều kiện trái) AND (điều kiện phải), với y > 0
```

Do đó vùng làm việc thực dụng (nửa trên) xấp xỉ **x ∈ [−40, 95] mm, y ∈ [55, 190] mm**. Quanh mỗi động cơ có **"vùng chết"** bán kính 60 mm không thể với tới.

## 4.7 Mô phỏng trên MATLAB

File `Kinematic_and_workspace.m` (kèm `Ik_matlab.txt`) là mô hình tham chiếu để **kiểm chứng động học và vẽ không gian làm việc** trước khi lập trình vi điều khiển:

- Định nghĩa cùng kích thước khâu (l0=5.5, l1=7, l2=13, l3=13, l4=7 cm — tương ứng 55/70/130/130/70 mm).
- Hàm `angle(...)` giải phương trình `A·cosθ+B·sinθ=C` — **giống hệt** phần cài đặt trong `ik_5bar.c`.
- Quét lưới điểm (x, y) và tô các điểm thỏa `condLeft & condRight` để vẽ **đồ thị không gian làm việc** (Hình 4.2).

```
    y
    ▲          ┌───────────────┐
    │       ╱▓▓▓▓▓▓▓▓▓▓▓▓▓╲       ← vùng với tới (giao 2 vành khăn)
    │     ╱▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓╲
    │    │▓▓▓▓ (vùng chết) ▓▓▓│
    │     ╲▓▓▓  R=60   ▓▓▓▓▓╱
    │       ╲___________╱
    └────A───────────C──────────►  x
       (0,0)       (55,0)
```
**Hình 4.2 — Minh họa không gian làm việc (đồ thị chi tiết xuất từ MATLAB)**

> **Quy trình đề xuất:** chạy `Kinematic_and_workspace.m` trong MATLAB/Octave để xuất hình không gian làm việc thật và bảng nghiệm IK mẫu, đưa vào thuyết minh làm minh chứng (xem mục 7.4).

## 4.8 Chọn điểm HOME và vùng cấm

Điểm **HOME** được chọn tại `X = L0/2 = 27,5 mm` (giữa hai động cơ, đối xứng) và `Y = 80 mm`. Lý do:

- Nằm **trong** vùng làm việc, cách xa hai "vùng chết" bán kính 60 mm quanh mỗi động cơ (tại X = 27,5 cần Y ≳ 53 mm).
- Chừa **biên độ (margin)** để chạy demo hình vuông ±10 mm quanh HOME mà không vượt biên.
- Tránh xa **điểm kỳ dị** ở biên vùng làm việc (nơi cơ cấu mất khả năng điều khiển một hướng).

## 4.9 Kiểm chứng chéo

Cùng một thuật toán động học ngược được cài đặt ở **ba nơi**, cần cho **kết quả trùng khớp**:

| Nơi cài đặt | Ngôn ngữ | Vai trò |
|---|---|---|
| `Kinematic_and_workspace.m` | MATLAB | Mô hình gốc, vẽ workspace |
| `src/ik_5bar.c` | C (firmware) | Điều khiển thời gian thực trên ESP32 |
| `hmi/index.html` | JavaScript | Mô phỏng + động học thuận trên HMI |

Việc đối chiếu kết quả tại vài điểm mẫu (ví dụ HOME, các góc của demo) giữa ba cài đặt là bước **xác nhận tính đúng đắn** của động học trước khi vận hành cơ cấu thật. Bản JS còn dùng dạng `atan2` (thay cho `atan`) để tăng ổn định số học khi mẫu số tiến về 0.
