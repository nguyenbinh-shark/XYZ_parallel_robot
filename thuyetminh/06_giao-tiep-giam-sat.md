# CHƯƠNG 6. GIAO TIẾP VÀ GIÁM SÁT

Ngoài lệnh văn bản gõ tay (mục 5.10), firmware hỗ trợ một **giao thức máy có khung + kiểm tra lỗi** để máy tính/GUI/MCU khác điều khiển robot một cách tin cậy, cùng một **giao diện điều khiển trên trình duyệt (Web HMI)**.

## 6.1 Giao thức serial có khung

- **Cổng:** USB-CDC / UART, **115200 baud, 8N1**, kết thúc dòng `\n`.
- **Định tuyến:** hai đường chạy **song song** trên cùng một cổng. Dòng bắt đầu bằng `$` → bộ giải mã giao thức (`proto_handle()`); các dòng khác → lệnh văn bản (`process()`). Log "người đọc" của firmware bắt đầu bằng `[` để host dễ lọc bỏ.

**Khung bản tin** (bốn loại phân biệt bằng ký tự đầu):

```
Host → ESP :  $<seq>,<CMD>,<arg...>*<CC>         (lệnh)
ESP  → Host:  #<seq>,OK|ERR,...*<CC>             (ACK/NAK — echo seq để khớp lệnh)
              @<STATE>,X,Y,Z,GRIP,DIST,...*<CC>  (telemetry định kỳ)
              !<EV>,...*<CC>                     (sự kiện bất đồng bộ)
```

- `<seq>`: số thứ tự, ESP **echo nguyên văn** vào phản hồi để host khớp lệnh ↔ đáp.
- `<CMD>`: tên lệnh (không phân biệt hoa/thường).
- `*<CC>`: **checksum XOR kiểu NMEA** — XOR của mọi byte giữa marker đầu và dấu `*`, in 2 chữ số hex hoa. Chiều Host→ESP có thể **bỏ** `*CC` (tiện gõ tay); nếu có `*`, ESP kiểm tra và trả `ERR,CRC` nếu sai.

```python
def checksum(body: str) -> str:   # body không gồm marker và '*CC'
    x = 0
    for ch in body:
        x ^= ord(ch)
    return f"{x:02X}"
# "1,MOVE,19.0,45.0" → *05 → gửi "$1,MOVE,19.0,45.0*05"
```

> **Quan trọng:** `#...OK` chỉ báo lệnh **được chấp nhận**, không có nghĩa chuyển động đã xong. Theo dõi hoàn tất bằng telemetry (`XYMV`/`ZMV`=0) hoặc sự kiện `!ST,...`.

**Mã lỗi (`ERR,<code>`):** `CRC` (checksum sai), `ARGS` (thiếu/sai tham số), `WORKSPACE` ((x,y) ngoài vùng làm việc), `RANGE` (Z ngoài hành trình), `BUSY` (đang chạy PP/homing), `CMD` (không nhận ra lệnh).

## 6.2 Tập lệnh máy

Lệnh được nhóm theo việc có được phép chạy khi robot đang bận hay không.

**Bảng 6.1 — Bảng lệnh giao thức (rút gọn)**

| Nhóm | Lệnh | Ghi chú |
|---|---|---|
| **Truy vấn/chung** (luôn nhận) | `PING`, `VER`, `STATUS`(=`POS`), `TLM ms`, `EVENTS 0\|1`, `DIST`, `GRIPQ`, `LIMQ`, `PPINFO`, `ABORT` | `ABORT` = dừng khẩn: hủy PP, dừng demo + nội suy XY |
| **Cấu hình PP** (nhận cả khi chạy) | `PICK x,y,z`, `PLACE x,y,z`, `SAFEZ z`, `SETTLE ms`, `GRIPMS ms`, `DROPMS ms`, `RUN` | `RUN` trả `ERR,BUSY` nếu đang chạy |
| **Homing/Teach** (chặn khi bận) | `HOMEZ`, `HOMEZS`, `HOMETOP`, `TEACHPICK`, `TEACHPLACE` | |
| **Thủ công** (chặn khi PP chạy) | `MOVE x,y`, `HOME`, `DEMO`, `Z mm`, `ZR mm`, `ZSTOP`, `ZZERO`, `SUCK`, `DROP`, `PUMP 0\|1` | `MOVE`→`ERR,WORKSPACE`; `Z`→`ERR,RANGE` nếu ngoài hành trình |

## 6.3 Telemetry và sự kiện

### 6.3.1 Khung telemetry

Bật bằng `TLM,<ms>` (0 = tắt). Khung gồm **11 trường**:

```
@<STATE>,<X>,<Y>,<Z>,<GRIP>,<DIST>,<XYMV>,<ZMV>,<LIM>,<S1>,<S2>*<CC>
```

| Trường | Kiểu | Ý nghĩa |
|---|---|---|
| STATE | chuỗi | Trạng thái PP (`IDLE`, `PICK_XY`, …) |
| X, Y | mm | Vị trí đầu công tác (1 số lẻ) |
| Z | mm | Vị trí trục Z (2 số lẻ) |
| GRIP | 0/1 | 1 = đang hút |
| DIST | mm | VL53L0X (giá trị cache, −1 = lỗi) |
| XYMV | 0/1 | 1 = servo XY đang nội suy |
| ZMV | 0/1 | 1 = trục Z đang chạy |
| LIM | 0/1 | 1 = công tắc giới hạn trên đang nhấn |
| S1, S2 | độ | Góc servo (động cơ) 1 & 2 hiện tại (0–180) |

Ví dụ: `@IDLE,19.0,45.0,12.30,0,142,0,0,0,93,87*14`.

> Hai trường **S1, S2** (góc servo thật) cho phép Web HMI vẽ dáng cơ cấu bằng **động học thuận** và hiển thị góc động cơ. Telemetry dùng giá trị **DIST cache** (không đọc trực tiếp VL53L0X) để phép đọc 33 ms không làm trễ vòng PID trục Z.

### 6.3.2 Sự kiện

Bật bằng `EVENTS,1` (hoặc tự bật khi host gửi khung `$` đầu tiên):

- `!ST,<STATE>` — mỗi lần máy trạng thái PP đổi trạng thái.
- `!EVT,NOOBJ` — hút nhưng không phát hiện vật; `!EVT,DROP` — rơi vật; `!EVT,REPICK` — thử gắp lại.

Sự kiện giúp host **phản ứng tức thì** với thay đổi trạng thái mà không cần liên tục hỏi (polling).

## 6.4 Giao diện Web HMI

File `hmi/index.html` là một **giao diện điều khiển chạy thẳng trên trình duyệt**, kết nối ESP32 qua **Web Serial API** (không cần cài đặt, không cần server). Chạy trên **Microsoft Edge / Google Chrome** (Firefox/Safari không hỗ trợ Web Serial).

```
┌─────────────────────────┐   Web Serial (USB-CDC, 115200)   ┌──────────────┐
│   Trình duyệt (HMI)     │ ◄──────────────────────────────► │   ESP32-S3   │
│  • Gửi khung lệnh $     │   $seq,CMD,..*CC  /  #  @  !      │  (firmware)  │
│  • Đọc ACK/telemetry/EV │ ◄─────────────────────────────── │              │
│  • Vẽ trạng thái + sơ đồ│                                  └──────────────┘
└─────────────────────────┘
```
**Hình 6.1 — Kiến trúc và giao diện Web HMI**

Đặc điểm chính:

- Dùng **đúng giao thức khung + checksum** ở mục 6.1 (hàm `send()` tự thêm `seq` và `*CC`).
- Khi kết nối, HMI tự gửi `VER`, `EVENTS,1`, `TLM,200` (stream mỗi 200 ms); có watchdog cảnh báo nếu mở cổng mà không nhận dữ liệu.
- **Bảng trạng thái** đọc từ telemetry: X/Y/Z, VL53L0X, góc động cơ S1/S2, trạng thái PP, và các **đèn báo** (giác hút, giới hạn trên, XY chạy, Z chạy).
- **Sơ đồ cơ cấu (canvas):** vẽ 5 khâu **từ góc servo thật** (S1, S2) → đảo offset (`servoToTheta`) ra góc hình học → **động học thuận** (giao 2 đường tròn `circInt`) ra đầu công tác; có đánh dấu điểm pick (P) và place (D). Nếu thiếu S1/S2 thì suy từ IK của (x, y).
- **Khối điều khiển:** jog XY (phím mũi tên / nút bước 0.5–10 mm / ô nhập + GO), Z lên/xuống (PageUp/Down), giác hút (HÚT/NHẢ), bơm (BẬT/TẮT), homing/teach, cấu hình + RUN/ABORT Pick & Place, và một nút **E-STOP** đỏ lớn (gửi `ABORT`).
- **Console** cuối trang log mọi khung gửi/nhận (màu phân biệt TX/ACK/lỗi/telemetry/sự kiện) để debug.

> **Lưu ý vận hành:** hằng số `SV1_OFF/SV2_OFF` trong HMI **phải bằng** `SERVO*_OFFSET` của firmware, nếu không hình vẽ và "X/Y từ servo" sẽ lệch. Web **không tự giới hạn** vùng làm việc — mọi điểm được gửi xuống, firmware quyết định hợp lệ hay trả `ERR,WORKSPACE`.

## 6.5 Ví dụ host Python

Có thể điều khiển robot từ máy tính bằng thư viện `pyserial`. Đoạn mã dưới minh họa gửi lệnh có checksum và đọc phản hồi/telemetry (trích `PROTOCOL.md`):

```python
import serial, threading

def cc(body):
    x = 0
    for ch in body: x ^= ord(ch)
    return f"{x:02X}"

class Robot:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=1)
        self.seq = 0
        threading.Thread(target=self._reader, daemon=True).start()
    def send(self, cmd, *args):
        self.seq += 1
        body = f"{self.seq},{cmd}" + ("," + ",".join(map(str,args)) if args else "")
        self.ser.write(f"${body}*{cc(body)}\r\n".encode())
    def _reader(self):
        for raw in self.ser:
            line = raw.decode(errors="replace").strip()
            if not line: continue
            m, body = line[0], line[1:]
            if "*" in body:
                body, got = body.rsplit("*", 1)
                if cc(body) != got.upper(): continue
            if   m == "#": print("ACK:", body)
            elif m == "@": print("TLM:", body.split(","))
            elif m == "!": print("EVT:", body)

r = Robot("COM5")
r.send("HOMEZ"); r.send("PICK", 19,45,5); r.send("PLACE", 34,45,5)
r.send("TLM", 200); r.send("RUN")
```

Một phiên làm việc mẫu: `PING → HOMEZ → TLM,200 → PICK → PLACE → RUN → (nhận chuỗi !ST,… tới !ST,DONE) → TLM,0`.
