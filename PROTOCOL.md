# GIAO THỨC SERIAL – Robot 5 Khâu Pick & Place

Giao thức **ASCII có khung + checksum**, chạy **song song** với lệnh text gõ tay.
Dùng cho host điều khiển máy: GUI Python/C#, MCU khác, ROS…

- **Cổng:** USB CDC / UART, **115200 baud, 8N1**, kết thúc dòng `\n` (LF).
- **Định tuyến:** dòng bắt đầu bằng `$` → giao thức máy; dòng khác → lệnh text cũ (xem `help`).
- Log "người đọc" của firmware bắt đầu bằng `[` (vd `[PP] -> PICK_XY`) — host **lọc theo ký tự đầu dòng** để bỏ qua.

---

## 1. Khung bản tin

### Host → ESP (lệnh)
```
$<seq>,<CMD>,<arg1>,<arg2>,...*<CC>\n
```
| Trường | Ý nghĩa |
|---|---|
| `$` | Ký tự bắt đầu khung |
| `<seq>` | Số thứ tự (chuỗi bất kỳ, thường 0–255). ESP **echo nguyên văn** vào phản hồi để host khớp lệnh ↔ đáp |
| `<CMD>` | Tên lệnh (không phân biệt hoa/thường, ESP tự viết hoa) |
| `<args>` | Tham số, ngăn bằng dấu `,` |
| `*<CC>` | **Tùy chọn.** Checksum XOR (2 hex). Có `*` → ESP kiểm tra; sai → `ERR,CRC`. Không có `*` → vẫn nhận (tiện gõ tay) |

### ESP → Host (phản hồi)
| Mẫu | Loại | Ý nghĩa |
|---|---|---|
| `#<seq>,OK,<CMD>[,<data>]*<CC>` | ACK | Lệnh được nhận / hoàn tất |
| `#<seq>,ERR,<code>[,<msg>]*<CC>` | NAK | Bị từ chối / lỗi |
| `@<telemetry>*<CC>` | Telemetry | Trạng thái định kỳ (khi bật `TLM`) |
| `!<EV>[,<arg>]*<CC>` | Sự kiện | Bất đồng bộ (khi bật `EVENTS`) |

> **Quan trọng:** `#...OK` chỉ báo lệnh **được chấp nhận**, không có nghĩa chuyển động đã xong.
> Theo dõi hoàn tất bằng telemetry (`XYMV`/`ZMV` = 0) hoặc sự kiện `!ST,...`.

---

## 2. Checksum (XOR kiểu NMEA)

`CC` = XOR của **tất cả byte nằm giữa marker đầu (`$`/`#`/`@`/`!`) và dấu `*`**, in **2 chữ số hex hoa**.

```python
def checksum(body: str) -> str:      # body KHÔNG gồm marker, KHÔNG gồm '*CC'
    x = 0
    for ch in body:
        x ^= ord(ch)
    return f"{x:02X}"
```
Ví dụ `body = "1,MOVE,19.0,45.0"` → `*05` → gửi `$1,MOVE,19.0,45.0*05`.

---

## 3. Bảng lệnh (Host → ESP)

### Truy vấn / điều khiển chung — *luôn nhận, kể cả khi đang chạy*
| CMD | Tham số | Phản hồi OK | Ghi chú |
|---|---|---|---|
| `PING` | – | `OK,PING,PONG` | Kiểm tra kết nối |
| `VER` | – | `OK,VER,1.0` | Phiên bản firmware |
| `STATUS` (= `POS`) | – | `OK,STATUS,<telemetry>` | Lấy 1 khung trạng thái tức thì |
| `TLM` | `ms` | `OK,TLM,<ms>` | Bật stream `@` mỗi `ms` (0 = tắt) |
| `EVENTS` | `0\|1` | `OK,EVENTS,<0\|1>` | Bật/tắt sự kiện `!` |
| `DIST` | – | `OK,DIST,<mm>` | Đọc VL53L0X (−1 = lỗi). Khi PP chạy → trả giá trị cache |
| `GRIPQ` | – | `OK,GRIPQ,<0\|1>` | 1 = phát hiện có vật |
| `LIMQ` | – | `OK,LIMQ,<0\|1>` | 1 = công tắc giới hạn trên đang bị nhấn |
| `PPINFO` | – | `OK,PPINFO,<10 trường>` | pickXYZ, placeXYZ, safez, settle, grip, drop |
| `ABORT` | – | `OK,ABORT` | Dừng khẩn: hủy PP, dừng demo + nội suy XY |

### Cấu hình Pick & Place — *nhận cả khi đang chạy (áp dụng lần sau)*
| CMD | Tham số | Ghi chú |
|---|---|---|
| `PICK` | `x,y,z` | Đặt vị trí lấy vật |
| `PLACE` | `x,y,z` | Đặt vị trí đặt vật |
| `SAFEZ` | `z` | Độ cao an toàn khi di chuyển XY |
| `SETTLE` | `ms` | Thời gian chờ servo XY ổn định |
| `GRIPMS` | `ms` | Thời gian giác hút bám |
| `DROPMS` | `ms` | Thời gian nhả vật |
| `RUN` | – | Bắt đầu chuỗi pick-and-place (`ERR,BUSY` nếu đang chạy) |

### Homing / Teach — *bị chặn khi đang bận (`ERR,BUSY`)*
| CMD | Tham số | Ghi chú |
|---|---|---|
| `HOMEZ` | – | Homing tức thì bằng VL53L0X; `OK,HOMEZ,instant` hoặc `,slow` (fallback) |
| `HOMEZS` | – | Homing chậm (robot phải ở vị trí cao nhất) |
| `HOMETOP` | – | Chạy lên đến công tắc giới hạn trên → đặt Z = Z_MAX (50mm) |
| `TEACHPICK` | – | Hạ Z tìm bề mặt → ghi `pick_z` |
| `TEACHPLACE` | – | Hạ Z tìm bề mặt → ghi `place_z` |

### Điều khiển thủ công — *bị chặn khi PP đang chạy (`ERR,BUSY`)*
| CMD | Tham số | OK / lỗi |
|---|---|---|
| `MOVE` | `x,y` | `OK,MOVE` / `ERR,WORKSPACE` |
| `HOME` | – | Về home XY |
| `DEMO` | – | Chạy demo hình vuông |
| `Z` | `mm` | Z tuyệt đối; `ERR,RANGE` nếu ngoài [0,50] |
| `ZR` | `mm` | Z tương đối (±) |
| `ZSTOP` | – | Dừng động cơ Z |
| `ZZERO` | – | Đặt Z hiện tại = 0 |
| `SUCK` | – | Bật bơm + hút |
| `DROP` | – | Nhả + tắt bơm (chặn ~150ms) |
| `PUMP` | `0\|1` | Bật/tắt riêng relay bơm |

---

## 4. Telemetry (`@`) và sự kiện (`!`)

### Khung telemetry
```
@<STATE>,<X>,<Y>,<Z>,<GRIP>,<DIST>,<XYMV>,<ZMV>,<LIM>,<S1>,<S2>*<CC>
```
| Trường | Kiểu | Ý nghĩa |
|---|---|---|
| STATE | chuỗi | Trạng thái PP: `IDLE`,`PICK_XY`,`PICK_LOWER`,… |
| X, Y | mm | Vị trí đầu công tác (1 số lẻ) |
| Z | mm | Vị trí trục Z (2 số lẻ) |
| GRIP | 0/1 | 1 = đang hút |
| DIST | mm | VL53L0X (giá trị cache, −1 = lỗi) |
| XYMV | 0/1 | 1 = servo XY đang nội suy |
| ZMV | 0/1 | 1 = trục Z đang chạy |
| LIM | 0/1 | 1 = công tắc giới hạn trên đang bị nhấn |
| S1, S2 | độ | Góc servo (động cơ) 1 & 2 hiện tại (0–180) |

Ví dụ: `@IDLE,19.0,45.0,12.30,0,142,0,0,0,93,87*14`

### Sự kiện (bật bằng `EVENTS,1` hoặc tự bật khi host gửi khung `$` đầu tiên)
| Khung | Khi nào |
|---|---|
| `!ST,<STATE>` | Mỗi lần state machine PP đổi trạng thái |
| `!EVT,NOOBJ` | Sau khi hút nhưng không phát hiện vật |

---

## 5. Mã lỗi (`#<seq>,ERR,<code>`)
| code | Ý nghĩa |
|---|---|
| `CRC` | Checksum sai |
| `ARGS` | Thiếu/sai tham số (msg gợi ý cú pháp đúng) |
| `WORKSPACE` | Điểm (x,y) ngoài vùng làm việc 5 khâu |
| `RANGE` | Z ngoài hành trình [0, 50] mm |
| `BUSY` | Đang chạy PP/homing (msg = trạng thái hiện tại) |
| `CMD` | Không nhận ra lệnh |

---

## 6. Phiên làm việc mẫu
```
$1,PING*05            ->  #1,OK,PING,PONG*..
$2,HOMEZ              ->  #2,OK,HOMEZ,instant*..
$3,TLM,200            ->  #3,OK,TLM,200*..        (bắt đầu nhận @ mỗi 200ms)
$4,PICK,19,45,5       ->  #4,OK,PICK*..
$5,PLACE,34,45,5      ->  #5,OK,PLACE*..
$6,RUN                ->  #6,OK,RUN*..
                          !ST,PICK_XY*..  !ST,PICK_LOWER*..  ...  !ST,DONE*..
$7,TLM,0              ->  #7,OK,TLM,0*..          (tắt stream)
```

---

## 7. Ví dụ host Python (pyserial)
```python
import serial, threading

def cc(body: str) -> str:
    x = 0
    for ch in body:
        x ^= ord(ch)
    return f"{x:02X}"

class Robot:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=1)
        self.seq = 0
        threading.Thread(target=self._reader, daemon=True).start()

    def send(self, cmd, *args):
        self.seq += 1
        body = f"{self.seq},{cmd}" + ("," + ",".join(map(str, args)) if args else "")
        line = f"${body}*{cc(body)}\r\n"
        self.ser.write(line.encode())
        return self.seq

    def _reader(self):
        for raw in self.ser:
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            m, body = line[0], line[1:]
            if "*" in body:                       # tách + kiểm tra checksum
                body, got = body.rsplit("*", 1)
                if cc(body) != got.upper():
                    print("CHECKSUM SAI:", line); continue
            if   m == "#": print("ACK :", body)    # seq,OK/ERR,...
            elif m == "@": print("TLM :", body.split(","))
            elif m == "!": print("EVT :", body)
            elif m == "[": pass                    # bỏ log người-đọc của firmware

r = Robot("COM5")        # đổi theo cổng thực tế
r.send("HOMEZ")
r.send("PICK", 19, 45, 5)
r.send("PLACE", 34, 45, 5)
r.send("TLM", 200)
r.send("RUN")
```
