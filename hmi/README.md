# HMI Web – Robot 5 Khâu Pick & Place

Giao diện điều khiển kiểu công nghiệp, chạy ngay trong trình duyệt qua **Web Serial API**.
Không cần cài đặt, không cần Python — chỉ là **1 file** [`index.html`](index.html).

## Cách dùng
1. Nạp firmware vào ESP32-S3 và cắm USB (nhận ra 1 cổng COM).
2. Mở [`index.html`](index.html) bằng **Microsoft Edge** hoặc **Google Chrome**
   (Firefox/Safari **không** hỗ trợ Web Serial).
3. Bấm **KẾT NỐI COM** → chọn cổng của ESP32 trong hộp thoại trình duyệt.
4. HMI tự gửi `VER`, `EVENTS,1`, `TLM,200` → bắt đầu nhận trạng thái mỗi 200ms.

## Tính năng
- **Bảng trạng thái:** X/Y/Z, khoảng cách VL53L0X, state Pick&Place, đèn báo
  giác hút / công tắc giới hạn trên / XY đang chạy / Z đang chạy.
- **Mô phỏng cơ cấu 5 khâu trực tiếp** (port động học ngược sang JS) – vẽ tay máy
  theo X/Y thời gian thực, kèm điểm pick (P) và place (D).
- **Jog** XY (nút + phím mũi tên) với bước nhảy chọn được, Z lên/xuống, đi tới tọa độ.
- **Giác hút/bơm**, **Homing/Teach** (HOMEZ, HOMEZ chậm, HOME đỉnh, teach pick/place).
- **Pick & Place:** đặt tọa độ pick/place + tham số → RUN / ABORT.
- **E-STOP** đỏ luôn hiển thị (gửi `ABORT`).
- **Console** xem toàn bộ khung gửi/nhận (`»` gửi, `«` ACK, `@` telemetry, `!` sự kiện).

## Nếu bấm Kết nối không hiện cổng / báo lỗi bảo mật
Một số cấu hình trình duyệt chặn Web Serial khi mở trực tiếp bằng `file://`.
Khi đó chạy 1 web server cục bộ (đã có sẵn Python):

```powershell
cd hmi
python -m http.server 8000
```
Rồi mở `http://localhost:8000/` trong Edge/Chrome.

## Liên quan
- Giao thức serial: [`../PROTOCOL.md`](../PROTOCOL.md)
- Báo cáo kỹ thuật: [`../BAOCAO.md`](../BAOCAO.md)
