# CHƯƠNG 1. TỔNG QUAN

## 1.1 Đặt vấn đề và lý do chọn đề tài

Trong các dây chuyền sản xuất và lắp ráp hiện đại, thao tác **gắp – đặt (Pick & Place)** là một trong những công đoạn phổ biến và lặp lại nhiều nhất: nhặt linh kiện từ băng tải, phân loại sản phẩm, xếp khay, cấp phôi cho máy… Yêu cầu đặt ra là robot phải **nhanh, chính xác, ổn định** và có giá thành hợp lý.

Các robot nối tiếp (serial) truyền thống có mỗi động cơ mang theo cả khối lượng của những khâu phía sau, dẫn đến quán tính lớn, khó đạt tốc độ cao. Ngược lại, **robot song song (parallel robot)** đặt các động cơ cố định trên đế, các khâu chủ yếu chịu lực dọc trục, nhờ đó **quán tính chuyển động thấp, độ cứng vững cao và tốc độ đáp ứng nhanh** — rất phù hợp cho bài toán Pick & Place.

Trong nhóm robot song song, **cơ cấu 5 khâu phẳng (five-bar linkage)** là cấu hình đơn giản, dễ chế tạo, chỉ cần **hai động cơ** đặt cố định để định vị đầu công tác trong mặt phẳng. Đây là mô hình lý tưởng để nghiên cứu và học tập về động học song song, đồng thời đủ khả năng thực hiện một hệ thống gắp – đặt hoàn chỉnh khi bổ sung thêm **trục Z** (nâng/hạ) và **cơ cấu gắp**.

Từ những lý do trên, đề tài **"Thiết kế và chế tạo robot song song 5 khâu thực hiện Pick & Place"** được lựa chọn nhằm kết hợp kiến thức **cơ khí – điện tử – điều khiển – lập trình** trong một sản phẩm cơ điện tử hoàn chỉnh.

## 1.2 Tổng quan về robot song song và cơ cấu 5 khâu

### 1.2.1 Khái niệm robot song song

Robot song song là loại robot mà đầu công tác được nối với đế cố định thông qua **nhiều chuỗi động học kín** hoạt động đồng thời. Cơ cấu 5 khâu gồm: đế cố định, hai **khâu chủ động** (gắn động cơ) và hai **khâu bị động**, khép kín tại **đầu công tác**. Bằng cách điều khiển góc quay của hai khâu chủ động, đầu công tác được định vị đến tọa độ (x, y) mong muốn trong mặt phẳng.

### 1.2.2 So sánh với robot nối tiếp

| Tiêu chí | Robot nối tiếp | Robot song song (5 khâu) |
|---|---|---|
| Vị trí động cơ | Trên các khâu chuyển động | **Cố định trên đế** |
| Quán tính | Lớn (mang theo khâu sau) | **Nhỏ** |
| Tốc độ / gia tốc | Thấp – trung bình | **Cao** |
| Độ cứng vững | Trung bình | **Cao** |
| Không gian làm việc | Rộng | Hẹp hơn |
| Điểm kỳ dị (singularity) | Ít | **Cần tránh** (gần biên) |
| Bài toán động học ngược | Phức tạp | **Đơn giản, dạng đóng** |

### 1.2.3 Ưu – nhược điểm của cơ cấu 5 khâu

- **Ưu điểm:** hai động cơ cố định trên đế → quán tính thấp, đáp ứng nhanh; cấu trúc đối xứng, cứng vững; động học ngược có nghiệm dạng đóng (giải nhanh trên vi điều khiển).
- **Nhược điểm:** không gian làm việc bị giới hạn (giao của hai vùng với hạn của hai cánh tay); tồn tại **điểm kỳ dị** ở biên vùng làm việc; đầu công tác đi theo đường cong khi nội suy trong không gian góc; xuất hiện **nội lực** trong chuỗi kín nếu hai khâu không chuyển động đồng bộ.

## 1.3 Ứng dụng Pick & Place trong công nghiệp

Thao tác Pick & Place bằng robot song song xuất hiện trong nhiều lĩnh vực:

- **Điện tử:** gắp – đặt linh kiện (SMD), IC lên bo mạch.
- **Thực phẩm – dược phẩm:** phân loại, đóng gói, xếp khay bánh kẹo, vỉ thuốc (robot Delta là ví dụ điển hình).
- **Đóng gói – logistics:** nhặt sản phẩm từ băng tải vào hộp, phân loại theo màu/kích thước.
- **Gia công:** cấp/tháo phôi cho máy CNC, máy ép.

Đề tài mô phỏng một trạm Pick & Place thu nhỏ: robot dò cao độ bề mặt bằng cảm biến, hạ đầu hút xuống gắp vật bằng chân không, nâng lên, di chuyển đến vị trí đặt và nhả vật — toàn bộ tự động theo một chu trình có kiểm soát.

## 1.4 Mục tiêu và nhiệm vụ của đề tài

### 1.4.1 Mục tiêu

Thiết kế, chế tạo và lập trình điều khiển một **mô hình robot song song 5 khâu 3 bậc tự do** (XY + Z) có khả năng thực hiện chu trình gắp – đặt vật tự động, ổn định và có thể giám sát/điều khiển từ máy tính.

### 1.4.2 Nhiệm vụ cụ thể

1. Nghiên cứu **động học** cơ cấu 5 khâu (bài toán ngược, thuận, không gian làm việc) và mô phỏng kiểm chứng trên MATLAB.
2. **Thiết kế cơ khí:** cụm định vị XY (five-bar), trục Z vít-me, đầu công tác gắn giác hút và cảm biến.
3. **Thiết kế điện – điện tử:** mạch điều khiển quanh ESP32-S3, driver động cơ, cảm biến, hiển thị, nguồn.
4. **Xây dựng phần mềm điều khiển:** thuật toán động học ngược, nội suy chuyển động servo, điều khiển vòng kín PD cho trục Z, đọc – lọc cảm biến, máy trạng thái Pick & Place.
5. **Truyền thông và giám sát:** giao thức serial có kiểm tra lỗi và giao diện điều khiển Web.
6. **Chế tạo, hiệu chuẩn và thực nghiệm** đánh giá độ chính xác, độ ổn định của hệ thống.

## 1.5 Phạm vi và giới hạn

- Mô hình mang tính **học tập/nghiên cứu**, tải trọng nhẹ (vật gắp bằng giác hút chân không cỡ nhỏ).
- Cụm XY dùng **servo RC** điều khiển **vòng hở** (theo góc lệnh); trục Z điều khiển **vòng kín** nhờ encoder + PD.
- Nội suy chuyển động XY thực hiện trong **không gian góc** (chưa nội suy đường thẳng Descartes).
- Truyền thông qua **cổng USB/serial** (chưa tích hợp không dây).
- Không đi sâu phân tích động lực học (dynamics) và tối ưu quỹ đạo tốc độ cao.

## 1.6 Phương pháp thực hiện và bố cục thuyết minh

**Phương pháp thực hiện:** kết hợp lý thuyết (động học, điều khiển) với mô phỏng (MATLAB) và thực nghiệm (chế tạo, đo đạc, hiệu chỉnh). Phần mềm phát triển trên **PlatformIO/Arduino**, chia module rõ ràng và có các **môi trường test riêng** cho từng cụm chức năng để dễ kiểm tra độc lập.

**Bố cục thuyết minh** gồm 8 chương:

- **Chương 1:** Tổng quan đề tài.
- **Chương 2:** Phân tích và thiết kế cơ khí.
- **Chương 3:** Thiết kế hệ thống điện – điện tử.
- **Chương 4:** Động học và mô phỏng.
- **Chương 5:** Thiết kế hệ thống điều khiển và phần mềm.
- **Chương 6:** Giao tiếp và giám sát.
- **Chương 7:** Chế tạo, thực nghiệm và đánh giá.
- **Chương 8:** Kết luận và hướng phát triển.

Kèm theo là các **phụ lục** về thông số cài đặt, sơ đồ đấu nối, mã nguồn chính, đặc tả giao thức và tài liệu tham khảo.
