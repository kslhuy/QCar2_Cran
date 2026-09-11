# Kiểm thử thủ công Electronics Digital Twin

Tài liệu này kiểm thử đúng kiến trúc sau:

```text
Vehicle plant truth
  -> sensor riêng của PCB
  -> sensor_node (any MCU/SoC or simulated role)
  -> SPI/UART có delay, jitter, loss, bit error
  -> compute_node (any MCU/SoC, RTOS or Linux target) chạy C++
  -> UART/CAN-FD tới vehicle
  -> sensor auxiliary hoặc fusion có chủ đích

V2V MsgPack
  -> vehicle bus -> COM C++ -> radio fault model
  -> COM C++ của xe đích -> vehicle bus -> V2V/Trust handlers
```

Sensor mock có sẵn của `VehicleLogic` vẫn là một nguồn riêng. PCB không đọc lại
sensor mock đó; nó sinh measurement từ plant truth. Mặc định `fusion.mode` là
`auxiliary`, vì vậy dữ liệu PCB được quan sát nhưng không âm thầm ghi đè sensor
của xe.

## 1. Build và chạy validation tự động

Từ thư mục gốc `QCar2_Cran`, chạy:

```powershell
.\run_electronics_manual_test.ps1
```

Script sẽ build DLL và static library C++, chạy toàn bộ đường sensor/node/bus,
GNSS dropout, brownout, radio delay/loss, V2V qua hai compute target, kiểm tra
manifest/HIL handshake với một Linux SoC custom, và mở Trust/Observer
Native-authority gate sau 1000 lần so sánh không lỗi ở mỗi stage. Báo cáo nằm ở:

```text
Development/multi_vehicle_self_driving_RealQcar/qcar/simulation/results/electronics_manual_validation.json
```

Kết quả mong đợi là `overall_status: PASS`. Có thể chạy nhanh khi đang phát
triển bằng:

```powershell
.\run_electronics_manual_test.ps1 -AuthorityComparisons 20
```

Mốc 20 chỉ là smoke test; dùng 1000 trước khi bật Native authority trong một
chạy thử chính thức.

## 2. Khởi động hệ thống để thao tác bằng web app

Sau khi validation PASS:

```powershell
.\start_electronics_manual_system.ps1 -VehicleCount 3
```

Script mở các terminal cần thiết cho ba fake vehicle, Python Ground Station
bridge và Vite web app. Mở URL được Vite in ra, sau đó bấm `CONNECT BRIDGE`.

Trong card của mỗi vehicle, mở `Electronics Digital Twin` và kiểm tra:

- `Firmware = native_cpp / sensor_frame_v2`;
- sensor node và compute node chuyển sang `running`;
- `V2V path = firmware`;
- bộ đếm `T / W / C / P checks` tăng dần;
- `Authority gate` thành `READY` khi bốn stage đều đạt ngưỡng;
- chọn `C++ Native authority` rồi bấm `Apply core`;
- `Trust/Observer core` đổi thành `C++ AUTHORITY` và dòng
  `Native T / W / C / P used` tăng ở cả bốn cột.

Nếu bất kỳ parity check nào sai hoặc DLL gặp lỗi runtime, mode tự trở về
`Shadow`, `Safety failback` tăng và Python lại là authority ngay trong chu kỳ
đó. Sau khi điều tra nguyên nhân, bấm `Reset parity`; hệ thống sẽ yêu cầu tích
lũy lại toàn bộ bằng chứng trước khi cho phép bật C++.

## 3. Các bài test fault bằng UI

Các preset luôn xóa fault cũ trước khi áp fault mới, nên có thể lặp lại:

1. `Nominal`: 12 V, sensor bình thường, bus không lỗi.
2. `V2V delay`: delay cố định 80 ms và jitter 20 ms ở radio.
3. `V2V loss`: mất 30% packet radio theo seed của từng twin.
4. `GNSS drop`: GNSS của PCB invalid; sensor xe hiện hữu vẫn độc lập.
5. `Brownout`: input 2 V, cả NAV và COM chuyển sang `brownout`.

Sau mỗi test, bấm `Nominal`. Có thể nhập trực tiếp delay, jitter, drop rate,
bit-error rate, sensor fault và voltage để tạo scenario khác. Với bit corruption,
CRC của link loại frame hỏng nên lớp MsgPack nhìn thấy packet loss thay vì data
không hợp lệ.

## 4. Tiêu chí chấp nhận

- Electronics telemetry xuất hiện cho từng xe và sensor PCB có frame mới.
- V2V firmware mode trao đổi local/fleet state mà không đổi MsgPack schema.
- Delay/loss làm thay đổi counter radio nhưng không crash vehicle process.
- Brownout dừng cả hai MCU; `Nominal` cho phép boot lại.
- Bốn parity failure counter bằng 0 trước khi bật Native authority.
- Sau khi bật, cả bốn native-use counter tăng và failback bằng 0.
- Tắt/đóng tất cả cửa sổ terminal khi hoàn tất phiên test.

Đây là transaction/functional SIL và HIL-contract validation hoàn chỉnh để test
hệ thống hiện tại. Nó
không tuyên bố mô phỏng analog SPICE, RF propagation, nhiệt PCB hay instruction-
accurate cho một chip cụ thể. Bước phần cứng tiếp theo là giữ nguyên C++ core và
thay virtual bus/time bằng BSP/HAL của MCU hoặc SoC đã chọn. Xem
[`HARDWARE_TARGETS.md`](HARDWARE_TARGETS.md).
