# RobustKLNet — training, evaluation và animation trên web

## Bắt đầu nhanh nhất

Tại thư mục **QCar2_Cran**, double-click **[`robust_workflow.cmd`](robust_workflow.cmd)**. Chọn:

| Menu | Việc thực hiện |
|---|---|
| **1** | Kiểm tra Python, các thư viện, checkpoint và Node/Vite |
| **2** | Evaluation nhanh, **không animation** |
| **3** | Evaluation nhanh, sau đó mở **animation trên web** |
| **4** | Evaluation đầy đủ, không animation |
| **5** | Test xe chạy vòng kín với Stanley + PID, không animation |
| **6** | Test vòng kín, sau đó mở animation trên web |
| **7** | Xem lại kết quả evaluation gần nhất, không chạy lại test |
| **8** | Train model mới và validation trong training |

**Muốn xem ngay dữ liệu đã có:** chọn **7**. Nếu chưa chạy workflow lần nào, chương trình dùng bộ kết quả `results/heldout.json` đã có trong dự án.

**Muốn kiểm tra từ đầu:** chọn **1 → 2 → 7**. Menu thực hiện một việc mỗi lần; mở lại file `.cmd` để chọn việc tiếp theo.

Checkpoint đã train sẵn đủ để chạy evaluation. **Không cần train lại trước mỗi lần test.**

## 1. Ba cách chạy khác nhau

```text
Training trên MockQCar
  └─ checkpoint mới + lịch sử train/validation
       └─ Evaluation với seed test riêng
            ├─ Không animation: metrics.json + metrics.npz
            └─ Có animation: cùng kết quả đó → Ground Station web replay

Chạy xe fake live
  └─ fake_vehicle_real_logic.py → Python Ground Station → web điều khiển xe
```

| Chế độ | Mục đích | Cần Python bridge? |
|---|---|---|
| Sensor evaluation | So sánh EKF, RobustKLNet trên **cùng chuỗi cảm biến bị attack** | Không |
| Closed-loop evaluation | Mỗi estimator điều khiển một lượt xe riêng; đo sai số bám đường và tốc độ | Không |
| Web evaluation replay | Xem animation, quỹ đạo, attack và các chỉ số của một evaluation đã hoàn tất | Không |
| Fake vehicle live | Điều khiển xe qua giao diện Ground Station, gửi lệnh và attack trong lúc chạy | Có |

Ở menu **3/6**, evaluation chạy xong rồi web mới mở. Animation là **replay**, không phải streaming trong lúc benchmark đang tính. Số liệu được tính trên toàn bộ mẫu gốc; giảm khung hình hiển thị không làm thay đổi RMSE. Trang replay không gửi lệnh điều khiển xe.

## 2. Setup một lần

### Python

Máy hiện tại đã có môi trường **Qcar**, Node.js và thư viện cần thiết. Thử ngay:

```powershell
.\robust_workflow.cmd check
```

Launcher tự tìm `python.exe` trong môi trường conda `Qcar`; không cần `conda activate` mỗi lần. Nếu dùng môi trường khác:

```powershell
.\robust_workflow.cmd check -CondaEnv MyEnv
# Hoặc chỉ rõ Python:
.\robust_workflow.cmd check -Python 'C:\path\to\env\python.exe'
```

Trên máy mới, mở Anaconda Prompt. Chỉ tạo môi trường nếu chưa có:

```powershell
conda create -n Qcar python=3.11
conda activate Qcar
python -m pip install torch numpy scipy pyyaml omegaconf matplotlib
```

Backend simulation `innovation_trust` chạy CPU; không yêu cầu CUDA. Các gói trên phục vụ training/headless evaluation. Phần xe live dùng thêm các thư viện của dự án và `websockets`.

### Web

Cần Node.js cùng npm. Cài dependency theo lockfile:

```powershell
cd .\GroundStation-Qcar-App
npm ci
cd ..
.\robust_workflow.cmd check
```

`npm ci` chỉ cần khi setup máy mới hoặc khi lockfile thay đổi. Headless evaluation/training không cần khởi động Vite hay Python GUI; lệnh `check` kiểm tra cả môi trường web.

## 3. Evaluation không animation

Mở PowerShell tại **QCar2_Cran**:

```powershell
# Nhanh: 1 seed, 15 giây mô phỏng, 5 kịch bản
.\robust_workflow.cmd quick

# Đầy đủ: 3 seed, mỗi lượt 60 giây mô phỏng, 13 kịch bản
.\robust_workflow.cmd full

# Xe tự chạy vòng kín: 3 seed, mỗi lượt 40 giây, 9 kịch bản
.\robust_workflow.cmd closed-loop
```

Thời lượng trên là thời gian **mô phỏng cho mỗi lượt**, không phải thời gian chờ hoàn tất toàn bộ lệnh. Tổng thời gian thực phụ thuộc máy và số estimator.

- `quick`: `clean`, `gps_jump`, `gps_ramp`, `wheel_freeze`, `gps_wheel`; so sánh EKF, RobustKLNet và analytical ablation.
- `full`: thêm đủ GPS freeze/dropout/noise, heading bias, wheel bias/scale, IMU bias, steering bias; thêm model GRU cũ để đối chiếu. Cần file `models/robust_kalmannet.best_robust.pt` đã có trong dự án.
- `closed-loop`: EKF và RobustKLNet điều khiển xe qua Stanley + PID trên đường tròn bán kính 2 m, tốc độ đặt 0.65 m/s. Hai xe có quỹ đạo thật khác nhau vì estimator ảnh hưởng controller.

Mặc định các lệnh dùng **`models/innovation_trust_sim.npz`** đã kiểm chứng. Terminal in đường dẫn checkpoint, thư mục output và bảng position RMSE.

Mỗi lần chạy ghi vào thư mục mới:

```text
Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/KalmaNet/Robust/
  results/workflow/<timestamp>_<action>/
    metrics.json       # RMSE, lỗi p95/max, cấu hình và hash checkpoint
    metrics.npz        # Toàn bộ quỹ đạo để vẽ/replay
```

Đọc cả **position RMSE (m)**, **heading RMSE (rad)** và **speed RMSE (m/s)**. Với vòng kín, đọc thêm `tracking_rmse_m` và `speed_tracking_rmse_mps`; lỗi bám đường bỏ 3 giây đầu để controller ổn định. Với sensor evaluation, `runtime_p95_ms` đo thời gian update estimator trên máy chạy test. Chỉ số thấp hơn là tốt hơn.

## 4. Evaluation có animation web

```powershell
# Chạy quick evaluation, sau đó tự mở trình duyệt
.\robust_workflow.cmd web

# Hoặc chạy xe vòng kín rồi mở animation
.\robust_workflow.cmd closed-loop-web

# Xem evaluation gần nhất, không tính lại
.\robust_workflow.cmd replay
```

Trang mở tại **http://127.0.0.1:3000/?evaluation=1**. Bạn cũng có thể bấm **Evaluation** trên thanh trên cùng của Ground Station.

1. Chọn **Scenario** và **Seed**.
2. Bấm **Play**, đổi tốc độ từ 0.25× đến 8×.
3. Kéo thanh thời gian để xem lúc attack bắt đầu và khi cảm biến hồi phục. Vùng đỏ đánh dấu khoảng attack.
4. Bật/tắt từng đường: ground truth, EKF, RobustKLNet, ablation hoặc GRU cũ nếu có.
5. Xem bảng RMSE và tốc độ tại con trỏ thời gian. Mũi tên trên đường biểu diễn vị trí và heading.

Ở sensor replay, các estimator dùng chung ground truth. Ở closed-loop, mỗi estimator có **đường truth riêng**; đường tròn nét chấm là đường cần bám.

Giữ terminal mở để web hoạt động. **Ctrl+C** dừng server; kết quả test vẫn được lưu. Nếu cổng 3000 đang dùng:

```powershell
.\robust_workflow.cmd replay -Port 3001
```

Đọc một bộ kết quả cụ thể (file `.npz` cùng tên phải nằm cạnh `.json`):

```powershell
$robust = '.\Development\multi_vehicle_self_driving_RealQcar\qcar\Observer\KalmaNet\Robust'
.\robust_workflow.cmd replay -Results "$robust\results\heldout.json"
# Bộ test xe chạy vòng kín đã lưu:
.\robust_workflow.cmd replay -Results "$robust\results\closed_loop.json"
```

Replay exporter tạo `GroundStation-Qcar-App/public/evaluation/latest.json`. Nút **Open replay JSON** đọc **file đã export này**, không đọc trực tiếp `metrics.json` hay `.npz`. Có thể giữ bản sao file export để chia sẻ hoặc mở lại. File sinh tự động không được đưa vào Git.

Nếu đã có Vite chạy, export dữ liệu rồi mở URL `/?evaluation=1` trên server đó:

```powershell
conda activate Qcar
python "$robust\export_evaluation_replay.py" "$robust\results\heldout.json" --output '.\GroundStation-Qcar-App\public\evaluation\latest.json'
```

## 5. Training → validation → test model mới

```powershell
# Train model mới, giữ nguyên model đã bàn giao
.\robust_workflow.cmd train

# Đánh giá đúng model vừa train
.\robust_workflow.cmd quick -Checkpoint latest
.\robust_workflow.cmd full -Checkpoint latest
.\robust_workflow.cmd closed-loop -Checkpoint latest

# Xem bộ test vừa chạy
.\robust_workflow.cmd replay
```

`train` dùng seed train **1–6**, validation **21–22**, 30 giây/drive, tối đa **60 epoch mỗi round**, **2 round**; có early stopping theo validation loss. Sensor test dùng **101–103**; closed-loop dùng **401–403**. Giữ các nhóm seed riêng để đánh giá khả năng tổng quát hóa. Khi đã dùng test để chỉnh model, nên đánh giá cuối bằng seed mới chưa dùng để lựa chọn model.

Output training:

```text
results/workflow/<timestamp>_training/
  innovation_trust.npz             # Checkpoint mới
  innovation_trust.history.json    # Train loss, val loss, seed, cấu hình
  training_cache_round*.npz        # Dữ liệu huấn luyện đã sinh
```

**`-Checkpoint latest`** chọn lần train đầy đủ gần nhất. Nếu chưa có, dùng checkpoint đã bàn giao. Không có tham số này thì evaluation luôn dùng model đã bàn giao. `replay` luôn xem checkpoint ghi trong kết quả đã lưu; không chạy lại estimator bằng model hiện tại.

Cũng có thể chỉ định đường dẫn:

```powershell
.\robust_workflow.cmd web -Checkpoint 'C:\path\to\innovation_trust.npz'
```

Train với cấu hình riêng:

```powershell
conda activate Qcar
$robust = '.\Development\multi_vehicle_self_driving_RealQcar\qcar\Observer\KalmaNet\Robust'
python "$robust\train_robust_kalmannet.py" --simulation-trust `
  --train-seeds 1 2 3 4 5 6 --val-seeds 21 22 `
  --duration 30 --epochs 100 --rounds 2 `
  --output "$robust\results\my_training\model.npz" `
  --cache "$robust\results\my_training\cache.npz"
.\robust_workflow.cmd quick -Checkpoint "$robust\results\my_training\model.npz"
```

Training trực tiếp bằng Python không cập nhật con trỏ `latest` của launcher; hãy truyền đúng đường dẫn output. Simulation backend mới dùng **`.npz`**. File YAML training cũ và checkpoint **`.pt`** thuộc pipeline GRU trước đây. Dùng `--simulation-trust` và `--simulation` để chọn pipeline simulation mới.

## 6. Xe fake live trong Ground Station

Phần này dùng **VehicleLogic thật với phần cứng MockQCar**, cho phép gửi lệnh qua web. Đây là một lượt vận hành tương tác; không tự tạo bảng benchmark giống menu 2–6.

Mở ba terminal từ **QCar2_Cran**. Kích hoạt `Qcar` trong hai terminal Python.

**Terminal A — Python Ground Station:**

```powershell
conda activate Qcar
cd .\Development\multi_vehicle_self_driving_RealQcar\qcar\GUI
python app_main.py --cars 1 --ip 127.0.0.1 --port 5000 --ws-port 8080
```

**Terminal B — một xe mô phỏng:**

```powershell
conda activate Qcar
cd .\Development\multi_vehicle_self_driving_RealQcar\qcar\simulation
python fake_vehicle_real_logic.py 0 127.0.0.1 5000 kinematic qcar `
  --longitudinal-model=qlabs_velocity --steering-model=default `
  --local-estimator=robust_kalman_net --use-direct-poses
```

**Terminal C — web:**

```powershell
cd .\GroundStation-Qcar-App
npm run dev -- --host 127.0.0.1 --port 3000 --strictPort
```

### 6.1 Kết nối trước khi gửi lệnh

Mở **http://127.0.0.1:3000/**. Trang live cần cả ba terminal ở trên hoạt động.

1. Kiểm tra **Bridge: CONNECTED** trên thanh trên cùng. Web mặc định kết nối `ws://127.0.0.1:8080` khi mở bằng địa chỉ trên. Nếu cần, bấm biểu tượng **phích cắm** có tooltip **Connect Bridge**.
2. Chờ **QCar 0** kết nối và khởi tạo xong. Bridge chỉ xác nhận kết nối web ↔ Terminal A; Terminal B còn phải kết nối TCP tới `127.0.0.1:5000`.
3. Chọn **QCar 0** bên trái. Đợi trạng thái xe ra khỏi `INITIALIZING` và dữ liệu cập nhật.
4. Trong **Observers**, đọc dòng **Active**. Đây là estimator do xe xác nhận; chọn trong dropdown chưa gửi lệnh cho đến khi bấm **Apply**.

Nếu ảnh hiển thị **Bridge: DISCONNECTED**, `Active: 0 / 1` và biểu đồ đứng yên: chưa thể thử attack. Kiểm tra lỗi ở Terminal A/B, sau đó kết nối lại. Dữ liệu cũ có thể vẫn nằm trên đồ thị; nó không chứng minh xe đang chạy.

### 6.2 Lượt thử đầu tiên bằng các nút trên web

Ví dụ dễ quan sát: **EKF → GPS jump → Stop Attack → hồi phục**.

1. Chọn **Local Obs = ekf**, bấm **Apply**, chờ **Active: ekf**. Cả `ekf` và `robust_kalman_net` đều hỗ trợ **Local Sensor Attack**.
2. Dùng path đã khởi tạo từ `fleet_config.yaml` cho lượt đầu. Nếu cần đổi, nhập chuỗi node hợp lệ của bản đồ trong **Path & Position** rồi bấm **Set Path**. Chuỗi trong ô nhập không tự áp dụng.
3. Đặt **Target Speed**, ví dụ **0.5 m/s**, và bấm nút xanh **Start** của xe. Chờ xe bám đường, quan sát khoảng **5–10 giây không attack**.
4. Mở tab **Local**. Trong **Local Sensor Attack**, chọn **Target = GPS**, **Type = jump**, **Burst = 500 to 500**, **Seed = 42**.
5. Bấm **Start Attack**. Chờ **STARTING → INJECTING**; ô **GPS** phải hiện `jump`. Quan sát quỹ đạo ước lượng, vận tốc và lệnh steering/throttle.
6. Sau khoảng **5 giây**, bấm **Stop Attack**. Chờ **IDLE**, rồi quan sát thêm **5–10 giây** để xem estimator/controller hồi phục.
7. Bấm **Stop** của xe khi kết thúc lượt. **Stop Attack** chỉ dừng làm sai cảm biến; **Stop** điều khiển trạng thái xe. Muốn dừng cả hai, bấm cả hai nút.

Freeze có thể ít biểu hiện khi xe đứng yên hoặc tín hiệu đang gần hằng số. Thử lúc xe đang chạy, tăng/giảm tốc hoặc vào cua tùy cảm biến.

### 6.3 Đổi EKF và Robust KalmanNet trên web

**Không cần khởi động lại Terminal B chỉ để đổi estimator.** Bấm **Stop Attack**, chọn estimator trong **Local Obs**, rồi **Apply**. Có thể bấm **Stop** xe trước và **Start** lại sau khi dòng **Active** đã đổi.

- `ekf`: EKF dùng cấu hình động học simulation của xe fake.
- `robust_kalman_net`: Robust KalmanNet; launcher này dùng backend `innovation_trust` và checkpoint simulation. Đây là mục cần chọn, không có mục riêng tên “robust EKF”.
- **Fleet Obs** phục vụ fleet/V2V; đổi mục này không thay estimator local đang thử.

**Apply tạo filter mới tại pose ước lượng hiện tại**, giữ cấu hình simulation và checkpoint live. Covariance và bộ nhớ filter được khởi tạo lại; không phải hai filter chạy đồng thời, và cũng không đưa xe về điểm xuất phát. Chờ vài giây baseline rồi mới attack. Attack cũ không được chuyển sang filter mới.

Để hai lượt có cùng điểm xuất phát, dừng Terminal B bằng **Ctrl+C**, chạy lại với `--local-estimator=ekf` hoặc `--local-estimator=robust_kalman_net`. Giữ cùng path, speed, controller, electronics, target/type, burst và seed; Start Attack ở vị trí/thời điểm tương tự. Thao tác tay và nhiễu live vẫn có thể tạo hai chuỗi dữ liệu khác nhau. Sensor benchmark phù hợp khi cần so sánh định lượng trên chính xác cùng dữ liệu.

### 6.4 Target và Type tác động lên tín hiệu nào

Attack được chèn vào **đầu vào estimator** trước khi update. Nó không đổi ground truth MockQCar trực tiếp. Estimate sai có thể khiến controller đổi lệnh, từ đó thay đổi quỹ đạo thật của xe.

| Target | Đầu vào bị tác động | Type có trên web |
|---|---|---|
| **GPS** | Vị trí `x, y` và trạng thái phép đo GPS | `noise`, `freeze`, `jump`, `dropout`, `reacquisition` |
| **Velocity** | Vận tốc đo từ motor tach/wheel | `bias`, `scale`, `freeze`, `noise`, `ramp`, `zero_out` |
| **IMU** | Gia tốc `ax, ay` và gyro `wz` | Cùng sáu type như Velocity |
| **Steering** | Steering đưa vào mô hình dự đoán của estimator | Cùng sáu type như Velocity |
| **Random Branch** | Một nhánh IMU, steering hoặc velocity ngẫu nhiên mỗi burst | Cùng sáu type như Velocity; không chọn GPS |

| Type | Hành vi |
|---|---|
| `bias` | Cộng độ lệch được lấy mẫu cho mỗi burst |
| `scale` | Nhân tín hiệu với hệ số sai |
| `freeze` | Giữ giá trị ở đầu burst; burst mới có thể lấy lại giá trị giữ mới |
| `noise` | Thêm nhiễu vào tín hiệu |
| `ramp` | Độ lệch tăng theo số update trong burst |
| `zero_out` | Đưa giá trị về 0; khác với báo mất dữ liệu |
| GPS `jump` | Dịch vị trí bằng một vector offset trong burst |
| GPS `dropout` | Báo phép đo vị trí không hợp lệ |
| GPS `reacquisition` | Mất GPS ở đầu burst, sau đó có lại với vị trí lệch |

GPS attack trên web tập trung vào `x, y`; chưa có nút riêng cho GPS heading bias, GPS ramp hoặc phối hợp GPS + wheel. Các trường hợp benchmark đó không ánh xạ một-một sang panel live. Web chọn một target/type mỗi lần; **Stop Attack** rồi đổi lựa chọn để thử trường hợp kế tiếp.

### 6.5 Burst, seed và trạng thái attack

**Start Attack bật chế độ lặp cho đến khi bấm Stop Attack.** Burst không phải tổng thời lượng lượt thử. Hết burst, generator có thể bắt đầu burst mới ngay; **Remaining** có thể tăng trở lại.

- **1 step = 1 update local estimator**, cho cả EKF và Robust KalmanNet.
- Fake launcher đặt nhịp observer mục tiêu **100 Hz**: 20 steps khoảng **0.2 s**, 200 steps khoảng **2 s**, 500 steps khoảng **5 s** nếu đạt nhịp này. Khi máy chạy chậm, thời gian thực dài hơn. Chỉ số 50 Hz trên biểu đồ không phải đồng hồ đếm attack.
- Min = max cho độ dài burst cố định; hai giá trị khác nhau cho phép lấy mẫu độ dài trong khoảng đó.
- **Seed** cố định giúp lặp lại việc lấy mẫu tham số/nhiễu. Cùng seed chưa đảm bảo hai lượt live có cùng đầu vào, thời điểm attack hay quỹ đạo.
- Độ mạnh dùng cấu hình mặc định của generator. **Intensity** là chỉ báo chênh lệch tín hiệu đang inject, không phải thanh chỉnh độ mạnh, RMSE hay xác suất phát hiện attack.

| Trạng thái | Ý nghĩa |
|---|---|
| **IDLE** | Inject đã tắt |
| **STARTING / STOPPING** | Đã gửi lệnh, đang chờ telemetry xác nhận |
| **ARMED** | Generator đã bật, chưa có metadata burst đang inject |
| **INJECTING** | Có burst attack; đọc Branch/GPS để biết loại thực tế |

Nút bị khóa khi bridge/xe chưa kết nối, xe chưa báo hỗ trợ, hoặc đang chờ xác nhận. Sau khoảng 8 giây không có xác nhận, panel báo kiểm tra kết nối/log. Đọc Terminal B nếu **Active** không đổi hoặc lệnh attack bị từ chối. Sau khi sửa code Python, khởi động lại Terminal B để dùng phiên bản mới.

### 6.6 Đọc realtime performance

Tab **Local** hiển thị **một estimator local đang hoạt động**:

| Biểu đồ | Điều quan sát được |
|---|---|
| **Trajectory X-Y** | Quỹ đạo ước lượng; chấm xanh là estimate mới nhất |
| **Velocity** | Vận tốc trong telemetry |
| **Throttle / Steering** | Phản ứng lệnh điều khiển khi estimate thay đổi |
| **Acceleration** | Gia tốc được truyền trong dữ liệu local |

**Chấm đỏ giữa đồ thị XY là gốc tọa độ, không phải ground truth hoặc EKF thứ hai.** Quan sát bước nhảy, dao động, trôi và thời gian trở lại ổn định trước/trong/sau attack. Đường estimate mượt không tự chứng minh estimate đúng. Tùy target và chuyển động, attack có thể gây ít thay đổi nhìn thấy.

Panel Local hiện **chưa có live ground-truth overlay, live RMSE hoặc hai đường EKF/Robust chạy song song**. Bật attack cho cả hai không bổ sung các biểu đồ đó. Robust KalmanNet không được đảm bảo tốt hơn EKF ở mọi attack hoặc mọi cấu hình.

Để đánh giá định lượng, dùng menu **3** hoặc **6**, vào **Evaluation** để xem truth, estimate từng phương pháp và RMSE. Menu 3 dùng cùng chuỗi cảm biến; menu 6 đánh giá các lượt closed-loop riêng. Đây là replay kết quả đã tính, không nhận nút attack live.

### 6.7 Thử các trường hợp

Với mỗi estimator, chạy **baseline → attack → recovery** cho từng hàng. Giữ path/speed như nhau và ghi lại seed/thiết lập. Đây là gợi ý thao tác, không phải kết quả thử đã được khẳng định.

| Lượt | Target / Type | Quan sát chính |
|---|---|---|
| 1 | Không attack | Hành vi baseline |
| 2 | GPS / jump | Estimate nhảy và controller phản ứng |
| 3 | GPS / freeze | Estimate bị kéo về vị trí cũ hay tiếp tục chuyển động |
| 4 | GPS / dropout | Hành vi khi không có correction GPS |
| 5 | GPS / reacquisition | Phản ứng khi GPS trở lại với offset |
| 6 | GPS / noise | Dao động của estimate và steering |
| 7 | Velocity / bias, rồi freeze | Vận tốc/throttle, nhất là khi đổi tốc độ |
| 8 | IMU / bias, rồi freeze | Heading/quỹ đạo khi vào cua |
| 9 | Steering / bias, rồi scale | Dự đoán hướng và bám đường |

Các type còn lại trong bảng 6.4 dùng cùng quy trình. Mỗi lần đổi type, **Stop Attack**, chờ hồi phục rồi bắt đầu lượt mới. Restart xe khi cần baseline mới. **V2V Attacks** và **Electronics Twin faults** là các cơ chế khác; giữ chúng tắt khi muốn tách riêng tác động của local sensor attack.

### 6.8 Checkpoint và electronics

Checkpoint live được chọn trong [`simulation/robust_estimator_config.py`](Development/multi_vehicle_self_driving_RealQcar/qcar/simulation/robust_estimator_config.py), hiện là `models/innovation_trust_sim.npz`. Launcher training/evaluation không tự đổi checkpoint live. Chỉ đổi đường dẫn `checkpoint` sau khi xem đánh giá model mới. Khởi tạo bằng CLI và đổi bằng **Apply** đều dùng cấu hình simulation này cho `robust_kalman_net`.

Xe fake còn dùng [`simulation/parameters.yaml`](Development/multi_vehicle_self_driving_RealQcar/qcar/simulation/parameters.yaml), gồm electronics twin. Nếu thiếu native electronics DLL, hoàn tất setup electronics hoặc đặt `electronics.enabled: false` khi chỉ cần kiểm tra estimator. Benchmark headless tự tắt electronics twin. Kết quả benchmark không khẳng định cùng độ chính xác cho mọi cấu hình electronics/controller live.

## 7. Kiểm tra nhanh và xử lý lỗi

```powershell
# Chỉ kiểm tra đường chạy, không dùng kết quả smoke để kết luận chất lượng
.\robust_workflow.cmd quick -Smoke
.\robust_workflow.cmd train -Smoke
.\robust_workflow.cmd closed-loop -Smoke
```

Smoke training không cập nhật `-Checkpoint latest`. Smoke evaluation vẫn là kết quả gần nhất để có thể replay ngay. Muốn quay về bộ đầy đủ, dùng `replay -Results ...\heldout.json` hoặc chạy lại `full`.

| Lỗi | Cách xử lý |
|---|---|
| Thiếu `torch`, `yaml`, `omegaconf` | Kiểm tra Python in ở đầu terminal; cài thư viện trong đúng env hoặc truyền `-Python` |
| Không tìm thấy Qcar | Dùng `-CondaEnv` hoặc `-Python`; xem phần setup |
| `npm`/Node không tồn tại | Cài Node.js rồi mở terminal mới |
| Thiếu Vite | Chạy `npm ci` trong `GroundStation-Qcar-App` |
| Cổng 3000 bận | Dùng `-Port 3001` hoặc dừng server cũ |
| Trang evaluation chưa có dữ liệu | Chạy `robust_workflow.cmd replay`; giữ cả `.json` và `.npz` của benchmark |
| Bridge disconnected trong live mode | Kiểm tra Terminal A và WebSocket cổng 8080; replay không cần bridge |
| Muốn headless trên máy không có Node | Chạy `quick/full/train/closed-loop`; các lệnh này không yêu cầu Node |
| PowerShell chặn script | Dùng file `.cmd`; launcher chỉ dùng ExecutionPolicy Bypass cho tiến trình đó |

## 8. Các file chính

- [Menu launcher](robust_workflow.cmd), [PowerShell launcher](robust_workflow.ps1)
- [Workflow Python](Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/KalmaNet/Robust/workflow.py)
- [Training simulation](Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/KalmaNet/Robust/train_innovation_trust.py)
- [Sensor benchmark](Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/KalmaNet/Robust/simulation_benchmark.py)
- [Closed-loop benchmark](Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/KalmaNet/Robust/closed_loop_benchmark.py)
- [Replay exporter](Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/KalmaNet/Robust/export_evaluation_replay.py)
- [Web evaluation component](GroundStation-Qcar-App/components/EvaluationReplay.tsx)
- [Báo cáo kết quả đã kiểm chứng](Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/KalmaNet/Robust/Rapport_paper/robust_kalmannet_trust_report.pdf)
