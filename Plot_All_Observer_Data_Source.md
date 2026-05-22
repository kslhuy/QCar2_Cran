# GUI "Plot All Observer" 实时绘图数据来源分析

## 📊 数据流路径概览

```
车辆端 (Vehicle Process)
    ↓
vehicle_logic._build_telemetry_data()
    ↓
_collect_distributed_debug_telemetry() ← 分布式观察器 debug 信号
    ↓
client_Ground_Station.queue_telemetry()
    ↓
WebSocket/TCP 传输 (20Hz)
    ↓
远程 GUI (Ground Station)
    ↓
remote_controller.py (消息处理)
    ↓
remote_scope_manager.receive_observer_telemetry()
    ↓
ObserverDataBuffer (环形缓冲区)
    ↓
RemoteObserverViewer (matplotlib 绘图进程)
    ↓
实时图表显示
```

---

## 🚗 车辆端数据来源

### 1. **主函数：_build_telemetry_data()**
**位置：** [vehicle_logic.py](../../qcar/vehicle_logic.py#L546)

核心作用：
- 每 10Hz 收集当前车辆状态
- 构建遥测数据包
- 调用 `_collect_distributed_debug_telemetry()` 附加分布式观察器信号

```python
def _build_telemetry_data(self) -> dict:
    """Build telemetry data dictionary"""
    # 1. 获取当前状态（本地观察器）
    state_info = self.vehicle_observer.get_estimated_state_for_control()
    
    # 2. 基础遥测数据
    telemetry = {
        'timestamp': time.time(),        # UNIX 时间戳
        'x': float(state_info['x']),     # 本地估计 X
        'y': float(state_info['y']),     # 本地估计 Y
        'th': float(state_info['theta']), # 本地估计 θ
        'v': float(state_info['velocity']), # 本地估计速度
        'u': float(self._last_u),        # 油门控制输出
        'delta': float(self._last_steering), # 转向控制输出
        'state': self.state_machine.state.name, # 当前状态机状态
        'gps_valid': self.vehicle_observer.is_gps_valid()
    }
    
    # 3. 添加分布式观察器 debug 信号 ← **关键数据源**
    telemetry.update(self._collect_distributed_debug_telemetry())
    
    return telemetry
```

**发送频率：** 10Hz (100ms)

---

### 2. **关键函数：_collect_distributed_debug_telemetry()**
**位置：** [vehicle_logic.py](../../qcar/vehicle_logic.py#L591)

核心作用：
- 从分布式 Luenberger 观察器提取 debug 数据
- 扁平化多维数组以适应 JSON 格式
- 包含所有舰队估计信号

**提取的数据包括：**

```python
def _collect_distributed_debug_telemetry(self) -> dict:
    """Collect distributed observer debug signals"""
    
    fleet_estimator = self.vehicle_observer.fleet_estimator
    debug_data = fleet_estimator.get_debug_data()  # 获取内部状态
    
    # 扁平化信号: 将多维数组转为可序列化的值
    flattened = {}
    
    # 1. 观察器状态向量 (x_vec_after_update / x_vec_before_update)
    # 形状: (observer_size * state_dim, num_cars)
    # 例如：x_vec_after_0, x_vec_after_1, ... (动力学模型状态)
    
    # 2. 动力学模型输出
    # dynamics_pred_0, dynamics_pred_1, ... (预测位置)
    
    # 3. 测量信号
    # measurement_x, measurement_y, ... (本地传感器测量)
    
    # 4. 共识信号
    # consensus_state_0, consensus_state_1, ... (其他车辆状态)
    
    # 5. 真实位置 (仅在仿真中)
    # true_position_x, true_position_y, ... (真实状态)
    
    return flattened
```

---

## 📡 网络传输

### 传输流程
```
1. 车辆端: vehicle_logic._send_telemetry_to_ground_station()
   ↓
2. 队列: self.client_Ground_Station.queue_telemetry(telemetry)
   ↓
3. 通讯: ground_station_client._send_queued_telemetry()
   ↓
4. 网络: WebSocket 或 TCP 发送 JSON 数据
   ↓
5. 接收: remote_controller.py (GUI 后端)
```

### 网络参数
- **协议：** WebSocket + JSON
- **发送频率：** 20Hz (client 端速率限制)
- **消息格式：** JSON object
- **消息类型：** `'telemetry'` 或 `'observer_telemetry'`

---

## 🖥️ GUI 端数据接收与缓冲

### 1. **消息处理：remote_controller.py**
**位置：** [remote_controller.py](../../qcar_gui/controllers/remote_controller.py#L530)

```python
# WebSocket 消息处理
if message_type == 'observer_telemetry':
    car_id = data.get('vehicle_id')
    telemetry = data.get('data', {})
    
    # 传递给 scope manager
    self.scope_manager.receive_observer_telemetry(car_id, telemetry)
```

---

### 2. **缓冲管理：remote_scope_manager.py**
**位置：** [remote_scope_manager.py](../../qcar_gui/controllers/remote_scope_manager.py#L154)

#### **ObserverDataBuffer 类**
```python
class ObserverDataBuffer:
    """线程安全的分布式观察器遥测缓冲区"""
    
    def __init__(self, max_points: int = 4000):
        # 环形缓冲区，最多存储 4000 个时间点
        self._times: Dict[int, deque] = {}      # 时间戳列表
        self._fields: Dict[int, Dict[str, deque]] = {} # 字段值列表
    
    def append(self, car_id: int, telemetry: Dict[str, Any]):
        """添加遥测数据到缓冲区"""
        # 1. 过滤数值字段 (忽略文本/布尔值)
        values = {}
        for key, value in telemetry.items():
            if key.startswith(('x_vec_', 'dynamics_', 'measurement_', 'consensus_', 'true_')):
                values[key] = float(value)
        
        # 2. 存储到时间序列
        self._times[car_id].append(time.time())
        self._fields[car_id][field_name].append(value)
    
    def snapshot(self) -> Dict[int, Dict[str, np.ndarray]]:
        """获取当前缓冲区快照（用于绘图）"""
        # 返回所有车辆的时间戳和字段数据
        # 格式：{car_id: {"time": [t1, t2, ...], "fields": {field: [v1, v2, ...]}}}
```

**缓冲特性：**
- 独立为每辆车维护时间序列
- 自动丢弃超过 4000 个点的旧数据
- 线程安全（带锁）
- 每条遥测包含所有信号字段

---

## 📈 绘图过程

### 1. **启动流程**
按下 GUI 中的 "Plot All Observer" 按钮：

```python
# app.py
def _launch_plot_all_observer_viewer(self):
    """启动 Plot All observer 查看器"""
    if not self._remote.is_plot_all_observer_viewer_running():
        # 启动参数
        started = self._remote.open_plot_all_observer_viewer(
            refresh_ms=150,     # 150ms 刷新周期（~6.7Hz 图表更新）
            time_window=0.0     # 显示所有数据
        )
```

---

### 2. **RemoteObserverViewer 类**
**位置：** [remote_scope_manager.py](../../qcar_gui/controllers/remote_scope_manager.py#L919)

```python
class RemoteObserverViewer:
    """在单独进程中运行的 Plot-All 观察器查看器"""
    
    def start(self):
        """启动绘图进程"""
        # 创建数据队列用于 IPC
        self._data_queue = mp.Queue(maxsize=4)  # 最多队列 4 个帧
        
        # 启动绘图进程
        self._process = mp.Process(
            target=_run_observer_plot_process,
            args=(self._data_queue, self._stop_event, ...)
        )
        self._process.start()
        
        # 启动数据喂养线程
        self._feeder_thread = threading.Thread(
            target=self._feed_data_to_process
        )
        self._feeder_thread.start()
    
    def _feed_data_to_process(self):
        """从缓冲区读取数据并发送到绘图进程"""
        while self.running:
            # 每 150ms 获取一次快照
            snap = self.buffer.snapshot()
            
            # 修剪到指定时间窗口
            trimmed = _observer_trim_snapshot(snap, time_window)
            
            # 通过队列发送到绘图进程
            self._data_queue.put_nowait(trimmed)
```

**关键参数：**
- 刷新周期：150ms（约 6.7 Hz 图表更新）
- 缓冲区容量：4 个数据帧
- 时间窗口：0.0（显示所有数据，不修剪）

---

### 3. **绘图渲染：_run_observer_plot_process()**
**位置：** [remote_scope_manager.py](../../qcar_gui/controllers/remote_scope_manager.py#L1084)

**matplotlib 动画更新周期：**
```python
ani = animation.FuncAnimation(
    fig, update_func,
    interval=int(1000 // fps),  # fps = refresh_ms 计算的帧率
    blit=False,
    cache_frame_data=False
)

# 例如：refresh_ms=150 → fps ≈ 6.7 Hz → interval ≈ 150ms
```

**绘图内容：**

#### **图表布局（3×2）：**

```
┌─────────────────────────────────────────┐
│     舰队轨迹 (车0,1,2,3 XY位置)  │ 速度 │
│                                  │      │
│                                  │ (v0) │
│                                  │ (v1) │
│                                  │ (v2) │
│                                  │ (v3) │
├──────────────────────────────────┼──────┤
│                                  │ 共识 │
│                                  │ 误差 │
│                                  │      │
├──────────────────────────────────┼──────┤
│       信任分数                    │ 信息 │
│    (T0, T1, T2, T3 vs time)     │ 面板 │
└──────────────────────────────────┴──────┘
```

#### **信号映射：**

| 图表 | 字段 | 含义 |
|------|------|------|
| **舰队轨迹** | `fleet_x_i`, `fleet_y_i` | 第 i 辆车的估计位置 |
| **速度** | `fleet_v_i` | 第 i 辆车的估计速度 |
| **共识误差** | `consensus_error` | 舰队状态共识偏差 |
| **信任分数** | `trust_i` | 对第 i 辆车的信任权重 |

---

## 🔄 数据采样到显示的完整链条

```
┌─────────────────────────────────────────┐
│ Vehicle Process (车辆)                  │
│ - vehicle_logic._build_telemetry_data() │
│ - 频率: 10Hz                            │
│ - 数据: x,y,θ,v,x_vec_*,dynamics_*...  │
└─────────┬───────────────────────────────┘
          │ WebSocket JSON
          ↓
┌─────────────────────────────────────────┐
│ Ground Station (远程 GUI)                │
│ - remote_controller.py                  │
│ - 接收消息处理                          │
└─────────┬───────────────────────────────┘
          │ receive_observer_telemetry()
          ↓
┌─────────────────────────────────────────┐
│ RemoteScopeManager                      │
│ - ObserverDataBuffer (环形缓冲)          │
│ - 存储 4000 个时间点                    │
│ - 线程安全                              │
└─────────┬───────────────────────────────┘
          │ snapshot() 每 150ms
          ↓
┌─────────────────────────────────────────┐
│ RemoteObserverViewer                    │
│ - 数据队列 (IPC)                        │
│ - 喂养线程 (主线程)                     │
└─────────┬───────────────────────────────┘
          │ matplotlib 进程 (独立进程)
          ↓
┌─────────────────────────────────────────┐
│ _run_observer_plot_process              │
│ - matplotlib FuncAnimation (6.7 Hz)    │
│ - 绘制舰队轨迹、速度、共识等            │
│ - TkAgg 后端显示                        │
└─────────────────────────────────────────┘
          │
          ↓
      [实时图表窗口]
```

---

## 📊 时间同步

### **时间基准：**
```python
# 远程 Scope Manager
_observer_plot_time_origin = time.monotonic()  # 初始化时的单调时间

# 遥测时间调整
telemetry_for_plot["time"] = time.monotonic() - _observer_plot_time_origin
# 结果: 从 0 开始计时的相对时间
```

### **时间参考：**
- **绝对时间：** `timestamp` (UNIX epoch)
- **相对时间：** `time` (相对于绘图启动时的秒数)
- **绘图使用：** 相对时间（便于实时显示）

---

## 🎯 Plot All Observer 与其他录制方式的区别

| 特性 | Plot All Observer | CSV 录制 | 单车 Scope |
|------|------------------|--------|----------|
| **数据来源** | 车辆遥测包 (20Hz) | 本地状态估计器 (10Hz) | 预设信号流 |
| **数据类型** | 分布式观察器 debug | 位置/速度/控制 | 自定义预设 |
| **存储位置** | 内存缓冲 (4000 点) | CSV 文件 | 内存缓冲 |
| **显示位置** | GUI 独立进程 | 离线分析 | GUI 内嵌 |
| **更新频率** | ~6.7 Hz 图表 | 点记录 10Hz | 20 Hz |
| **时间窗口** | 全部/自定义 | 整个运行 | 可配置 |
| **用途** | 实时舰队监测 | 事后分析 | 单车诊断 |

---

## 🔌 信号过滤规则

从遥测中提取数值信号的规则（在 `ObserverDataBuffer.append()` 中）：

```python
OBSERVER_NUMERIC_FIELD_PREFIXES = (
    "x_vec_after_",      # ← 观察器状态向量 (更新后)
    "x_vec_before_",     # ← 观察器状态向量 (更新前)
    "dynamics_",         # ← 动力学模型预测
    "measurement_",      # ← 测量信号
    "consensus_",        # ← 共识状态
    "true_position_",    # ← 真实位置 (仿真)
    "true_velocity_",    # ← 真实速度 (仿真)
    "true_acceleration_", # ← 真实加速度 (仿真)
)

OBSERVER_NUMERIC_EXACT_KEYS = (
    "x", "y", "th", "v", "u", "delta", "acceleration", "control_input",
)
```

只有匹配这些前缀的字段才会被缓冲和绘制。

---

## 总结

**Plot All Observer 实时绘图数据来源：**

1. ✅ **主要来源：** `vehicle_logic._collect_distributed_debug_telemetry()`
2. ✅ **从何获取：** 分布式 Luenberger 观察器的内部 debug 数据
3. ✅ **传输方式：** WebSocket JSON 遥测包 (20Hz)
4. ✅ **缓冲存储：** ObserverDataBuffer (环形缓冲 4000 点)
5. ✅ **实时渲染：** matplotlib 独立进程 (6.7 Hz 更新)
6. ✅ **核心信号：** 舰队位置、速度、共识误差、信任权重

