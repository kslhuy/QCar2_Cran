# Quick Start Guide - Refactored Vehicle Control

## Installation

1. **Install additional dependencies:**
   ```powershell
   pip install -r requirements_refactored.txt
   ```

2. **Verify installation:**
   ```powershell
   python -c "import yaml; print('YAML support: OK')"
   ```

## Running the System

### Option 1: Basic Run (Single Car)
```powershell
cd qcar
python vehicle_control_refactored.py --car-id 0
```

### Option 2: With Custom Configuration
```powershell
# Copy and edit the example config
cp config_example.yaml my_config.yaml
# Edit my_config.yaml with your parameters
python vehicle_control_refactored.py --config my_config.yaml
```

### Option 3: Multi-Vehicle Setup
```powershell
# Terminal 1 - Car 0
python vehicle_control_refactored.py --car-id 0 --port 5000

# Terminal 2 - Car 1
python vehicle_control_refactored.py --car-id 1 --port 5001

# Terminal 3 - Car 2
python vehicle_control_refactored.py --car-id 2 --port 5002
```

### Option 4: With Remote Control
```powershell
python vehicle_control_refactored.py --host 192.168.1.100 --car-id 0
```

## Common Use Cases

### 1. Quick Test Run
```powershell
# Run for 60 seconds with low speed
python vehicle_control_refactored.py --v-ref 0.5
```

### 2. Debug Mode
Create `debug_config.yaml`:
```yaml
logging:
  log_level: "DEBUG"
  console_output: true

timing:
  tf: 300  # 5 minutes
```

Run:
```powershell
python vehicle_control_refactored.py --config debug_config.yaml
```

### 3. Performance Testing
```powershell
# High frequency control
python vehicle_control_refactored.py --config high_perf_config.yaml
```

`high_perf_config.yaml`:
```yaml
timing:
  controller_update_rate: 500
  telemetry_send_rate: 50
```

## Monitoring

### Real-time Logs
```powershell
# Watch main log
Get-Content logs\vehicle_0_*.log -Wait -Tail 50

# Watch telemetry
Get-Content data_logs\run_*\telemetry_vehicle_0.csv -Wait -Tail 20
```

### Post-Analysis
```powershell
# View last run telemetry
$latest = Get-ChildItem data_logs | Sort-Object LastWriteTime -Descending | Select-Object -First 1
Import-Csv "$($latest.FullName)\telemetry_vehicle_0.csv" | Format-Table
```

## Testing

### Run All Tests
```powershell
python -m pytest test_components.py -v
```

### Run Specific Test
```powershell
python -m pytest test_components.py::TestControllers -v
```

### Test Coverage
```powershell
pip install pytest-cov
python -m pytest test_components.py --cov=. --cov-report=html
```

## Troubleshooting

### Issue: "Import Error: yaml"
**Solution:**
```powershell
pip install pyyaml
```

### Issue: "YOLO server not responding"
**Solution:**
- Ensure YOLO server is running before starting vehicle control
- Check YOLO server IP and port configuration
- Verify network connectivity

### Issue: "GPS timeout"
**Solution:**
- Check GPS hardware connection
- Verify GPS calibration
- System will fall back to dead reckoning automatically

### Issue: "Network connection failed"
**Solution:**
- Verify host PC IP address
- Check firewall settings
- Ensure port is not in use: `netstat -an | findstr "5000"`

### Issue: "Control loop frequency too low"
**Solution:**
- Reduce `controller_update_rate` in config
- Check for system resource constraints
- Review performance logs for bottlenecks

## Configuration Tips

### Speed Tuning
```yaml
speed:
  v_ref: 0.75      # Start low, increase gradually
  K_p: 0.1         # Proportional gain (responsiveness)
  K_i: 1.0         # Integral gain (steady-state error)
  max_throttle: 0.3
```

**Tuning guidelines:**
- If oscillating: Decrease K_p
- If slow to reach target: Increase K_p or K_i
- If overshooting: Decrease K_i

### Steering Tuning
```yaml
steering:
  K_stanley: 0.7   # Stanley gain (path tracking)
```

**Tuning guidelines:**
- If cutting corners: Increase K_stanley
- If oscillating around path: Decrease K_stanley
- Typical range: 0.3 - 1.5

### Safety Settings
```yaml
safety:
  emergency_stop_distance: 0.2  # meters
  gps_timeout_max: 100         # iterations
```

**For different scenarios:**
- High speed: Increase emergency_stop_distance
- Unreliable GPS: Increase gps_timeout_max

## Migration from Original Code

### Step 1: Backup
```powershell
cp vehicle_control_real.py vehicle_control_real_backup.py
```

### Step 2: Create Config
Extract your current parameters to a config file:
```yaml
speed:
  v_ref: 0.75  # Your current v_ref
  K_p: 0.1     # Your current K_p
  K_i: 1.0     # Your current K_i
```

### Step 3: Test
```powershell
# Test with new system
python vehicle_control_refactored.py --config my_config.yaml

# Compare logs with original
```

### Step 4: Deploy
Once verified, update your launch scripts to use the new system.

## Performance Benchmarks

Expected performance on typical hardware:
- Control loop frequency: 200 Hz (sustained)
- Network latency: < 10 ms
- GPS update rate: 10 Hz
- Telemetry logging: No performance impact

## Advanced Features

### Remote Parameter Tuning
Send commands over network:
```json
{
  "v_ref": 0.8,
  "command": "resume"
}
```

### Custom State Callbacks
```python
def on_emergency_stop():
    print("Emergency stop activated!")
    # Custom logic here

controller.state_machine.register_on_enter(
    VehicleState.EMERGENCY_STOP,
    on_emergency_stop
)
```

### Performance Monitoring
Access real-time stats:
```python
stats = controller.perf_monitor.get_statistics()
print(f"Loop frequency: {stats['loop_time']['frequency']:.1f} Hz")
```

## Support

- **Documentation**: See REFACTORING_README.md
- **Example Config**: config_example.yaml
- **Test Suite**: test_components.py
- **Original Code**: vehicle_control_real.py (reference)

## Next Steps

1. ✅ Install dependencies
2. ✅ Run basic test
3. ✅ Create your configuration
4. ✅ Run with YOLO server
5. ✅ Monitor logs and telemetry
6. ✅ Tune parameters as needed
7. ⬜ Deploy multi-vehicle setup
8. ⬜ Implement custom features
