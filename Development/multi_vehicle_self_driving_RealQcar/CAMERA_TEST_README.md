# YOLO Camera Testing Guide

This guide explains different methods to test if the YOLO camera is working on your QCar.

## Prerequisites

- QCar is powered on and connected to your network
- You know the QCar's IP address (check `config.txt`)
- SSH access to the QCar is configured (run `ssh_setup.bat` first if needed)

## Method 1: Visual Test with Observer (Recommended)

This method shows you the camera feed with YOLO detections in real-time.

### Steps:

1. Open a terminal in this directory
2. Run the test batch file:
   ```batch
   .\test_yolo_camera.bat
   ```

3. What to expect:
   - An "Observer" window will open on your computer
   - You should see live camera feed from the QCar
   - Objects detected by YOLO will be highlighted with bounding boxes
   - FPS counter should be displayed

4. Press `Ctrl+C` in the terminal to stop

### Troubleshooting:
- **No image appears**: Check network connection and firewall settings
- **Black screen**: Camera might be blocked or not properly connected
- **Low FPS**: Normal, YOLO processing is computationally intensive

---

## Method 2: Quick Camera Test (No YOLO)

This tests just the camera hardware without YOLO processing.

### Via SSH (Manual):

```batch
ssh nvidia@<QCAR_IP>
cd /home/nvidia/Documents
python test_camera.py
```

### Using Python Script:

```batch
python python\remote_camera_test.py -ip 192.168.2.108 -rp /home/nvidia/Documents
```

Replace `192.168.2.108` with your QCar's IP from `config.txt`.

### Expected Output:
```
===========================================================
YOLO Camera Test Script
===========================================================

Initializing camera...
✓ Camera initialized successfully

Testing camera capture...
Press Ctrl+C to exit

Frame 1: ✓ RGB (480, 640, 3), Depth (480, 640)
Frame 2: ✓ RGB (480, 640, 3), Depth (480, 640)
...
Frame 10: ✓ RGB (480, 640, 3), Depth (480, 640)

===========================================================
Results: 10/10 frames captured successfully
✓ Camera is working properly!
===========================================================
```

---

## Method 3: Direct SSH Test

For advanced users who want direct control:

```batch
ssh nvidia@<QCAR_IP>
cd /home/nvidia/Documents

# Test camera only
python test_camera.py

# Test YOLO server (no observer)
python yolo_server.py -p False -i 192.168.2.200

# Test YOLO server with observer
python yolo_server.py -p True -i <YOUR_PC_IP> -w 320 -ht 200
```

---

## Common Issues

### Issue 1: "No module named 'pit'"
**Solution**: YOLO libraries not installed. Run on QCar:
```bash
pip install <required_packages>
```

### Issue 2: "Camera port already in use"
**Solution**: Another process is using the camera. Kill it:
```bash
pkill -f yolo_server.py
```

### Issue 3: Observer shows nothing
**Solutions**:
1. Check firewall allows UDP traffic on port used by Probe
2. Verify your PC IP in config.txt is correct
3. Make sure both devices are on the same network

### Issue 4: "Connection refused"
**Solution**: 
1. Verify QCar IP: `ping <QCAR_IP>`
2. Check SSH access: `ssh nvidia@<QCAR_IP>`
3. Password is: `nvidia`

---

## Configuration File

Check `config.txt` for your current settings:

```ini
REMOTE_PATH=/home/nvidia/Documents
QCAR_IPS=[192.168.2.108]
CALIBRATE_IP=192.168.2.108
PROBING_IP=192.168.2.108
LOCAL_IP=192.168.2.200
WIDTH=320
HEIGHT=200
```

- **PROBING_IP**: QCar that sends camera feed to observer
- **LOCAL_IP**: Your PC's IP address (must be correct!)
- **WIDTH/HEIGHT**: Observer window size

---

## Quick Checklist

- [ ] QCar is powered on
- [ ] QCar is connected to network
- [ ] Can ping QCar IP
- [ ] SSH keys are set up (`ssh_setup.bat` run)
- [ ] Required scripts uploaded to QCar
- [ ] Firewall allows UDP traffic
- [ ] LOCAL_IP in config.txt matches your PC

---

## Need Help?

1. Check the log files in the `logs/` directory
2. Try the simplest test first (Method 2)
3. Verify hardware connections on the QCar
4. Check camera is not physically blocked
