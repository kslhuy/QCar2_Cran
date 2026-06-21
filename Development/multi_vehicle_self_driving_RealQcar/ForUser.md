# QCar Multi-Vehicle Self-Driving System 

### ⚡ For real Qcar 

```powershell

Edit `config.txt` with your IPs , and the real car

#  Start GUI  (another cmd)
cd .\Development\multi_vehicle_self_driving_RealQcar\qcar\GUI\      
python .\app_main.py
# You can connect fully with the GUI 


# -----  OLDS stuff -------
# Start all vehicles (in another terminal)
cd .\Development\multi_vehicle_self_driving_RealQcar      

# .\start_refactored.bat 
.\start_enhanced.bat 
fleet_config.yaml is that file need to put correct IP adresse

(Wait its will run automatique)

# To Stop all vehicles (in same terminal of the start)
cd .\Development\multi_vehicle_self_driving_RealQcar      

.\stop_enhanced.bat   ## Stop all the python process (logic is shutdown) = stop_refactored
.\stop_all_cars.bat   ## Stop all physical part ( lidars  )

# For probing
python multi_probing.py --car 0

```

### For Simulation Qlabs
 
```powershell


# (In Simulator) # For spawn 2 Qcar
cd .\Development\QCar2_multi-vehicle_control\
# Spawn 3 vehicle for platoon
python .\initCars_Studio.py   -u
# Spawn 1 vehicle   
python .\initCars_Studio.py   -u -n 1  
python .\initCars_Studio.py   -u -n 3 

#  Start GUI  (another cmd)
cd .\Development\multi_vehicle_self_driving_RealQcar\qcar\GUI\      
python .\app_main.py
cd .\GroundStation-Qcar-App\
# Start the development server
npm run dev

# Run the real logic of vehicle 0   (another cmd)

cd .\Development\multi_vehicle_self_driving_RealQcar\qcar      
python .\vehicle_main.py --host 127.0.0.1 --port 5000 --car-id 0

# Run the real logic of vehicle 1 (another cmd) 

cd .\Development\multi_vehicle_self_driving_RealQcar\qcar      
python .\vehicle_main.py --host 127.0.0.1 --port 5000 --car-id 1
python .\vehicle_main.py --host 127.0.0.1 --port 5000 --car-id 2

python .\vehicle_main.py --host 127.0.0.1 --port 5000 --car-id 3

# Should enable probing in the Development\multi_vehicle_self_driving_RealQcar\qcar\fleet_config.yaml
# For probing
python multi_probing.py --car 0


Development\multi_vehicle_self_driving_RealQcar\qcar\Observer\TrustbasedDistributedObserver


(Logs file is in same folder (\Development\multi_vehicle_self_driving_RealQcar\qcar))
```

### For CARLA Simulation (Windows Python, no ROS)

This is the new CARLA bridge mode. It runs CARLA locally on Windows and makes
the CARLA ego vehicle look like the existing QCar hardware/GPS interface, so
`VehicleLogic`, the observer, state machine, controllers, telemetry, and
ground-station commands can stay mostly the same.

Requirements:
- Windows 11
- Python 3.10
- CARLA UE5 server running locally
- CARLA Python API installed in the same Python environment
- No ROS 2 required for this mode

```powershell
# Start CARLA UE5 first, then run this in another terminal.
cd .\Development\multi_vehicle_self_driving_RealQcar\qcar

# Smoke test: one ego, CARLA sync mode, auto start, no ground station.
python .\vehicle_main_carla.py --auto-start --spawn-point-index 0 --goal-spawn-indices 10 --draw-route --no-ground-station
```

If you want to connect to the Ground Station GUI:

```powershell
# Terminal 1: start GUI
cd .\Development\multi_vehicle_self_driving_RealQcar\qcar\GUI
python .\app_main.py

# Terminal 2: run CARLA bridge with Ground Station connection
cd .\Development\multi_vehicle_self_driving_RealQcar\qcar
python .\vehicle_main_carla.py --auto-start --ground-station-host 127.0.0.1 --ground-station-port 5000 --car-id 0
```

Useful options:

```powershell
# Use another CARLA RPC host/port
python .\vehicle_main_carla.py --host 127.0.0.1 --port 2000

# Attach to an already spawned CARLA vehicle with role_name="ego"
python .\vehicle_main_carla.py --use-existing-ego --ego-role-name ego --auto-start

# Select route by CARLA spawn point indices
python .\vehicle_main_carla.py --spawn-point-index 5 --goal-spawn-indices 20,35 --draw-route --auto-start

# Tune speed and CARLA control conversion
python .\vehicle_main_carla.py --v-ref 0.8 --command-to-speed-gain 6.0 --max-target-speed 2.5 --auto-start

# Show all options
python .\vehicle_main_carla.py --help
```

What this CARLA mode currently does:
- Connects to `localhost:2000` by default
- Spawns or selects one ego vehicle
- Spawns RGB camera, lidar, IMU, and collision sensors
- Runs CARLA in synchronous mode by default with `--fixed-dt 0.05`
- Generates `VehicleLogic.waypoint_sequence` from CARLA waypoints/global route planner
- Uses default `stanley` lateral control and `pid` longitudinal control
- Converts QCar-style throttle output to CARLA throttle/brake using a speed PID
- Disables YOLO in v1 and uses default `YOLOData`
- Converts lidar points into the existing 10-float opponent payload format
- Stops the vehicle on collision or Ctrl+C

Notes:
- If you see `CARLA Python API is not importable`, install the CARLA Python package for the Python version used to run `vehicle_main_carla.py`.
- If you see `MPC controller not available: No module named 'casadi'`, it is not blocking unless you select `--path-lateral-controller mpc`.
- v1 is single-ego only. Multi-car V2V and ROS sensor topics can be added after the single-car bridge is stable.


### For Calibration

# In separate terminal
```powershell
cd C:\Users\Quang Huy Nugyen\Desktop\PHD_paper\Simulation\QCAR\QCar2_Cran\Development\multi_vehicle_self_driving_RealQcar\qcar\Calibration\On_Track_SysID\src

python online_sysid_zmq_worker.py --sample-port 18880 --control-port 18881 --status-port 18882 --plot-model
```

### Development Quick Test Mode (Fake Vehicle use math equation like We do with Matlab)
```powershell
# Start GUI 

cd .\Development\multi_vehicle_self_driving_RealQcar\qcar\GUI     
python .\app_main.py


# Test with fake vehicles (no hardware , no Qlabs , math equation)
cd .\Development\multi_vehicle_self_driving_RealQcar\qcar\simulation    
python fake_vehicle_real_logic.py 0

# Test with fake vehicles (with parameter Qcar ) (no hardware , no Qlabs)
python fake_vehicle_real_logic.py 1




# — Limo System ROS

## Architecture

```
SDCQcar  ──static TF──▶  map  ──▶  odom  ──▶  base_link
 (QCar world)            (ROS world)
```

- **SDCQcar**: SDCSRoadMap coordinates (waypoints, QCar logic)
- **map**: AMCL / Nav2 / RViz (normal ROS)
- One static TF connects them — defined in the launch file


## Step 2 — Rebuild after calibration
# Rebuild only limo_nav_huy_test
```bash
cd ~/agilex_ws
colcon build --packages-select limo_nav_huy_test --symlink-install
source install/setup.bash
```

# Rebuild all packages necessaires
colcon build --packages-select limo_base limo_bringup limo_nav_huy_test --symlink-install

---
## Step 3 — Run the full system (new)

### Terminal 0 — Start the robot
```bash
ros2 launch limo_bringup limo_start.launch.py
```
# Terminal 1 (without vehicle_main)
<!-- ```bash
ros2 launch limo_nav_huy_test navigationV2V_qcar_frames.launch.py start_vehicle_main:=false
``` -->

-- Slam toolbox localization (new)
```bash
ros2 launch limo_nav_huy_test localization_slam_toolbox_qcar.launch.py
```

# Terminal 2 (vehicle_main separate)
```bash
ros2 run limo_nav_huy_test vehicle_main_ros_qcar
```

### Terminal 3 — (Optional) RViz
```bash
ros2 run rviz2 rviz2 -d ~/agilex_ws/install/limo_nav_huy_test/share/limo_nav_huy_test/rviz/nav2_copy.rviz
```

# Terminal 4  -  (Optional) 
Send an SDC pose to trigger AMCL init (acts like RViz 2D Pose Estimate):

```bash
ros2 topic pub --once /initialpose_sdc geometry_msgs/msg/PoseStamped \
"{header: {frame_id: SDCQcar}, pose: {position: {x: -1.28205, y: -0.45991, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.358, w: 0.934}}}"
```

## QCAR ROS 

### Terminal 1

```bash
cd /home/nvidia/Documents/qcar2/Development/ros2
source install/setup.bash
ros2 launch ros2test localization_cartographer_qcar.launch.py
  # pbstream:=/absolute/path/to/your_map.pbstream
```

### Terminal 2

```bash
cd /home/nvidia/Documents/qcar2/Development/ros2
source install/setup.bash

```

Copy-safe one-line equivalent:

```bash
ros2 run ros2test vehicle_main_ros_qcar --ros-args -p car_id:=3 -p host:=192.168.137.1 -p v_ref:=0.6 -p vehicle_type:=Qcar
```
