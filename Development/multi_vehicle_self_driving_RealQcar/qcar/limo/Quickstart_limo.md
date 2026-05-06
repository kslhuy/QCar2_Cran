# Quick Start — Limo + QCar SDCSRoadMap System

## Architecture

```
SDCQcar  ──static TF──▶  map  ──▶  odom  ──▶  base_link
 (QCar world)            (ROS world)
```

- **SDCQcar**: SDCSRoadMap coordinates (waypoints, QCar logic)
- **map**: AMCL / Nav2 / RViz (normal ROS)
- One static TF connects them — defined in the launch file

---

## Step 0 — Build

```bash
cd ~/agilex_ws
colcon build --packages-select limo_nav_huy_test --symlink-install
source install/setup.bash
```


---



---
## Step 2 — Run the full system (new)

### Terminal 0 — Start the robot
```bash
ros2 launch limo_bringup limo_start.launch.py
```
# Terminal 1 (without vehicle_main)
-- Amcl localization (old)
<!-- ```bash 
ros2 launch limo_nav_huy_test navigationV2V_qcar_frames.launch.py start_vehicle_main:=false
``` -->

-- Slam toolbox localization (new)
```bash
ros2 launch limo_nav_huy_test localization_slam_toolbox_qcar.launch.py
```

# Terminal 2 (vehicle_main separate)
```bash
ros2 run limo_nav_huy_test vehicle_main_ros_qcar --ros-args -p car_id:=3
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
---

## Step 1 — Calibrate (one time only)

Find the `SDCQcar → map` transform by aligning waypoints with your physical map.

### Terminal 1 — Start the robot
```bash
ros2 launch limo_bringup limo_start.launch.py
```

### Terminal 2 — Start map + AMCL + waypoint visualization
```bash
ros2 launch limo_nav_huy_test maptest_qcar.launch.py
```

### Terminal 3 — Run the alignment helper
```bash
ros2 run limo_nav_huy_test waypoint_alignment_helper
```

### In RViz
- Fixed Frame = `map`
- Add `/map` topic
- Add `/waypoints_test` (MarkerArray — cyan spheres)

### Adjust with keyboard
| Key | Action |
|-----|--------|
| `w/s` | Move Y ±0.1m |
| `a/d` | Move X ±0.1m |
| `W/S` | Move Y ±0.5m |
| `A/D` | Move X ±0.5m |
| `q/e` | Rotate ±5° |
| `Q/E` | Rotate ±15° |
| `z/x` | Scale ±0.05 |
| `p` | Print current values |
| **`c`** | **Print static TF command** |
| `ESC` | Quit |

### When aligned → press `c`

You will see output like:
```
--- PASTE INTO LAUNCH FILE ---
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='sdcqcar_to_map',
    arguments=['-2.1000', '0.1000', '0',
               '-4.4506', '0', '0', 'SDCQcar', 'map'],
    output='screen'
),
ros2 run tf2_ros static_transform_publisher \
  -1.3000 -1.2000 0 \
  2.0071 0 0 \
  SDCQcar map
```

### Paste into your launch file

Replace the `sdcqcar_to_map` Node in:
- `launch/navigationV2V_qcar_frames.launch.py`
- `launch/maptest_qcar.launch.py` (if you use it)

Then **stop the alignment helper**. You never run it again.

---

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



<!-- ## Step 3 — Run the full system old

### Terminal 1 — Start the robot
```bash
ros2 launch limo_bringup limo_start.launch.py
```

### Terminal 2 — Launch navigation + vehicle controller
```bash
ros2 launch limo_nav_huy_test navigationV2V_qcar_frames.launch.py
```

### Terminal 3 — (Optional) RViz
```bash
ros2 run rviz2 rviz2 -d ~/agilex_ws/install/limo_nav_huy_test/share/limo_nav_huy_test/rviz/nav2_copy.rviz
```

In RViz, set initial pose with `2D Pose Estimate` (this goes to AMCL normally).

--- -->

## Verify TF tree

```bash
ros2 run tf2_tools view_frames
# Expected: SDCQcar → map → odom → base_link
```

```bash
ros2 run tf2_ros tf2_echo SDCQcar base_link
# Should show the robot pose in SDCQcar coordinates
```

ros2 run tf_checker

---

ros2 topic list | grep odom
ros2 topic hz /wheel/odom
ros2 topic hz /laser/odom


ros2 topic list | grep odom
ros2 topic hz /wheel/odom
ros2 topic hz /laser/odom


ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom




-------- 
Create the Map (Option A)

Boot up your robot.
Run: ros2 launch limo_bringup limo_start.launch.py
Run: ros2 launch limo_bringup limo_slam_box.launch.py
Drive the robot around your track to map the environment fully.
Once mapped, open another terminal and save it: ros2 run nav2_map_server map_saver_cli -f /home/agilex/agilex_ws/bib_cran_map_2
Note: You can also use the SLAM Toolbox Rviz plugin "Save Map" and "Serialize Map" buttons and save it to /home/agilex/agilex_ws/bib_cran_map. This will generate the .posegraph and .data files.
Run the New Localization

Close everything except limo_start.launch.py.
Run the new launch file we created: ros2 launch limo_nav_huy_test localization_slam_toolbox_qcar.launch.py