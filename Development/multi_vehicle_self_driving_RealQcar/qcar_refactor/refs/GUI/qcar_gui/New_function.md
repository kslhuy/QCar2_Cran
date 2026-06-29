New Features Added
1. Deploy Vehicles Tab 📡
A new tabbed interface with two tabs:

Connected Vehicles - The existing functionality (cars that connect automatically)
Deploy Vehicles - New panel for manually connecting to real QCars
2. Vehicle Connection Panels 🚗
Each vehicle slot includes:

IP Address input - Enter the QCar's IP (default: 192.168.2.1XX)
Vehicle Type selector - QCar or Limo
Path Number - Which path to follow
Initial Velocity - Starting speed (m/s)
Checkboxes for: Probing (YOLO), GPS Calibration, Left-hand traffic
Action Buttons:
🔍 Test - Test SSH connection
🔌 Connect - Establish SSH connection
📤 Upload Files - Transfer Python scripts, YAML configs, and folders
▶️ Start Vehicle - Launch vehicle_main.py on the QCar
⬛ Stop Vehicle - Kill the vehicle program
3. Batch Operations 🎛️
Master controls for fleet deployment:

Connect All - Connect to all configured vehicles
Upload All - Upload files to all connected vehicles
Start All - Start programs on all vehicles
Stop All - Stop all running vehicles
Add/Remove Vehicle - Dynamically add or remove vehicle slots
4. New Files Created
widgets/vehicle_connection_panel.py - UI panels
controllers/vehicle_connector.py - SSH/SCP service