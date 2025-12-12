"""
Start script for multi-vehicle refactored control system
Automatically reads configuration from config.txt and starts all vehicles
"""
import paramiko
import argparse
from scp import SCPClient
import subprocess
import os
import glob
import time
import re

# --- Parse command line arguments ---
parser = argparse.ArgumentParser(description="Run refactored vehicle control on multiple QCars.")
parser.add_argument('-c', '--config', type=str, default='../config.txt', 
                    help="Path to config.txt file (default: ../config.txt)")
parser.add_argument('--skip-upload', action='store_true', 
                    help="Skip uploading files (useful for quick restart)")
parser.add_argument('--no-logs', action='store_true', 
                    help="Disable log file creation on vehicles")
args = parser.parse_args()

# --- Read configuration from config.txt ---
def read_config(config_path):
    """Read configuration from config.txt file"""
    config = {}
    config_full_path = os.path.normpath(os.path.join(os.path.dirname(__file__), config_path))
    
    print(f"Reading configuration from: {config_full_path}")
    
    with open(config_full_path, 'r') as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#'):
                if '=' in line:
                    key, value = line.split('=', 1)
                    key = key.strip()
                    value = value.strip()
                    
                    # Parse QCAR_IPS and PROBING_IP as lists
                    if key == 'QCAR_IPS':
                        # Remove brackets and split by comma
                        value = value.strip('[]')
                        config[key] = [ip.strip() for ip in value.split(',')]
                    elif key == 'PROBING_IP':
                        # Parse PROBING_IP as list (can be multiple IPs)
                        if ',' in value:
                            config[key] = [ip.strip() for ip in value.split(',')]
                        else:
                            config[key] = [value.strip()]
                    else:
                        config[key] = value
    
    return config

# Read configuration
config = read_config(args.config)

# Extract configuration values
QCAR_IPS = config.get('QCAR_IPS', [])
LOCAL_IP = config.get('LOCAL_IP', '192.168.2.200')
PROBING_IPS = config.get('PROBING_IP', [QCAR_IPS[0]] if QCAR_IPS else [])
REMOTE_PATH = config.get('REMOTE_PATH', '/home/nvidia/Documents/multi_vehicle_RealCar')
WIDTH = config.get('WIDTH', '320')
HEIGHT = config.get('HEIGHT', '200')

# Constants
USERNAME = "nvidia"
PASSWORD = "nvidia"
BASE_PORT = 5000
LOCAL_SCRIPTS_PATH = "../qcar"
LOCAL_OBSERVER_PATH = "observer.py"
CURRENT_DIR = os.path.dirname(os.path.realpath(__file__))
SCRIPTS_PATH = os.path.normpath(os.path.join(CURRENT_DIR, LOCAL_SCRIPTS_PATH))

# --- Display configuration ---
print("="*70)
print(" Multi-Vehicle Refactored Control System - Startup")
print("="*70)
print(f"\nConfiguration:")
print(f"  Number of QCars: {len(QCAR_IPS)}")
print(f"  QCar IPs: {', '.join(QCAR_IPS)}")
print(f"  Host PC IP: {LOCAL_IP}")
print(f"  Probing QCars: {', '.join(PROBING_IPS)}")
print(f"  Remote Path: {REMOTE_PATH}")
print(f"  Base Port: {BASE_PORT}")
print(f"  Observer Size: {WIDTH}x{HEIGHT}")
print(f"  Skip Upload: {args.skip_upload}")
print(f"  Remote Logging: {'Disabled' if args.no_logs else 'Enabled'}")
print("="*70)

# --- Helper to create SSH + SCP connections ---
def create_ssh_and_scp(ip):
    """Create SSH and SCP connections to QCar"""
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(ip, username=USERNAME, password=PASSWORD, timeout=10)
    scp = SCPClient(ssh.get_transport())
    return ssh, scp

# --- Upload files to QCar ---
def upload_files(ip, scp):
    """Upload Python, YAML, text files, and StateMachine folder to QCar"""
    # Upload Python files
    py_files = glob.glob(os.path.join(SCRIPTS_PATH, "*.py"))
    for file in py_files:
        scp.put(file, REMOTE_PATH)
    print(f"  [✓] Uploaded {len(py_files)} Python files")
    
    # Upload YAML configuration files
    yaml_files = (glob.glob(os.path.join(SCRIPTS_PATH, "*.yaml")) + 
                  glob.glob(os.path.join(SCRIPTS_PATH, "*.yml")))
    for file in yaml_files:
        scp.put(file, REMOTE_PATH)
    print(f"  [✓] Uploaded {len(yaml_files)} YAML configuration files")
    
    # Upload text files (requirements, etc.)
    txt_files = glob.glob(os.path.join(SCRIPTS_PATH, "*.txt"))
    for file in txt_files:
        scp.put(file, REMOTE_PATH)
    print(f"  [✓] Uploaded {len(txt_files)} text files")
    
    # # Upload markdown documentation (optional)
    # md_files = glob.glob(os.path.join(SCRIPTS_PATH, "*.md"))
    # for file in md_files:
    #     scp.put(file, REMOTE_PATH)
    # if md_files:
    #     print(f"  [✓] Uploaded {len(md_files)} documentation files")
    
    ########## UPDATED Folder BELOW ##########

    
    # Upload additional folders: Yolo, Observer, V2V, Controller
    additional_folders = ["StateMachine" , "Yolo", "Observer", "V2V", "Controller"]
    for folder_name in additional_folders:
        folder_path = os.path.join(SCRIPTS_PATH, folder_name)
        if os.path.exists(folder_path):
            try:
                # Upload the entire folder directory recursively
                scp.put(folder_path, REMOTE_PATH, recursive=True)
                
                # Count files for reporting
                folder_files = []
                for root, dirs, files in os.walk(folder_path):
                    for file in files:
                        if not file.endswith('.pyc') and '__pycache__' not in root:
                            folder_files.append(file)
                
                print(f"  [✓] Uploaded {folder_name} folder with {len(folder_files)} files")
            except Exception as e:
                print(f"  [⚠] Error uploading {folder_name} folder: {e}")
        else:
            print(f"  [⚠] {folder_name} folder not found at {folder_path}")

# --- Start observer process locally ---
probing_process = None

# --- Main startup sequence ---
try:
    for idx, ip in enumerate(QCAR_IPS):
        car_id = idx
        port = BASE_PORT + car_id
        is_probing = (ip in PROBING_IPS)
        
        print(f"\n{'='*70}")
        print(f" Starting QCar {car_id}: {ip} (Port: {port})")
        print(f"{'='*70}")
        
        try:
            # Create SSH connection
            print(f"  [→] Connecting to {ip}...")
            ssh, scp = create_ssh_and_scp(ip)
            print(f"  [✓] Connected")
            
            # Upload files (unless skipped)
            if not args.skip_upload:
                print(f"  [→] Uploading files...")
                upload_files(ip, scp)
            else:
                print(f"  [⊘] Skipped file upload")
            
            # Start multi-probing locally if we have probing vehicles (Client Side (Ground Station))
            if is_probing and probing_process is None:
                print(f"  [→] Starting multi-probing.py locally...")
                # Count how many vehicles are probing
                # probing_count = sum(1 for ip in QCAR_IPS if ip in PROBING_IPS)
                probing_script = os.path.join(CURRENT_DIR, "multi_probing.py")
                if os.path.exists(probing_script):
                    probing_process = subprocess.Popen([
                        "python", probing_script, 
                        "--cars", str(idx),
                        "--width", WIDTH, 
                        "--height", HEIGHT
                    ])
                    print(f"  [✓] multi-probing started for {idx} vehicle")
                else:
                    print(f"  [⚠] multi-probing script not found: {probing_script}")
            
            # Kill any existing processes on the QCar
            print(f"  [→] Stopping any existing processes...")
            ssh.exec_command(f"pkill -f vehicle_control")
            ssh.exec_command(f"pkill -f yolo_server")
            time.sleep(1)
            
            # These are shell redirection operators used in Linux/Unix commands:

            # > /dev/null 2>&1 &
            # > - Redirects standard output (stdout)
            # /dev/null - A special file that discards all data written to it (like a black hole)
            # 2>&1 - Redirects standard error (stderr, file descriptor 2) to the same destination as stdout (file descriptor 1)
            # & - Runs the process in the background
            # Result: All output (both normal output and errors) is discarded, nothing is saved.

            # Start vehicle_main.py remotely
            print(f"  [→] Starting vehicle_main.py...")
            if args.no_logs:
                cmd_vehicle = (
                    f"cd {REMOTE_PATH} && "
                    f"nohup python vehicle_main.py "
                    f"--host {LOCAL_IP} "
                    f"--port {BASE_PORT} "
                    f"--car-id {car_id} "
                    f"> /dev/null 2>&1 &"
                )
            else:
                
            # > vehicle_{car_id}.log 2>&1 &
            # > vehicle_{car_id}.log - Redirects stdout to a log file (e.g., vehicle_0.log)
            # 2>&1 - Redirects stderr to the same file as stdout
            # & - Runs the process in the background
            # Result: All output (both normal output and errors) is saved to the log file for later review.
                cmd_vehicle = (
                    f"cd {REMOTE_PATH} && "
                    f"nohup python vehicle_main.py "
                    f"--host {LOCAL_IP} "
                    f"--port {BASE_PORT} "
                    f"--car-id {car_id} "
                    f"> vehicle_{car_id}.log 2>&1 &"
                )
            ssh.exec_command(cmd_vehicle)
            print(f"  [✓] Vehicle control started (Car ID: {car_id})")
            
            time.sleep(2)  # Give vehicle control time to initialize
            
            # Start yolo_server.py remotely (Run the YOLO server on the QCar)
            print(f"  [→] Starting yolo_server.py...")
            probing_flag = "True" if is_probing else "False"
            if args.no_logs:
                cmd_yolo = (
                    f"cd {REMOTE_PATH} && "
                    f"nohup python yolo_server.py "
                    f"-p {probing_flag} "
                    f"-i {LOCAL_IP} "
                    f"-w {WIDTH} "
                    f"-ht {HEIGHT} "
                    f"--caridx {car_id} "
                    f"> /dev/null 2>&1 &"
                )
            else:
                cmd_yolo = (
                    f"cd {REMOTE_PATH} && "
                    f"nohup python yolo_server.py "
                    f"-p {probing_flag} "
                    f"-i {LOCAL_IP} "
                    f"-w {WIDTH} "
                    f"-ht {HEIGHT} "
                    f"--caridx {car_id} "
                    f"> yolo_{car_id}.log 2>&1 &"
                )
            ssh.exec_command(cmd_yolo)
            print(f"  [✓] YOLO server started (Probing: {is_probing}, Car ID: {car_id})")
            
            # Close connections
            ssh.close()
            scp.close()
            
            print(f"  [✓] QCar {car_id} ({ip}) startup complete")
            
        except Exception as e:
            print(f"  [✗] Error starting QCar {car_id} ({ip}): {e}")
            continue
        
        time.sleep(3)  # Wait before starting next QCar
    
    # Summary
    print(f"\n{'='*70}")
    print(" Startup Complete")
    print(f"{'='*70}")
    print(f"\nAll {len(QCAR_IPS)} QCars have been started:")
    for idx, ip in enumerate(QCAR_IPS):
        print(f"  • QCar {idx}: {ip} (Port: {BASE_PORT + idx})")
    
    print(f"\nHost PC: {LOCAL_IP}")
    print(f"\nLogs on each QCar:")
    for idx in range(len(QCAR_IPS)):
        print(f"  • vehicle_{idx}.log - Vehicle control log")
        print(f"  • yolo_{idx}.log - YOLO server log")
    
    print(f"\nTo view logs on QCar:")
    print(f"  ssh nvidia@<qcar_ip>")
    print(f"  cd {REMOTE_PATH}")
    print(f"  tail -f vehicle_0.log")
    
    print(f"\nTo stop all vehicles, run: python stop.py --config {args.config}")
    print("="*70)
    
    input("\nPress Enter to exit...")

except KeyboardInterrupt:
    print("\n\n[!] Startup interrupted by user")
except Exception as e:
    print(f"\n[✗] Fatal error: {e}")
    import traceback
    traceback.print_exc()
finally:
    # Cleanup
    print("\nCleaning up...")
    if probing_process:
        print("  Stopping probing process...")
        probing_process.terminate()
