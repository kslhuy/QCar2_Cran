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
                    
                    # Parse QCAR_IPS as list
                    if key == 'QCAR_IPS':
                        # Remove brackets and split by comma
                        value = value.strip('[]')
                        config[key] = [ip.strip() for ip in value.split(',')]
                    else:
                        config[key] = value
    
    return config

# Read configuration
config = read_config(args.config)

# Extract configuration values
QCAR_IPS = config.get('QCAR_IPS', [])
LOCAL_IP = config.get('LOCAL_IP', '192.168.2.200')
PROBING_IP = config.get('PROBING_IP', QCAR_IPS[0] if QCAR_IPS else None)
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
print(f"  Probing QCar: {PROBING_IP}")
print(f"  Remote Path: {REMOTE_PATH}")
print(f"  Base Port: {BASE_PORT}")
print(f"  Observer Size: {WIDTH}x{HEIGHT}")
print(f"  Skip Upload: {args.skip_upload}")
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
    """Upload Python, YAML, and text files to QCar"""
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
    
    # Upload markdown documentation (optional)
    md_files = glob.glob(os.path.join(SCRIPTS_PATH, "*.md"))
    for file in md_files:
        scp.put(file, REMOTE_PATH)
    if md_files:
        print(f"  [✓] Uploaded {len(md_files)} documentation files")

# --- Start observer process locally ---
observer_process = None

# --- Main startup sequence ---
try:
    for idx, ip in enumerate(QCAR_IPS):
        car_id = idx
        port = BASE_PORT + car_id
        is_probing = (ip == PROBING_IP)
        
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
            
            # Start observer locally if this is the probing QCar
            if is_probing and observer_process is None:
                print(f"  [→] Starting observer.py locally (probing mode)...")
                observer_script = os.path.join(CURRENT_DIR, LOCAL_OBSERVER_PATH)
                if os.path.exists(observer_script):
                    observer_process = subprocess.Popen(["python", observer_script])
                    print(f"  [✓] Observer started")
                else:
                    print(f"  [⚠] Observer script not found: {observer_script}")
            
            # Kill any existing processes on the QCar
            print(f"  [→] Stopping any existing processes...")
            ssh.exec_command(f"pkill -f vehicle_control")
            ssh.exec_command(f"pkill -f yolo_server")
            time.sleep(1)
            
            # Start vehicle_control_refactored.py remotely
            print(f"  [→] Starting vehicle_control_refactored.py...")
            cmd_vehicle = (
                f"cd {REMOTE_PATH} && "
                f"nohup python vehicle_control_refactored.py "
                f"--host {LOCAL_IP} "
                f"--port {BASE_PORT} "
                f"--car-id {car_id} "
                f"> vehicle_{car_id}.log 2>&1 &"
            )
            ssh.exec_command(cmd_vehicle)
            print(f"  [✓] Vehicle control started (Car ID: {car_id})")
            
            time.sleep(2)  # Give vehicle control time to initialize
            
            # Start yolo_server.py remotely
            print(f"  [→] Starting yolo_server.py...")
            probing_flag = "True" if is_probing else "False"
            cmd_yolo = (
                f"cd {REMOTE_PATH} && "
                f"nohup python yolo_server.py "
                f"-p {probing_flag} "
                f"-i {LOCAL_IP} "
                f"-w {WIDTH} "
                f"-ht {HEIGHT} "
                f"> yolo_{car_id}.log 2>&1 &"
            )
            ssh.exec_command(cmd_yolo)
            print(f"  [✓] YOLO server started (Probing: {is_probing})")
            
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
    if observer_process:
        print("  Stopping observer process...")
        observer_process.terminate()
