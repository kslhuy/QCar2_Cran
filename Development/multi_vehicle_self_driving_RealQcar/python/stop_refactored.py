"""
Stop script for multi-vehicle refactored control system
Cleanly stops all vehicle control and YOLO processes on QCars
"""
import paramiko
import argparse
import os
import time

# --- Parse command line arguments ---
parser = argparse.ArgumentParser(description="Stop all QCar processes.")
parser.add_argument('-c', '--config', type=str, default='../config.txt', 
                    help="Path to config.txt file (default: ../config.txt)")
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
                        value = value.strip('[]')
                        config[key] = [ip.strip() for ip in value.split(',')]
                    else:
                        config[key] = value
    
    return config

# Read configuration
config = read_config(args.config)
QCAR_IPS = config.get('QCAR_IPS', [])
USERNAME = "nvidia"
PASSWORD = "nvidia"

# --- Display configuration ---
print("="*70)
print(" Multi-Vehicle Control System - Shutdown")
print("="*70)
print(f"\nStopping {len(QCAR_IPS)} QCars:")
for idx, ip in enumerate(QCAR_IPS):
    print(f"  • QCar {idx}: {ip}")
print("="*70)

# --- Helper to create SSH connection ---
def create_ssh(ip):
    """Create SSH connection to QCar"""
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(ip, username=USERNAME, password=PASSWORD, timeout=10)
    return ssh

# --- Stop processes on QCar ---
def stop_processes(ip, car_id):
    """Stop all vehicle control and YOLO processes on QCar"""
    try:
        print(f"\n[QCar {car_id}] Stopping processes on {ip}...")
        ssh = create_ssh(ip)
        
        # Kill vehicle control processes
        print(f"  [→] Stopping vehicle_control...")
        stdin, stdout, stderr = ssh.exec_command("pkill -f vehicle_control")
        stdout.channel.recv_exit_status()  # Wait for command to complete
        
        # Kill YOLO server processes
        print(f"  [→] Stopping yolo_server...")
        stdin, stdout, stderr = ssh.exec_command("pkill -f yolo_server")
        stdout.channel.recv_exit_status()
        
        # Verify processes stopped
        time.sleep(1)
        stdin, stdout, stderr = ssh.exec_command("pgrep -f 'vehicle_control|yolo_server'")
        remaining = stdout.read().decode().strip()
        
        if remaining:
            print(f"  [⚠] Some processes may still be running")
            # Force kill if needed
            print(f"  [→] Force stopping remaining processes...")
            ssh.exec_command("pkill -9 -f vehicle_control")
            ssh.exec_command("pkill -9 -f yolo_server")
            time.sleep(1)
        
        print(f"  [✓] QCar {car_id} ({ip}) stopped successfully")
        
        ssh.close()
        return True
        
    except Exception as e:
        print(f"  [✗] Error stopping QCar {car_id} ({ip}): {e}")
        return False

# --- Main shutdown sequence ---
try:
    stopped_count = 0
    
    for idx, ip in enumerate(QCAR_IPS):
        if stop_processes(ip, idx):
            stopped_count += 1
        time.sleep(1)
    
    # Summary
    print(f"\n{'='*70}")
    print(" Shutdown Complete")
    print(f"{'='*70}")
    print(f"\nStopped {stopped_count}/{len(QCAR_IPS)} QCars successfully")
    
    if stopped_count < len(QCAR_IPS):
        print(f"\n[⚠] Warning: {len(QCAR_IPS) - stopped_count} QCar(s) failed to stop")
        print("    You may need to stop them manually via SSH")
    
    print("="*70)

except KeyboardInterrupt:
    print("\n\n[!] Shutdown interrupted by user")
except Exception as e:
    print(f"\n[✗] Fatal error: {e}")
    import traceback
    traceback.print_exc()

input("\nPress Enter to exit...")
