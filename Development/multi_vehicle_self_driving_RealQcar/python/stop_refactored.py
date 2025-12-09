"""
Stop script for multi-vehicle refactored control system
Cleanly stops all vehicle control and YOLO processes on QCars
"""
import argparse
import os
import time
import subprocess
import socket

# --- Parse command line arguments ---
parser = argparse.ArgumentParser(description="Stop all QCar processes.")
parser.add_argument('-c', '--config', type=str, default='config.txt', 
                    help="Path to config.txt file (default: config.txt)")
args = parser.parse_args()

# --- Read configuration from config.txt ---
def read_config(config_path):
    """Read configuration from config.txt file"""
    config = {}
    
    # If path is relative, look in parent directory (where config.txt actually is)
    if not os.path.isabs(config_path):
        # Go up one level from python/ directory to find config.txt
        parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config_full_path = os.path.join(parent_dir, config_path)
    else:
        config_full_path = config_path
    
    print(f"Reading configuration from: {config_full_path}")
    
    if not os.path.exists(config_full_path):
        print(f"[ERROR] Config file not found: {config_full_path}")
        print(f"[INFO] Looking for config.txt in current directory...")
        # Fallback: try current working directory
        fallback_path = os.path.join(os.getcwd(), 'config.txt')
        if os.path.exists(fallback_path):
            print(f"[INFO] Found config.txt in: {fallback_path}")
            config_full_path = fallback_path
        else:
            raise FileNotFoundError(f"Config file not found at {config_full_path} or {fallback_path}")
    
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
                        # Handle both comma and space separated values
                        if ',' in value:
                            config[key] = [ip.strip() for ip in value.split(',')]
                        else:
                            config[key] = [ip.strip() for ip in value.split()]
                    else:
                        config[key] = value
    
    return config

# Read configuration
try:
    config = read_config(args.config)
    QCAR_IPS = config.get('QCAR_IPS', [])
    
    if not QCAR_IPS:
        print("[ERROR] No QCar IPs found in configuration")
        print("[INFO] Please check config.txt file contains QCAR_IPS=...")
        input("\nPress Enter to exit...")
        exit(1)
        
except FileNotFoundError as e:
    print(f"[ERROR] Configuration file error: {e}")
    print("[INFO] Make sure config.txt exists in the main directory")
    input("\nPress Enter to exit...")
    exit(1)
except Exception as e:
    print(f"[ERROR] Error reading configuration: {e}")
    input("\nPress Enter to exit...")
    exit(1)
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
    import paramiko
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        ssh.connect(ip, username=USERNAME, password=PASSWORD, timeout=15)
        return ssh
    except Exception as e:
        print(f"  [✗] SSH connection failed: {e}")
        raise

# --- Stop QUARC models via Windows quarc_run (same as stop_all_cars.bat) ---
def stop_quarc_models(ip, car_id):
    """Stop QUARC models using Windows quarc_run command - exactly like stop_all_cars.bat"""
    try:
        print(f"  [→] Stopping QUARC models (hardware control)...")
        
        # Use quarc_run to stop all QUARC models - same command as stop_all_cars.bat
        quarc_target = f"tcpip://{ip}:17000"
        cmd = ["quarc_run", "-q", "-Q", "-t", quarc_target, "*.rt-linux_qcar2"]
        
        print(f"  [→] Running: quarc_run -q -Q -t {quarc_target} *.rt-linux_qcar2")
        
        try:
            # Run quarc_run command with timeout
            result = subprocess.run(cmd, 
                                  capture_output=True, 
                                  text=True, 
                                  timeout=15)
            
            if result.returncode == 0:
                print(f"  [✓] QUARC models stopped successfully")
                return True
            else:
                print(f"  [⚠] quarc_run returned code {result.returncode}")
                if result.stderr:
                    print(f"      stderr: {result.stderr.strip()}")
                if result.stdout:
                    print(f"      stdout: {result.stdout.strip()}")
                # Don't treat non-zero as complete failure - models might already be stopped
                return True  # Consider it successful (like the batch script does)
                
        except subprocess.TimeoutExpired:
            print(f"  [⚠] quarc_run command timed out (15s)")
            return False
        except FileNotFoundError:
            print(f"  [✗] quarc_run command not found")
            print(f"      Please ensure QUARC is installed and in PATH")
            return False
        except Exception as e:
            print(f"  [⚠] Error running quarc_run: {e}")
            return False
            
    except Exception as e:
        print(f"  [✗] Error stopping QUARC models: {e}")
        return False


# --- Stop processes on QCar ---
def stop_processes(ip, car_id):
    """Stop all vehicle control and YOLO processes on QCar"""
    try:
        print(f"\n[QCar {car_id}] Stopping processes on {ip}...")
        
        # First stop Python processes via SSH
        try:
            import paramiko
            ssh = create_ssh(ip)
            
            # Check what processes are running
            print(f"  [→] Stopping Python processes...")
            stdin, stdout, stderr = ssh.exec_command("pgrep -f 'vehicle_main|yolo_server'")
            running_pids = stdout.read().decode().strip()
            if running_pids:
                print(f"  [i] Found processes: PIDs {running_pids.replace(chr(10), ', ')}")
            else:
                print(f"  [i] No relevant processes found")
            
            # Kill vehicle control and YOLO processes with SIGTERM first
            print(f"  [→] Sending SIGTERM to vehicle_main and yolo_server processes...")
            ssh.exec_command("pkill -15 -f 'vehicle_main'")
            ssh.exec_command("pkill -15 -f 'yolo_server'")
            time.sleep(2)
            
            # Check if processes are still running
            stdin, stdout, stderr = ssh.exec_command("pgrep -f 'vehicle_main|yolo_server'")
            remaining = stdout.read().decode().strip()
            
            if remaining:
                print(f"  [→] Force killing remaining processes with SIGKILL...")
                ssh.exec_command("pkill -9 -f 'vehicle_main'")
                ssh.exec_command("pkill -9 -f 'yolo_server'")
                time.sleep(1)
                
                # Final verification
                stdin, stdout, stderr = ssh.exec_command("pgrep -f 'vehicle_main|yolo_server'")
                still_remaining = stdout.read().decode().strip()
                if still_remaining:
                    print(f"  [⚠] Warning: Some processes still running: PIDs {still_remaining.replace(chr(10), ', ')}")
                else:
                    print(f"  [✓] All processes stopped (forced)")
            else:
                print(f"  [✓] All processes stopped gracefully")
            
            ssh.close()
            print(f"  [✓] Python processes stopped")
            python_stopped = True
            
        except Exception as e:
            print(f"  [⚠] Error stopping Python processes via SSH: {e}")
            print(f"      (SSH may not be available or configured)")
            python_stopped = False
        
        # # Now stop QUARC models (hardware control) using Windows quarc_run
        # quarc_stopped = stop_quarc_models(ip, car_id)
        
        # if python_stopped and quarc_stopped:
        #     print(f"  [✓] QCar {car_id} ({ip}) fully stopped (software + hardware)")
        # elif python_stopped:
        #     print(f"  [⚠] QCar {car_id} ({ip}) software stopped, QUARC status unclear")
        # elif quarc_stopped:
        #     print(f"  [⚠] QCar {car_id} ({ip}) hardware stopped, Python status unclear")
        # else:
        #     print(f"  [⚠] QCar {car_id} ({ip}) stopping had issues")
        #     print(f"      You may need to check manually")
        
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
