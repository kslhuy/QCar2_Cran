"""
Script to run hardware_stop.py on remote QCars via SSH
This ensures proper hardware shutdown (Lidar and QCar DAQ)
"""
import argparse
import os
import time
import subprocess

# --- Parse command line arguments ---
parser = argparse.ArgumentParser(description="Stop QCar hardware on all vehicles.")
parser.add_argument('-c', '--config', type=str, default='config.txt', 
                    help="Path to config.txt file (default: config.txt)")
args = parser.parse_args()

# --- Read configuration from config.txt ---
def read_config(config_path):
    """Read configuration from config.txt file"""
    config = {}
    
    # If path is relative, look in parent directory (where config.txt actually is)
    if not os.path.isabs(config_path):
        parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config_full_path = os.path.join(parent_dir, config_path)
    else:
        config_full_path = config_path
    
    if not os.path.exists(config_full_path):
        fallback_path = os.path.join(os.getcwd(), 'config.txt')
        if os.path.exists(fallback_path):
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
                    
                    # Parse lists
                    if key == 'QCAR_IPS':
                        value = value.strip('[]')
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
    REMOTE_PATH = config.get('REMOTE_PATH', '/home/nvidia/Documents/multi_vehicle_RealCar')
    
    if not QCAR_IPS:
        print("[ERROR] No QCar IPs found in configuration")
        exit(1)
        
except Exception as e:
    print(f"[ERROR] Error reading configuration: {e}")
    exit(1)

USERNAME = "nvidia"
PASSWORD = "nvidia"

print("="*70)
print(" Hardware Stop - Terminating Lidar and QCar DAQ")
print("="*70)

# --- Helper to run SSH command ---
def run_ssh_command(ip, command, description):
    """Run SSH command on remote QCar"""
    import paramiko
    
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        
        print(f"  [{ip}] {description}...", end=' ', flush=True)
        ssh.connect(ip, username=USERNAME, password=PASSWORD, timeout=10)
        
        stdin, stdout, stderr = ssh.exec_command(command, timeout=15)
        exit_status = stdout.channel.recv_exit_status()
        
        if exit_status == 0:
            print("✓")
            return True
        else:
            error_msg = stderr.read().decode().strip()
            print(f"✗ (exit code: {exit_status})")
            if error_msg:
                print(f"      Error: {error_msg}")
            return False
            
    except Exception as e:
        print(f"✗ ({str(e)})")
        return False
    finally:
        try:
            ssh.close()
        except:
            pass

# --- Upload and run hardware_stop.py on each QCar ---
def upload_and_run_hardware_stop(ip, remote_path):
    """Upload hardware_stop.py and run it on remote QCar"""
    import paramiko
    from scp import SCPClient
    
    try:
        # Get local path to hardware_stop.py
        local_file = os.path.join(os.path.dirname(__file__), 'hardware_stop.py')
        
        if not os.path.exists(local_file):
            print(f"  [{ip}] ✗ hardware_stop.py not found locally")
            return False
        
        # Upload file
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(ip, username=USERNAME, password=PASSWORD, timeout=10)
        
        # print(f"  [{ip}] Uploading hardware_stop.py...", end=' ', flush=True)
        # scp = SCPClient(ssh.get_transport())
        # remote_file = f"{remote_path}/hardware_stop.py"
        # scp.put(local_file, remote_file)
        # scp.close()
        # print("✓")
        
        # Run the script
        print(f"  [{ip}] Running hardware stop...", end=' ', flush=True)
        command = f"cd {remote_path} && python3 hardware_stop.py"
        stdin, stdout, stderr = ssh.exec_command(command, timeout=15)
        exit_status = stdout.channel.recv_exit_status()
        
        if exit_status == 0:
            output = stdout.read().decode().strip()
            print("✓")
            if output:
                print(f"      {output}")
            return True
        else:
            error_msg = stderr.read().decode().strip()
            print(f"✗")
            if error_msg:
                print(f"      Error: {error_msg}")
            return False
            
    except Exception as e:
        print(f"✗ ({str(e)})")
        return False
    finally:
        try:
            ssh.close()
        except:
            pass

# --- Stop hardware on all QCars ---
print(f"\nStopping hardware on {len(QCAR_IPS)} QCars...\n")

success_count = 0
for idx, ip in enumerate(QCAR_IPS):
    print(f"QCar {idx} ({ip}):")
    if upload_and_run_hardware_stop(ip, REMOTE_PATH):
        success_count += 1
    print()

print("="*70)
print(f"Hardware stop complete: {success_count}/{len(QCAR_IPS)} successful")
print("="*70)

if success_count < len(QCAR_IPS):
    exit(1)
