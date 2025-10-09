import paramiko
import argparse
from scp import SCPClient
import os
import glob
import time

# --- Configuration ---
parser = argparse.ArgumentParser(description="Calibrate QCar and distribute results.")
parser.add_argument('-cip', '--calibrate_ip', type=str, required=True, help="IP address of the calibrating QCar")
parser.add_argument('-ip', '--qcar_ips', type=str, required=True, help="IP addresses of QCars to distribute results")
parser.add_argument('-rp','--remote_path',type=str,required=True, help="Remote path on QCar")
args = parser.parse_args()

CALIBRATE_IP = args.calibrate_ip
QCAR_IPS = args.qcar_ips.split(",")  # Comma-separated list of IPs
REMOTE_PATH = args.remote_path
USERNAME = "nvidia"
PASSWORD = "nvidia"
LOCAL_SCRIPTS_PATH = "../qcar"
LOCAL_DOWNLOAD_DIR = "../reference_scan"
CURRENT_DIR = os.path.dirname(os.path.realpath(__file__))
SCRIPTS_PATH = os.path.normpath(os.path.join(CURRENT_DIR,LOCAL_SCRIPTS_PATH))

# --- Helper to create SSH + SCP connections ---
def create_ssh_and_scp(ip):
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(ip, username=USERNAME, password=PASSWORD)
    scp = SCPClient(ssh.get_transport())
    return ssh, scp

# --- Step 1: Upload *.py, *.yaml, and *.txt files to calibrating QCar ---
ssh_cal, scp_cal = create_ssh_and_scp(CALIBRATE_IP)

# Upload Python files
py_files = glob.glob(os.path.join(SCRIPTS_PATH, "*.py"))
for file in py_files:
    scp_cal.put(file, REMOTE_PATH)
print(f"[{CALIBRATE_IP}] Uploaded {len(py_files)} Python scripts.")

# Upload YAML files
yaml_files = glob.glob(os.path.join(SCRIPTS_PATH, "*.yaml")) + glob.glob(os.path.join(SCRIPTS_PATH, "*.yml"))
for file in yaml_files:
    scp_cal.put(file, REMOTE_PATH)
print(f"[{CALIBRATE_IP}] Uploaded {len(yaml_files)} YAML configuration files.")

# Upload text files
txt_files = glob.glob(os.path.join(SCRIPTS_PATH, "*.txt"))
for file in txt_files:
    scp_cal.put(file, REMOTE_PATH)
print(f"[{CALIBRATE_IP}] Uploaded {len(txt_files)} text files.")

# --- Step 2: Run calibration command remotely ---
cmd = f"cd {REMOTE_PATH} && python vehicle_control.py -c True"
stdin, stdout, stderr = ssh_cal.exec_command(cmd)
print(f"[{CALIBRATE_IP}] Running vehicle_control.py in calibration mode...")
print(stdout.read().decode())
print(stderr.read().decode())

# --- Step 3: Download angles_new.mat and distance_new.mat files back to local machine ---
scp_cal.get(f"{REMOTE_PATH}/angles_new.mat", LOCAL_DOWNLOAD_DIR)
scp_cal.get(f"{REMOTE_PATH}/distance_new.mat", LOCAL_DOWNLOAD_DIR)
print(f"[{CALIBRATE_IP}] Downloaded .mat files.")
ssh_cal.close()
scp_cal.close()

# --- Step 4: Distribute .mat, .py, .yaml, and .txt files to all QCars ---
mat_files = glob.glob(os.path.join(LOCAL_DOWNLOAD_DIR, "*.mat"))
py_files = glob.glob(os.path.join(SCRIPTS_PATH, "*.py"))
yaml_files = glob.glob(os.path.join(SCRIPTS_PATH, "*.yaml")) + glob.glob(os.path.join(SCRIPTS_PATH, "*.yml"))
txt_files = glob.glob(os.path.join(SCRIPTS_PATH, "*.txt"))

for ip in QCAR_IPS:
    print(f"\n[{ip}] Distributing files...")
    ssh, scp = create_ssh_and_scp(ip)
    
    # Remove old .mat files
    print(f"[{ip}] Removing old .mat files...")
    ssh.exec_command(f"rm -f {REMOTE_PATH}/*.mat")
    time.sleep(1)

    # Upload .mat files
    for mat in mat_files:
        scp.put(mat, REMOTE_PATH)
        print(f"[{ip}] Uploaded {os.path.basename(mat)}")
    
    # Upload Python files
    for file in py_files:
        scp.put(file, REMOTE_PATH)
    print(f"[{ip}] Uploaded {len(py_files)} Python scripts.")
    
    # Upload YAML files
    for file in yaml_files:
        scp.put(file, REMOTE_PATH)
    print(f"[{ip}] Uploaded {len(yaml_files)} YAML configuration files.")
    
    # Upload text files
    for file in txt_files:
        scp.put(file, REMOTE_PATH)
    print(f"[{ip}] Uploaded {len(txt_files)} text files.")

    ssh.close()
    scp.close()
    time.sleep(3)  # Simulate `timeout /t 3`

input("Calibration complete. Press Enter to exit...")  # Wait for user input before exiting
