#!/usr/bin/env python3
"""
Quick remote YOLO camera test
This script uploads the test script to QCar and runs it
"""
import paramiko
import argparse
from scp import SCPClient
import os

parser = argparse.ArgumentParser(description="Test YOLO camera on QCar")
parser.add_argument('-ip', '--qcar_ip', type=str, required=True, help="IP address of QCar")
parser.add_argument('-rp','--remote_path', type=str, required=True, help="Remote path on QCar")
args = parser.parse_args()

QCAR_IP = args.qcar_ip
REMOTE_PATH = args.remote_path
USERNAME = "nvidia"
PASSWORD = "nvidia"
LOCAL_TEST_SCRIPT = "python/test_camera.py"

print(f"Connecting to QCar at {QCAR_IP}...")

try:
    # Create SSH connection
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(QCAR_IP, username=USERNAME, password=PASSWORD)
    scp = SCPClient(ssh.get_transport())
    
    print(f"✓ Connected successfully")
    
    # Upload test script
    print(f"Uploading test script...")
    scp.put(LOCAL_TEST_SCRIPT, REMOTE_PATH)
    print(f"✓ Test script uploaded")
    
    # Run test script
    print(f"\nRunning camera test on QCar...\n")
    print("=" * 60)
    
    cmd = f"cd {REMOTE_PATH} && python test_camera.py"
    stdin, stdout, stderr = ssh.exec_command(cmd)
    
    # Print output in real-time
    for line in stdout:
        print(line, end='')
    
    # Print any errors
    error_output = stderr.read().decode()
    if error_output:
        print("\nErrors:")
        print(error_output)
    
    print("=" * 60)
    
    ssh.close()
    scp.close()
    
    print("\nTest complete!")

except Exception as e:
    print(f"✗ Error: {e}")
    print("\nMake sure:")
    print("1. QCar is powered on and connected to network")
    print(f"2. QCar IP address {QCAR_IP} is correct")
    print("3. You can ping the QCar from your computer")
