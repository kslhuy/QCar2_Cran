"""
Enhanced Start Script for Multi-Vehicle Refactored Control System
Uses YAML configuration for flexible fleet management with individual vehicle settings
"""
import os
import sys
import time
import yaml
import argparse
import paramiko
from scp import SCPClient
import glob
import subprocess
import traceback


class FleetStarter:
    """Manages startup of multiple QCar vehicles with individual configurations"""
    
    def __init__(self, config_path: str, args):
        self.args = args
        self.config_path = os.path.normpath(os.path.join(os.path.dirname(__file__), config_path))
        self.config = self._load_config()
        self.probing_process = None
        self.current_dir = os.path.dirname(os.path.realpath(__file__))
        self.scripts_path = os.path.normpath(os.path.join(self.current_dir, "../qcar"))
        
    def _load_config(self) -> dict:
        """Load and validate YAML configuration"""
        print(f"Loading configuration from: {self.config_path}")
        
        try:
            with open(self.config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            # Validate required sections
            required_sections = ['ground_station', 'remote', 'vehicles']
            for section in required_sections:
                if section not in config:
                    raise ValueError(f"Missing required section: {section}")
            
            # Filter enabled vehicles
            config['vehicles'] = [v for v in config['vehicles'] if v.get('enabled', True)]
            
            if not config['vehicles']:
                raise ValueError("No enabled vehicles found in configuration")
            
            return config
            
        except Exception as e:
            print(f"[ERROR] Failed to load configuration: {e}")
            sys.exit(1)
    
    def display_configuration(self):
        """Display fleet configuration summary"""
        gs = self.config['ground_station']
        remote = self.config['remote']
        image_size = self.config.get('image_size', {})
        upload = self.config.get('upload', {})
        
        print("="*70)
        print(" Enhanced Multi-Vehicle Control System - Startup")
        print("="*70)
        print(f"\nGround Station:")
        print(f"  Host IP: {gs['local_ip']}")
        print(f"  Base Port: {gs['base_port']}")
        
        print(f"\nRemote Configuration:")
        print(f"  Username: {remote['username']}")
        print(f"  Remote Path: {remote['remote_path']}")
        
        print(f"\nObserver Settings:")
        print(f"  Resolution: {image_size.get('width', 320)}x{image_size.get('height', 200)}")
        
        print(f"\nUpload Settings:")
        print(f"  Skip Upload: {upload.get('skip_upload', False) or self.args.skip_upload}")
        print(f"  Remote Logging: {upload.get('enable_remote_logs', True) and not self.args.no_logs}")
        
        print(f"\nVehicle Fleet ({len(self.config['vehicles'])} vehicles):")
        print("-"*70)
        
        for vehicle in self.config['vehicles']:
            path_info = self._get_path_info(vehicle['path_number'])
            print(f"  • Car {vehicle['car_id']}: {vehicle['ip']}")
            print(f"    Description: {vehicle.get('description', 'N/A')}")
            print(f"    Path: {vehicle['path_number']} - {path_info['name']}")
            print(f"    Nodes: {path_info['nodes']}")
            print(f"    Probing: {'Yes' if vehicle.get('probing', False) else 'No'}")
            print(f"    Calibration: {'Yes' if vehicle.get('calibrate', False) else 'No'}")
            print(f"    Initial Speed: {vehicle.get('initial_v_ref', 0.75)} m/s")
            print(f"    Port: {gs['base_port'] + vehicle['car_id']}")
            print()
        
        print("="*70)
    
    def _get_path_info(self, path_number: int) -> dict:
        """Get path information from configuration"""
        paths = self.config.get('paths', {})
        if path_number in paths:
            return paths[path_number]
        else:
            return {
                'name': f"Path {path_number}",
                'nodes': [],
                'description': 'Custom path'
            }
    
    def create_ssh_and_scp(self, ip: str):
        """Create SSH and SCP connections to QCar"""
        remote = self.config['remote']
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(ip, username=remote['username'], password=remote['password'], timeout=10)
        scp = SCPClient(ssh.get_transport())
        return ssh, scp
    
    def upload_files(self, ip: str, scp):
        """Upload Python, YAML, text files, and required folders to QCar"""
        remote_path = self.config['remote']['remote_path']
        
        # Upload Python files
        py_files = glob.glob(os.path.join(self.scripts_path, "*.py"))
        for file in py_files:
            scp.put(file, remote_path)
        print(f"    [✓] Uploaded {len(py_files)} Python files")
        
        # Upload YAML configuration files
        yaml_files = (glob.glob(os.path.join(self.scripts_path, "*.yaml")) + 
                      glob.glob(os.path.join(self.scripts_path, "*.yml")))
        for file in yaml_files:
            scp.put(file, remote_path)
        print(f"    [✓] Uploaded {len(yaml_files)} YAML files")
        
        # Upload text files
        txt_files = glob.glob(os.path.join(self.scripts_path, "*.txt"))
        for file in txt_files:
            scp.put(file, remote_path)
        print(f"    [✓] Uploaded {len(txt_files)} text files")
        
        # Upload required folders
        folders = ["StateMachine", "Yolo", "Observer", "V2V", "Controller"]
        for folder_name in folders:
            folder_path = os.path.join(self.scripts_path, folder_name)
            if os.path.exists(folder_path):
                try:
                    scp.put(folder_path, remote_path, recursive=True)
                    folder_files = []
                    for root, dirs, files in os.walk(folder_path):
                        folder_files.extend([os.path.join(root, f) for f in files])
                    print(f"    [✓] Uploaded {folder_name} folder ({len(folder_files)} files)")
                except Exception as e:
                    print(f"    [⚠] Error uploading {folder_name}: {e}")
            else:
                print(f"    [⚠] {folder_name} folder not found")
    
    def start_multi_probing(self):
        """Start multi-probing process locally for probing vehicles"""
        gs = self.config['ground_station']
        image_size = self.config.get('image_size', {})
        probing_vehicles = [v for v in self.config['vehicles'] if v.get('probing', False)]
        
        if not probing_vehicles:
            return
        
        print(f"\n[→] Starting multi-probing.py locally for {len(probing_vehicles)} vehicle(s)...")
        
        probing_script = os.path.join(self.current_dir, "multi_probing.py")
        if os.path.exists(probing_script):
            # Get car IDs of probing vehicles
            probing_car_ids = [str(v['car_id']) for v in probing_vehicles]
            
            # Build command with car IDs and image size
            cmd = [
                "python", probing_script,
                "--cars"
            ] + probing_car_ids + [
                "--width", str(image_size.get('width', 320)),
                "--height", str(image_size.get('height', 200))
            ]
            
            self.probing_process = subprocess.Popen(cmd)
            print(f"[✓] Multi-probing started for Car IDs: {', '.join(probing_car_ids)}")
        else:
            print(f"[⚠] multi_probing.py not found at {probing_script}")
    
    def start_vehicle(self, vehicle: dict):
        """Start a single vehicle with its individual configuration"""
        gs = self.config['ground_station']
        remote = self.config['remote']
        upload = self.config.get('upload', {})
        
        car_id = vehicle['car_id']
        ip = vehicle['ip']
        port = gs['base_port'] + car_id
        
        print(f"\n{'='*70}")
        print(f" Starting QCar {car_id}: {ip} (Port: {port})")
        print(f"{'='*70}")
        
        try:
            # Connect to QCar
            print(f"  [→] Connecting to {ip}...")
            ssh, scp = self.create_ssh_and_scp(ip)
            print(f"  [✓] Connected")
            
            # Upload files
            skip_upload = upload.get('skip_upload', False) or self.args.skip_upload
            if not skip_upload:
                print(f"  [→] Uploading files...")
                self.upload_files(ip, scp)
            else:
                print(f"  [⊘] Skipped file upload")
            
            # Kill existing processes
            print(f"  [→] Stopping any existing processes...")
            # ssh.exec_command(f"pkill -f vehicle_control")
            ssh.exec_command(f"pkill -f yolo_server")
            time.sleep(1)
            
            # Build vehicle_main.py command with individual settings
            print(f"  [→] Starting vehicle_main.py with custom settings...")
            enable_logs = upload.get('enable_remote_logs', True) and not self.args.no_logs
            
            # Build command arguments
            cmd_args = [
                f"--host {gs['local_ip']}",
                f"--port {gs['base_port']}",
                f"--car-id {car_id}",
                f"--path-number {vehicle['path_number']}"
            ]
            
            if vehicle.get('calibrate', False):
                cmd_args.append("--calibrate")
            
            if vehicle.get('left_hand_traffic', False):
                cmd_args.append("--left-hand-traffic")
            
            # Add initial velocity reference
            if 'initial_v_ref' in vehicle:
                cmd_args.append(f"--v-ref {vehicle['initial_v_ref']}")
            
            cmd_vehicle = (
                f"cd {remote['remote_path']} && "
                f"nohup python vehicle_main.py {' '.join(cmd_args)} "
                f"{f'> vehicle_{car_id}.log 2>&1 &' if enable_logs else '> /dev/null 2>&1 &'}"
            )
            ssh.exec_command(cmd_vehicle)
            print(f"  [✓] Vehicle control started")
            print(f"      Path: {vehicle['path_number']} ({self._get_path_info(vehicle['path_number'])['name']})")
            print(f"      Calibration: {vehicle.get('calibrate', False)}")
            print(f"      Initial Speed: {vehicle.get('initial_v_ref', 0.75)} m/s")
            
            time.sleep(2)
            
            # Start YOLO server
            print(f"  [→] Starting yolo_server.py...")
            probing_flag = "True" if vehicle.get('probing', False) else "False"
            
            cmd_yolo = (
                f"cd {remote['remote_path']} && "
                f"nohup python yolo_server.py "
                f"--probing {probing_flag} "
                f"--car-id {car_id} "
                f"{f'> yolo_{car_id}.log 2>&1 &' if enable_logs else '> /dev/null 2>&1 &'}"
            )
            ssh.exec_command(cmd_yolo)
            print(f"  [✓] YOLO server started (Probing: {vehicle.get('probing', False)})")
            
            # Close connections
            ssh.close()
            scp.close()
            
            print(f"  [✓] QCar {car_id} ({ip}) startup complete")
            return True
            
        except Exception as e:
            print(f"  [✗] Error starting QCar {car_id} ({ip}): {e}")
            return False
    
    def start_fleet(self):
        """Start all vehicles in the fleet"""
        self.display_configuration()
        
        # Confirm start
        if not self.args.yes:
            response = input("\nProceed with vehicle startup? (y/n): ")
            if response.lower() != 'y':
                print("Startup cancelled by user")
                return
        
        print("\n" + "="*70)
        print(" Starting Fleet")
        print("="*70)
        

        
        # Start each vehicle
        success_count = 0
        for vehicle in self.config['vehicles']:
            if self.start_vehicle(vehicle):
                success_count += 1
            time.sleep(3)  # Wait between vehicle starts
        
        # Start multi-probing :
        self.start_multi_probing()
        # Display summary
        self.display_summary(success_count)
    
    def display_summary(self, success_count: int):
        """Display startup summary"""
        gs = self.config['ground_station']
        remote = self.config['remote']
        total = len(self.config['vehicles'])
        
        print(f"\n{'='*70}")
        print(" Startup Complete")
        print(f"{'='*70}")
        print(f"\nSuccessfully started {success_count}/{total} vehicles")
        
        print(f"\nVehicle Status:")
        for vehicle in self.config['vehicles']:
            car_id = vehicle['car_id']
            print(f"  • Car {car_id}: {vehicle['ip']} (Port: {gs['base_port'] + car_id})")
            print(f"    Path: {vehicle['path_number']} - {self._get_path_info(vehicle['path_number'])['name']}")
        
        print(f"\nHost PC: {gs['local_ip']}")
        print(f"\nTo view logs on QCar:")
        print(f"  ssh {remote['username']}@<qcar_ip>")
        print(f"  cd {remote['remote_path']}")
        print(f"  tail -f vehicle_0.log")
        print(f"\nTo stop all vehicles:")
        print(f"  python stop_refactored.py --config {self.args.config}")
        print("="*70)
    
    def cleanup(self):
        """Cleanup resources"""
        if self.probing_process:
            print("\nStopping probing process...")
            self.probing_process.terminate()


def main():
    parser = argparse.ArgumentParser(
        description="Enhanced Multi-Vehicle Control System Startup",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument('-c', '--config', type=str, default='../fleet_config.yaml',
                        help="Path to fleet configuration YAML file (default: ../fleet_config.yaml)")
    parser.add_argument('--skip-upload', action='store_true',
                        help="Skip uploading files (useful for quick restart)")
    parser.add_argument('--no-logs', action='store_true',
                        help="Disable log file creation on vehicles")
    parser.add_argument('-y', '--yes', action='store_true',
                        help="Skip confirmation prompt")
    
    args = parser.parse_args()
    
    try:
        starter = FleetStarter(args.config, args)
        starter.start_fleet()
        
        input("\nPress Enter to exit...")
        
    except KeyboardInterrupt:
        print("\n\n[!] Startup interrupted by user")
    except Exception as e:
        print(f"\n[✗] Fatal error: {e}")
        traceback.print_exc()
    finally:
        if 'starter' in locals():
            starter.cleanup()


if __name__ == "__main__":
    main()
