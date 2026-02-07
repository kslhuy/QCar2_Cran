"""
Vehicle Connector Service for QCar Fleet Controller.

This module provides SSH connection, file upload, and remote execution
capabilities for connecting to real QCar vehicles.
"""

import os
import glob
import time
import threading
import subprocess
from typing import Callable, Dict, List, Optional, Tuple, Any
from dataclasses import dataclass

try:
    import paramiko
    from scp import SCPClient
    PARAMIKO_AVAILABLE = True
except ImportError:
    PARAMIKO_AVAILABLE = False
    print("[Warning] paramiko/scp not installed. SSH functionality will be disabled.")
    print("Install with: pip install paramiko scp")


@dataclass
class RemoteConfig:
    """Remote connection configuration."""
    username: str = "nvidia"
    password: str = "nvidia"
    remote_path: str = "/home/nvidia/Documents/multi_vehicle_RealCar"
    timeout: int = 10


@dataclass
class GroundStationConfig:
    """Ground station configuration."""
    local_ip: str = "192.168.2.200"
    base_port: int = 5000


class VehicleConnector:
    """
    Service for connecting to and managing QCar vehicles via SSH.
    
    Provides functionality to:
    - Test SSH connections
    - Upload Python scripts, YAML configs, and folders
    - Start/stop vehicle control programs
    - Monitor connection status
    """
    
    def __init__(self, 
                 remote_config: RemoteConfig = None,
                 ground_station_config: GroundStationConfig = None,
                 scripts_path: str = None,
                 progress_callback: Callable[[str], None] = None,
                 log_callback: Callable[[str, str], None] = None):
        """
        Initialize the vehicle connector.
        
        Args:
            remote_config: SSH connection settings
            ground_station_config: Ground station network settings
            scripts_path: Path to the qcar scripts folder
            progress_callback: Callback for progress updates
            log_callback: Callback for logging (message, level)
        """
        self.remote_config = remote_config or RemoteConfig()
        self.gs_config = ground_station_config or GroundStationConfig()
        
        # Determine scripts path
        if scripts_path:
            self.scripts_path = scripts_path
        else:
            # Default: relative to this file's location
            current_dir = os.path.dirname(os.path.realpath(__file__))
            # Go up from controllers/ to qcar_gui/, then up to GUI/, then into qcar's parent
            self.scripts_path = os.path.normpath(
                os.path.join(current_dir, "..", "..", "..", "..", "qcar")
            )
        
        self.progress_callback = progress_callback
        self.log_callback = log_callback
        
        # Connection tracking
        self._connections: Dict[int, Tuple[paramiko.SSHClient, Any]] = {}
        self._lock = threading.Lock()
    
    def _log(self, message: str, level: str = 'INFO') -> None:
        """Log a message."""
        if self.log_callback:
            self.log_callback(message, level)
        else:
            print(f"[{level}] {message}")
    
    def _progress(self, message: str) -> None:
        """Report progress."""
        if self.progress_callback:
            self.progress_callback(message)
    
    def is_available(self) -> bool:
        """Check if SSH functionality is available."""
        return PARAMIKO_AVAILABLE
    
    def test_connection(self, ip: str) -> Tuple[bool, str]:
        """
        Test SSH connection to a vehicle.
        
        Args:
            ip: IP address of the vehicle
            
        Returns:
            Tuple of (success, message)
        """
        if not PARAMIKO_AVAILABLE:
            return False, "paramiko not installed"
        
        try:
            self._progress(f"Testing connection to {ip}...")
            
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(
                ip,
                username=self.remote_config.username,
                password=self.remote_config.password,
                timeout=self.remote_config.timeout
            )
            
            # Test command execution
            stdin, stdout, stderr = ssh.exec_command("hostname")
            hostname = stdout.read().decode().strip()
            
            ssh.close()
            
            self._log(f"Connection test successful: {ip} (hostname: {hostname})", 'SUCCESS')
            return True, f"Connected to {hostname}"
            
        except paramiko.AuthenticationException:
            msg = f"Authentication failed for {ip}"
            self._log(msg, 'ERROR')
            return False, msg
        except paramiko.SSHException as e:
            msg = f"SSH error: {str(e)}"
            self._log(msg, 'ERROR')
            return False, msg
        except Exception as e:
            msg = f"Connection failed: {str(e)}"
            self._log(msg, 'ERROR')
            return False, msg
    
    def connect(self, car_id: int, ip: str) -> Tuple[bool, str]:
        """
        Establish SSH connection to a vehicle.
        
        Args:
            car_id: Vehicle identifier
            ip: IP address of the vehicle
            
        Returns:
            Tuple of (success, message)
        """
        if not PARAMIKO_AVAILABLE:
            return False, "paramiko not installed"
        
        try:
            self._progress(f"Connecting to {ip}...")
            
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(
                ip,
                username=self.remote_config.username,
                password=self.remote_config.password,
                timeout=self.remote_config.timeout
            )
            
            scp = SCPClient(ssh.get_transport())
            
            with self._lock:
                # Close existing connection if any
                if car_id in self._connections:
                    self._close_connection(car_id)
                self._connections[car_id] = (ssh, scp)
            
            self._log(f"Car {car_id}: Connected to {ip}", 'SUCCESS')
            return True, f"Connected to {ip}"
            
        except Exception as e:
            msg = f"Connection failed: {str(e)}"
            self._log(f"Car {car_id}: {msg}", 'ERROR')
            return False, msg
    
    def disconnect(self, car_id: int) -> None:
        """Disconnect from a vehicle."""
        with self._lock:
            self._close_connection(car_id)
    
    def _close_connection(self, car_id: int) -> None:
        """Close connection for a specific car (internal, no lock)."""
        if car_id in self._connections:
            ssh, scp = self._connections[car_id]
            try:
                scp.close()
                ssh.close()
            except:
                pass
            del self._connections[car_id]
    
    def is_connected(self, car_id: int) -> bool:
        """Check if a vehicle is connected."""
        with self._lock:
            if car_id not in self._connections:
                return False
            
            ssh, _ = self._connections[car_id]
            try:
                transport = ssh.get_transport()
                return transport is not None and transport.is_active()
            except:
                return False
    
    def upload_files(self, car_id: int, ip: str = None) -> Tuple[bool, str]:
        """
        Upload Python scripts, YAML configs, and folders to a vehicle.
        
        Args:
            car_id: Vehicle identifier
            ip: IP address (optional, will reconnect if provided)
            
        Returns:
            Tuple of (success, message)
        """
        if not PARAMIKO_AVAILABLE:
            return False, "paramiko not installed"
        
        # Reconnect if IP provided and not connected
        if ip and not self.is_connected(car_id):
            success, msg = self.connect(car_id, ip)
            if not success:
                return False, msg
        
        with self._lock:
            if car_id not in self._connections:
                return False, "Not connected"
            
            ssh, scp = self._connections[car_id]
        
        try:
            remote_path = self.remote_config.remote_path
            uploaded_count = 0
            
            # Create remote directory if needed
            self._progress("Creating remote directory...")
            ssh.exec_command(f"mkdir -p {remote_path}")
            time.sleep(0.5)
            
            # Upload Python files
            self._progress("Uploading Python files...")
            py_files = glob.glob(os.path.join(self.scripts_path, "*.py"))
            for file in py_files:
                try:
                    scp.put(file, remote_path)
                    uploaded_count += 1
                except Exception as e:
                    self._log(f"Failed to upload {os.path.basename(file)}: {e}", 'WARNING')
            self._log(f"Car {car_id}: Uploaded {len(py_files)} Python files", 'INFO')
            
            # Upload YAML files
            self._progress("Uploading YAML configuration files...")
            yaml_files = (glob.glob(os.path.join(self.scripts_path, "*.yaml")) + 
                         glob.glob(os.path.join(self.scripts_path, "*.yml")))
            for file in yaml_files:
                try:
                    scp.put(file, remote_path)
                    uploaded_count += 1
                except Exception as e:
                    self._log(f"Failed to upload {os.path.basename(file)}: {e}", 'WARNING')
            self._log(f"Car {car_id}: Uploaded {len(yaml_files)} YAML files", 'INFO')
            
            # Upload text files
            self._progress("Uploading text files...")
            txt_files = glob.glob(os.path.join(self.scripts_path, "*.txt"))
            for file in txt_files:
                try:
                    scp.put(file, remote_path)
                    uploaded_count += 1
                except Exception as e:
                    self._log(f"Failed to upload {os.path.basename(file)}: {e}", 'WARNING')
            self._log(f"Car {car_id}: Uploaded {len(txt_files)} text files", 'INFO')
            
            # Upload required folders
            folders = ["StateMachine", "Yolo", "Observer", "V2V", "Controller","simulation"]
            for folder_name in folders:
                folder_path = os.path.join(self.scripts_path, folder_name)
                if os.path.exists(folder_path):
                    self._progress(f"Uploading {folder_name} folder...")
                    try:
                        scp.put(folder_path, remote_path, recursive=True)
                        # Count files in folder
                        folder_files = []
                        for root, dirs, files in os.walk(folder_path):
                            folder_files.extend(files)
                        uploaded_count += len(folder_files)
                        self._log(f"Car {car_id}: Uploaded {folder_name} ({len(folder_files)} files)", 'INFO')
                    except Exception as e:
                        self._log(f"Failed to upload {folder_name}: {e}", 'WARNING')
                else:
                    self._log(f"Car {car_id}: {folder_name} folder not found (skipped)", 'WARNING')
            
            self._log(f"Car {car_id}: Upload complete ({uploaded_count} files total)", 'SUCCESS')
            return True, f"Uploaded {uploaded_count} files"
            
        except Exception as e:
            msg = f"Upload failed: {str(e)}"
            self._log(f"Car {car_id}: {msg}", 'ERROR')
            return False, msg
    
    def start_vehicle(self, car_id: int, ip: str,
                      vehicle_type: str = "Qcar",
                      path_number: int = 0,
                      calibrate: bool = False,
                      left_hand_traffic: bool = False,
                      initial_v_ref: float = 0.6,
                      enable_logs: bool = True) -> Tuple[bool, str]:
        """
        Start the vehicle control program on a QCar.
        
        Args:
            car_id: Vehicle identifier
            ip: IP address of the vehicle
            vehicle_type: "Qcar" or "Limo"
            path_number: Path selection number
            calibrate: Whether to start in calibration mode
            left_hand_traffic: Use left-hand traffic rules
            initial_v_ref: Initial velocity reference
            enable_logs: Create log files on vehicle
            
        Returns:
            Tuple of (success, message)
        """
        if not PARAMIKO_AVAILABLE:
            return False, "paramiko not installed"
        
        # Reconnect if not connected
        if not self.is_connected(car_id):
            success, msg = self.connect(car_id, ip)
            if not success:
                return False, msg
        
        with self._lock:
            if car_id not in self._connections:
                return False, "Not connected"
            
            ssh, _ = self._connections[car_id]
        
        try:
            remote_path = self.remote_config.remote_path
            port = self.gs_config.base_port
            host = self.gs_config.local_ip
            
            # Kill any existing processes
            self._progress("Stopping existing processes...")
            ssh.exec_command("pkill -f vehicle_main")
            ssh.exec_command("pkill -f yolo_server")
            time.sleep(1)
            
            # Build command arguments
            cmd_args = [
                f"--host {host}",
                f"--port {port}",
                f"--car-id {car_id}",
                f"--path-number {path_number}",
                f"--vehicle-type {vehicle_type}",
                f"--v-ref {initial_v_ref}"
            ]
            
            if calibrate:
                cmd_args.append("--calibrate")
            
            if left_hand_traffic:
                cmd_args.append("--left-hand-traffic")
            
            # Build full command
            log_redirect = f"> vehicle_{car_id}.log 2>&1 &" if enable_logs else "> /dev/null 2>&1 &"
            cmd = (
                f"cd {remote_path} && "
                f"nohup python vehicle_main.py {' '.join(cmd_args)} "
                f"{log_redirect}"
            )
            
            self._progress("Starting vehicle control program...")
            ssh.exec_command(cmd)
            
            time.sleep(2)  # Wait for startup
            
            # Verify process started
            stdin, stdout, stderr = ssh.exec_command("pgrep -f vehicle_main")
            pids = stdout.read().decode().strip()
            
            if pids:
                self._log(f"Car {car_id}: Vehicle program started (PID: {pids.split()[0]})", 'SUCCESS')
                self._log(f"Car {car_id}: Path={path_number}, Calibrate={calibrate}, V_ref={initial_v_ref}", 'INFO')
                return True, f"Vehicle started (PID: {pids.split()[0]})"
            else:
                # Check for errors in log
                if enable_logs:
                    stdin, stdout, stderr = ssh.exec_command(f"tail -5 {remote_path}/vehicle_{car_id}.log")
                    log_tail = stdout.read().decode().strip()
                    if log_tail:
                        self._log(f"Car {car_id} log: {log_tail}", 'WARNING')
                
                return False, "Process did not start (check logs)"
            
        except Exception as e:
            msg = f"Failed to start: {str(e)}"
            self._log(f"Car {car_id}: {msg}", 'ERROR')
            return False, msg
    
    def stop_vehicle(self, car_id: int, ip: str = None, 
                     stop_quarc: bool = True, 
                     stop_hardware: bool = True) -> Tuple[bool, str]:
        """
        Stop the vehicle control program on a QCar (enhanced version).
        
        This performs the same operations as stop_enhanced.bat:
        1. Stop Python processes (vehicle_main, yolo_server)
        2. Stop QUARC models (via local quarc_run command)
        3. Stop hardware (Lidar and QCar DAQ via remote script)
        
        Args:
            car_id: Vehicle identifier
            ip: IP address (optional, will reconnect if needed)
            stop_quarc: Whether to stop QUARC models
            stop_hardware: Whether to stop hardware (Lidar/DAQ)
            
        Returns:
            Tuple of (success, message)
        """
        if not PARAMIKO_AVAILABLE:
            return False, "paramiko not installed"
        
        # Reconnect if IP provided and not connected
        if ip and not self.is_connected(car_id):
            success, msg = self.connect(car_id, ip)
            if not success:
                return False, msg
        
        results = []
        overall_success = True
        
        # Step 1: Stop Python processes
        self._progress("Step 1/3: Stopping Python processes...")
        python_success, python_msg = self._stop_python_processes(car_id)
        results.append(f"Python: {python_msg}")
        if not python_success:
            overall_success = False
        
        # Step 2: Stop QUARC models (runs locally on Ground Station)
        if stop_quarc and ip:
            self._progress("Step 2/3: Stopping QUARC models...")
            quarc_success, quarc_msg = self._stop_quarc_models(ip)
            results.append(f"QUARC: {quarc_msg}")
            # QUARC failure is not critical
        else:
            results.append("QUARC: Skipped")
        
        # Step 3: Stop hardware (runs remotely on QCar)
        if stop_hardware:
            self._progress("Step 3/3: Stopping hardware...")
            hw_success, hw_msg = self._stop_hardware(car_id)
            results.append(f"Hardware: {hw_msg}")
            # Hardware failure is not critical
        else:
            results.append("Hardware: Skipped")
        
        # Compile final message
        final_msg = " | ".join(results)
        self._log(f"Car {car_id}: Stop complete - {final_msg}", 'SUCCESS' if overall_success else 'WARNING')
        
        return overall_success, final_msg
    
    def _stop_python_processes(self, car_id: int) -> Tuple[bool, str]:
        """Stop Python vehicle control and YOLO processes."""
        with self._lock:
            if car_id not in self._connections:
                return False, "Not connected"
            
            ssh, _ = self._connections[car_id]
        
        try:
            # Check what's running
            stdin, stdout, stderr = ssh.exec_command("pgrep -f 'vehicle_main|yolo_server'")
            running_pids = stdout.read().decode().strip()
            
            if not running_pids:
                return True, "No processes running"
            
            self._log(f"Car {car_id}: Found processes (PIDs: {running_pids.replace(chr(10), ', ')})", 'INFO')
            
            # Send SIGTERM first (graceful)
            ssh.exec_command("pkill -15 -f 'vehicle_main'")
            ssh.exec_command("pkill -15 -f 'yolo_server'")
            time.sleep(2)
            
            # Check if still running
            stdin, stdout, stderr = ssh.exec_command("pgrep -f 'vehicle_main|yolo_server'")
            remaining = stdout.read().decode().strip()
            
            if remaining:
                # Force kill with SIGKILL
                self._log(f"Car {car_id}: Force killing remaining processes...", 'WARNING')
                ssh.exec_command("pkill -9 -f 'vehicle_main'")
                ssh.exec_command("pkill -9 -f 'yolo_server'")
                time.sleep(1)
                
                # Final check
                stdin, stdout, stderr = ssh.exec_command("pgrep -f 'vehicle_main|yolo_server'")
                still_running = stdout.read().decode().strip()
                if still_running:
                    return False, f"Some processes still running: {still_running}"
                else:
                    return True, "Stopped (forced)"
            else:
                return True, "Stopped gracefully"
                
        except Exception as e:
            return False, f"Error: {str(e)}"
    
    def _stop_quarc_models(self, ip: str) -> Tuple[bool, str]:
        """
        Stop QUARC models using Windows quarc_run command.
        
        This runs locally on the Ground Station (Windows PC) and connects
        to the QCar via TCP/IP to stop the QUARC real-time model.
        """
        quarc_target = f"tcpip://{ip}:17000"
        cmd = ["quarc_run", "-q", "-Q", "-t", quarc_target, "*.rt-linux_qcar2"]
        
        self._log(f"Running QUARC stop command: {' '.join(cmd)}", 'INFO')
        
        try:
            result = subprocess.run(
                cmd, 
                capture_output=True, 
                text=True, 
                timeout=15,
                shell=True  # Needed on Windows
            )
            
            if result.returncode == 0:
                return True, "Stopped"
            else:
                # Non-zero return might just mean no model was running
                stderr = result.stderr.strip() if result.stderr else ""
                if "not found" in stderr.lower() or result.returncode == 1:
                    return True, "No model running"
                return True, f"Returned {result.returncode}"
                
        except FileNotFoundError:
            self._log("quarc_run not found - ensure QUARC is installed and in PATH", 'WARNING')
            return False, "quarc_run not found"
        except subprocess.TimeoutExpired:
            return False, "Timed out"
        except Exception as e:
            return False, f"Error: {str(e)}"
    
    def _stop_hardware(self, car_id: int) -> Tuple[bool, str]:
        """
        Stop hardware (Lidar and QCar DAQ) by running hardware_stop.py remotely.
        
        This terminates the Lidar and QCar DAQ if they are still in use.
        """
        with self._lock:
            if car_id not in self._connections:
                return False, "Not connected"
            
            ssh, _ = self._connections[car_id]
        
        try:
            remote_path = self.remote_config.remote_path
            
            # Create a simple hardware stop script inline and execute it
            # This is more reliable than depending on hardware_stop.py existing
            hw_stop_script = """
python3 -c "
try:
    from pal.products.qcar import QCar, QCarLidar
    myLidar = QCarLidar()
    myCar = QCar()
    myCar.terminate()
    myLidar.terminate()
    print('Hardware stopped successfully')
except Exception as e:
    print(f'Hardware stop error: {e}')
"
"""
            
            stdin, stdout, stderr = ssh.exec_command(hw_stop_script, timeout=10)
            output = stdout.read().decode().strip()
            error = stderr.read().decode().strip()
            
            if "successfully" in output.lower():
                return True, "Stopped"
            elif error:
                # Hardware might not be running, which is fine
                if "not initialized" in error.lower() or "not found" in error.lower():
                    return True, "Not running"
                return False, error[:50]
            else:
                return True, output[:50] if output else "Done"
                
        except Exception as e:
            return False, f"Error: {str(e)}"
    
    def stop_vehicle_quick(self, car_id: int, ip: str = None) -> Tuple[bool, str]:
        """
        Quick stop - only stops Python processes (no QUARC or hardware).
        
        Use this for faster stopping when you don't need full shutdown.
        """
        return self.stop_vehicle(car_id, ip, stop_quarc=False, stop_hardware=False)
    
    def stop_all_vehicles(self, vehicle_ips: Dict[int, str]) -> Dict[int, Tuple[bool, str]]:
        """
        Stop all vehicle control programs.
        
        Args:
            vehicle_ips: Dictionary of car_id -> ip
            
        Returns:
            Dictionary of car_id -> (success, message)
        """
        results = {}
        for car_id, ip in vehicle_ips.items():
            results[car_id] = self.stop_vehicle(car_id, ip)
        return results
    
    def cleanup(self) -> None:
        """Close all connections."""
        with self._lock:
            for car_id in list(self._connections.keys()):
                self._close_connection(car_id)
        self._log("All connections closed", 'INFO')
    
    def calibrate_vehicle(self, car_id: int, ip: str, 
                          distribute_ips: List[str] = None) -> Tuple[bool, str]:
        """
        Run LiDAR calibration on a vehicle (like calibrate.bat).
        
        This performs:
        1. Upload scripts to the calibrating vehicle
        2. Run vehicle_control.py -c True for calibration
        3. Download the generated .mat files
        4. Optionally distribute .mat files to other vehicles
        
        Args:
            car_id: Vehicle identifier
            ip: IP address of the vehicle to calibrate
            distribute_ips: List of IPs to distribute results to (optional)
            
        Returns:
            Tuple of (success, message)
        """
        if not PARAMIKO_AVAILABLE:
            return False, "SSH not available (install paramiko)"
        
        # Reconnect if not connected
        if not self.is_connected(car_id):
            success, msg = self.connect(car_id, ip)
            if not success:
                return False, f"Connection failed: {msg}"
        
        with self._lock:
            if car_id not in self._connections:
                return False, "No connection available"
            ssh, scp = self._connections[car_id]
        
        try:
            # Step 1: Upload scripts
            self._progress("Step 1/4: Uploading scripts...")
            upload_success, upload_msg = self.upload_files(car_id, ip)
            if not upload_success:
                return False, f"Upload failed: {upload_msg}"
            
            # Step 2: Run calibration command
            self._progress("Step 2/4: Running LiDAR calibration (this may take a while)...")
            remote_path = self.remote_config.remote_path
            cmd = f"cd {remote_path} && python vehicle_control.py -c True"
            
            self._log(f"Car {car_id}: Running calibration command: {cmd}", 'INFO')
            stdin, stdout, stderr = ssh.exec_command(cmd, timeout=120)
            output = stdout.read().decode()
            error = stderr.read().decode()
            
            if error and "error" in error.lower():
                return False, f"Calibration error: {error[:100]}"
            
            self._log(f"Car {car_id}: Calibration output: {output[:200]}", 'INFO')
            
            # Step 3: Download .mat files
            self._progress("Step 3/4: Downloading calibration results...")
            local_download_dir = os.path.join(
                os.path.dirname(self.scripts_path), 
                "reference_scan"
            )
            os.makedirs(local_download_dir, exist_ok=True)
            
            mat_files = ["angles_new.mat", "distance_new.mat"]
            downloaded = []
            
            for mat_file in mat_files:
                try:
                    remote_file = f"{remote_path}/{mat_file}"
                    local_file = os.path.join(local_download_dir, mat_file)
                    scp.get(remote_file, local_file)
                    downloaded.append(mat_file)
                    self._log(f"Car {car_id}: Downloaded {mat_file}", 'INFO')
                except Exception as e:
                    self._log(f"Car {car_id}: Failed to download {mat_file}: {e}", 'WARNING')
            
            if not downloaded:
                return False, "No .mat files were generated"
            
            # Step 4: Distribute to other vehicles (if requested)
            if distribute_ips:
                self._progress(f"Step 4/4: Distributing to {len(distribute_ips)} vehicles...")
                dist_results = self._distribute_calibration_files(
                    local_download_dir, 
                    distribute_ips
                )
                self._log(f"Distribution results: {dist_results}", 'INFO')
            else:
                self._progress("Step 4/4: Skipped distribution (no other vehicles)")
            
            return True, f"Calibration complete! Downloaded: {', '.join(downloaded)}"
            
        except Exception as e:
            return False, f"Calibration error: {str(e)}"
    
    def _distribute_calibration_files(self, local_dir: str, 
                                       target_ips: List[str]) -> Dict[str, str]:
        """
        Distribute calibration .mat files to multiple vehicles.
        
        Args:
            local_dir: Directory containing .mat files
            target_ips: List of vehicle IPs to distribute to
            
        Returns:
            Dictionary of ip -> result message
        """
        import glob
        import time
        
        results = {}
        mat_files = glob.glob(os.path.join(local_dir, "*.mat"))
        
        if not mat_files:
            return {"error": "No .mat files found to distribute"}
        
        for ip in target_ips:
            try:
                self._progress(f"Distributing to {ip}...")
                
                # Create new connection for distribution
                ssh = paramiko.SSHClient()
                ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                ssh.connect(
                    ip,
                    username=self.remote_config.username,
                    password=self.remote_config.password,
                    timeout=self.remote_config.timeout
                )
                scp = SCPClient(ssh.get_transport())
                
                remote_path = self.remote_config.remote_path
                
                # Remove old .mat files
                ssh.exec_command(f"rm -f {remote_path}/*.mat")
                time.sleep(0.5)
                
                # Upload new .mat files
                for mat_file in mat_files:
                    scp.put(mat_file, remote_path)
                
                results[ip] = f"✓ Uploaded {len(mat_files)} files"
                self._log(f"Distributed {len(mat_files)} .mat files to {ip}", 'SUCCESS')
                
                scp.close()
                ssh.close()
                
            except Exception as e:
                results[ip] = f"✗ Error: {str(e)}"
                self._log(f"Failed to distribute to {ip}: {e}", 'ERROR')
        
        return results
    
    def set_ground_station_config(self, local_ip: str, base_port: int) -> None:
        """Update ground station configuration."""
        self.gs_config = GroundStationConfig(local_ip=local_ip, base_port=base_port)
    
    def set_remote_config(self, username: str, password: str, remote_path: str) -> None:
        """Update remote connection configuration."""
        self.remote_config = RemoteConfig(
            username=username,
            password=password,
            remote_path=remote_path
        )
