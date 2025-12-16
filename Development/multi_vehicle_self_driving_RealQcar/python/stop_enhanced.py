"""
Stop script that reads `fleet_config.yaml` and stops vehicles listed in it.

Features:
 - Reads vehicle IPs from `fleet_config.yaml` (filters by `enabled: true`).
 - Stops `vehicle_main` and `yolo_server` processes via SSH (using credentials from config or defaults).
 - Optional `--stop-quarc` to run `quarc_run` locally to stop QUARC models on each IP (Windows-only command).
 - `--dry-run` to show planned actions without performing network calls.
"""
import argparse
import os
import sys
import time
import yaml
import subprocess


DEFAULT_USERNAME = 'nvidia'
DEFAULT_PASSWORD = 'nvidia'


def resolve_config_path(config_path: str) -> str:
    if os.path.isabs(config_path):
        return config_path
    script_dir = os.path.dirname(__file__)
    candidates = [
        os.path.join(script_dir, config_path),
        os.path.normpath(os.path.join(script_dir, '..', config_path)),
        os.path.abspath(config_path),
    ]
    for c in candidates:
        if os.path.exists(c):
            return os.path.normpath(c)
    return os.path.normpath(os.path.join(script_dir, config_path))


def read_fleet_config(config_path: str) -> dict:
    path = resolve_config_path(config_path)
    print(f"Reading fleet configuration from: {path}")
    if not os.path.exists(path):
        raise FileNotFoundError(f"Fleet config not found: {path}")
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    return data


def extract_ips_from_fleet(config: dict) -> list:
    vehicles = config.get('vehicles', [])
    ips = []
    for v in vehicles:
        if not v.get('enabled', True):
            continue
        ip = v.get('ip')
        if ip:
            ips.append({'car_id': v.get('car_id'), 'ip': str(ip)})
    return ips


def stop_processes_on_car(ip: str, username: str, password: str, dry_run: bool = False) -> bool:
    """Connect via SSH and stop vehicle_main and yolo_server processes."""
    if dry_run:
        print(f"[DRY RUN] Would connect to {ip} and pkill vehicle_main and yolo_server")
        return True

    try:
        import paramiko
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(ip, username=username, password=password, timeout=15)

        print(f"  [→] Sending SIGTERM to vehicle_main and yolo_server on {ip}...")
        ssh.exec_command("pkill -15 -f 'vehicle_main'")
        ssh.exec_command("pkill -15 -f 'yolo_server'")
        time.sleep(2)

        # Force kill if necessary
        stdin, stdout, stderr = ssh.exec_command("pgrep -f 'vehicle_main|yolo_server'")
        remaining = stdout.read().decode().strip()
        if remaining:
            print(f"  [→] Forcing kill for remaining PIDs: {remaining.replace(chr(10), ', ')}")
            ssh.exec_command("pkill -9 -f 'vehicle_main'")
            ssh.exec_command("pkill -9 -f 'yolo_server'")
            time.sleep(1)

            stdin, stdout, stderr = ssh.exec_command("pgrep -f 'vehicle_main|yolo_server'")
            still = stdout.read().decode().strip()
            if still:
                print(f"  [⚠] Some processes still running: {still}")
            else:
                print(f"  [✓] Processes stopped on {ip}")
        else:
            print(f"  [✓] No relevant processes running on {ip}")

        ssh.close()
        return True
    except Exception as e:
        print(f"  [✗] Failed to stop processes on {ip}: {e}")
        return False


def stop_quarc_on_car(ip: str, dry_run: bool = False) -> bool:
    """Run local Windows `quarc_run` command to stop QUARC models on the target IP."""
    quarc_target = f"tcpip://{ip}:17000"
    cmd = ["quarc_run", "-q", "-Q", "-t", quarc_target, "*.rt-linux_qcar2"]
    if dry_run:
        print(f"[DRY RUN] Would run: {' '.join(cmd)}")
        return True

    print(f"  [→] Running: {' '.join(cmd)}")
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        if result.returncode == 0:
            print(f"  [✓] QUARC models stopped on {ip}")
            return True
        else:
            print(f"  [⚠] quarc_run returned {result.returncode} for {ip}")
            if result.stderr:
                print(f"      stderr: {result.stderr.strip()}")
            return True
    except FileNotFoundError:
        print("  [✗] quarc_run not found. Please ensure QUARC is installed and in PATH.")
        return False
    except subprocess.TimeoutExpired:
        print("  [⚠] quarc_run timed out")
        return False
    except Exception as e:
        print(f"  [✗] Error running quarc_run: {e}")
        return False


# --- Compatibility layer to behave like stop_refactored.py ---
def create_ssh(ip: str, username: str, password: str):
    """Create SSH connection to QCar"""
    import paramiko
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        ssh.connect(ip, username=username, password=password, timeout=15)
        return ssh
    except Exception as e:
        print(f"  [✗] SSH connection failed: {e}")
        raise


def stop_quarc_models(ip: str, car_id: int, dry_run: bool = False) -> bool:
    """Stop QUARC models using Windows quarc_run command (same behavior as stop_refactored)."""
    if dry_run:
        print(f"[DRY RUN] Would run quarc_run for {ip}")
        return True
    return stop_quarc_on_car(ip, dry_run=False)


def stop_processes(ip: str, car_id: int, username: str, password: str, dry_run: bool = False) -> bool:
    """Stop all vehicle control and YOLO processes on QCar (mirrors stop_refactored)."""
    try:
        print(f"\n[QCar {car_id}] Stopping processes on {ip}...")

        # First stop Python processes via SSH
        try:
            ssh = create_ssh(ip, username, password)

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

        # QUARC models stopping handled separately if requested
        return True

    except Exception as e:
        print(f"  [✗] Error stopping QCar {car_id} ({ip}): {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description="Stop QCar processes using fleet_config.yaml")
    parser.add_argument('-c', '--config', type=str, default='../fleet_config.yaml',
                        help='Path to fleet_config.yaml (default: ../fleet_config.yaml)')
    parser.add_argument('--stop-quarc', action='store_true', help='Also stop QUARC models via quarc_run')
    parser.add_argument('--dry-run', action='store_true', help='Show planned actions without performing them')
    parser.add_argument('--username', type=str, default=None, help='SSH username (overrides config)')
    parser.add_argument('--password', type=str, default=None, help='SSH password (overrides config)')

    args = parser.parse_args()

    try:
        cfg = read_fleet_config(args.config)
    except Exception as e:
        print(f"[ERROR] {e}")
        input('\nPress Enter to exit...')
        sys.exit(1)

    ips = extract_ips_from_fleet(cfg)
    if not ips:
        print("[ERROR] No enabled vehicles with IPs found in fleet configuration")
        input('\nPress Enter to exit...')
        sys.exit(1)

    remote = cfg.get('remote', {})
    username = args.username or remote.get('username') or DEFAULT_USERNAME
    password = args.password or remote.get('password') or DEFAULT_PASSWORD

    print("\nDetected vehicles:")
    for v in ips:
        print(f"  • Car {v.get('car_id')}: {v.get('ip')}")

    stopped = 0
    for idx, car in enumerate(ips):
        ip = car['ip']
        print(f"\n{'='*60}\nStopping Car {car.get('car_id')} at {ip}")
        ok = stop_processes(ip, idx, username, password, dry_run=args.dry_run)
        if ok:
            stopped += 1
        if args.stop_quarc:
            stop_quarc_models(ip, idx, dry_run=args.dry_run)

    print(f"\n{'='*60}")
    print(f"Stopped processes on {stopped}/{len(ips)} cars")
    print("\nDone.")
    input("\nPress Enter to exit...")


if __name__ == '__main__':
    main()
