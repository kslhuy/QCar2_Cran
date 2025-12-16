"""Calibrate workflow using `fleet_config.yaml`.

This script will:
 - Read `fleet_config.yaml` to find the calibrating vehicle (flagged with `calibrate: true` or specified by `--calibrate-car-id`).
 - Upload required scripts to the calibrating vehicle and run `vehicle_main.py --calibrate` remotely.
 - Download resulting calibration `.mat` files from calibrator.
 - Distribute `.mat` (and optionally updated scripts/configs) to other enabled vehicles.

Supports `--dry-run` to preview actions without making network changes.
"""
import argparse
import os
import sys
import time
import glob
import yaml


DEFAULT_USERNAME = 'nvidia'
DEFAULT_PASSWORD = 'nvidia'
LOCAL_DOWNLOAD_DIR = 'reference_scan'


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


def read_fleet_config(path: str) -> dict:
    p = resolve_config_path(path)
    print(f"Reading fleet configuration from: {p}")
    if not os.path.exists(p):
        raise FileNotFoundError(f"Fleet config not found: {p}")
    with open(p, 'r') as f:
        return yaml.safe_load(f)


def find_calibrator(cfg: dict, calibrate_car_id: int | None):
    vehicles = cfg.get('vehicles', [])
    if calibrate_car_id is not None:
        for v in vehicles:
            if v.get('car_id') == calibrate_car_id:
                return v
        raise ValueError(f"Calibrate car id {calibrate_car_id} not found in fleet config")

    for v in vehicles:
        if v.get('calibrate', False):
            return v

    # fallback: pick first enabled vehicle
    for v in vehicles:
        if v.get('enabled', True):
            print("[INFO] No vehicle flagged with calibrate: true; using first enabled vehicle as calibrator")
            return v

    raise ValueError("No enabled vehicles found in fleet config")


def get_target_ips(cfg: dict, exclude_car_id: int):
    ips = []
    for v in cfg.get('vehicles', []):
        if not v.get('enabled', True):
            continue
        cid = v.get('car_id')
        if cid == exclude_car_id:
            continue
        ip = v.get('ip')
        if ip:
            ips.append({'car_id': cid, 'ip': str(ip)})
    return ips


def create_ssh_and_scp(ip, username, password):
    import paramiko
    from scp import SCPClient
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(ip, username=username, password=password, timeout=15)
    scp = SCPClient(ssh.get_transport())
    return ssh, scp


def upload_to_calibrator(ssh, scp, scripts_path, remote_path, dry_run=False):
    if dry_run:
        print(f"[DRY RUN] Would upload Python/YAML/TXT files from {scripts_path} to {remote_path}")
        return

    py_files = glob.glob(os.path.join(scripts_path, "*.py"))
    for f in py_files:
        scp.put(f, remote_path)
    yaml_files = glob.glob(os.path.join(scripts_path, "*.yaml")) + glob.glob(os.path.join(scripts_path, "*.yml"))
    for f in yaml_files:
        scp.put(f, remote_path)
    txt_files = glob.glob(os.path.join(scripts_path, "*.txt"))
    for f in txt_files:
        scp.put(f, remote_path)


def run_calibration(ssh, remote_path, dry_run=False):
    cmd = f"cd {remote_path} && nohup python vehicle_main.py --calibrate > calibrate.log 2>&1 &"
    if dry_run:
        print(f"[DRY RUN] Would run on remote: {cmd}")
        return True

    print(f"[→] Running calibration on {remote_path}...")
    stdin, stdout, stderr = ssh.exec_command(cmd)
    time.sleep(1)
    return True


def download_results(scp, remote_path, local_dir, dry_run=False):
    if dry_run:
        print(f"[DRY RUN] Would download angles_new.mat and distance_new.mat from {remote_path} to {local_dir}")
        return []
    os.makedirs(local_dir, exist_ok=True)
    files = []
    for name in ['angles_new.mat', 'distance_new.mat']:
        remote_file = os.path.join(remote_path, name)
        try:
            scp.get(remote_file, local_dir)
            files.append(os.path.join(local_dir, name))
        except Exception as e:
            print(f"[⚠] Failed to download {remote_file}: {e}")
    return files


def distribute_files(scp, remote_path, files, dry_run=False):
    if dry_run:
        print(f"[DRY RUN] Would distribute {files} to {remote_path}")
        return
    for f in files:
        scp.put(f, remote_path)


def main():
    parser = argparse.ArgumentParser(description="Calibrate using fleet_config.yaml")
    parser.add_argument('-c', '--config', type=str, default='../fleet_config.yaml', help='Path to fleet_config.yaml')
    parser.add_argument('--calibrate-car-id', type=int, default=None, help='Car ID to use as calibrator (overrides fleet config)')
    parser.add_argument('--dry-run', action='store_true', help='Show actions without network operations')
    parser.add_argument('--distribute-scripts', action='store_true', help='Also distribute updated scripts and configs to all cars')
    args = parser.parse_args()

    try:
        cfg = read_fleet_config(args.config)
    except Exception as e:
        print(f"[ERROR] {e}")
        input('\nPress Enter to exit...')
        sys.exit(1)

    try:
        calibrator = find_calibrator(cfg, args.calibrate_car_id)
    except Exception as e:
        print(f"[ERROR] {e}")
        input('\nPress Enter to exit...')
        sys.exit(1)

    cal_ip = str(calibrator.get('ip'))
    cal_id = calibrator.get('car_id')
    remote_cfg = cfg.get('remote', {})
    username = remote_cfg.get('username') or DEFAULT_USERNAME
    password = remote_cfg.get('password') or DEFAULT_PASSWORD
    remote_path = remote_cfg.get('remote_path')
    scripts_path = os.path.normpath(os.path.join(os.path.dirname(__file__), '../qcar'))

    print(f"Using Car {cal_id} ({cal_ip}) as calibrator")

    if args.dry_run:
        print("Running in DRY RUN mode — no network operations will be performed")

    # Connect to calibrator
    if args.dry_run:
        print(f"[DRY RUN] Would connect to {cal_ip} as {username}")
        scp_files = []
    else:
        try:
            ssh_cal, scp_cal = create_ssh_and_scp(cal_ip, username, password)
        except Exception as e:
            print(f"[ERROR] Cannot connect to calibrator {cal_ip}: {e}")
            input('\nPress Enter to exit...')
            sys.exit(1)

        # Upload scripts and configs
        upload_to_calibrator(ssh_cal, scp_cal, scripts_path, remote_path, dry_run=args.dry_run)

        # Run calibration remotely
        run_calibration(ssh_cal, remote_path, dry_run=args.dry_run)

        # Download results
        scp_files = download_results(scp_cal, remote_path, LOCAL_DOWNLOAD_DIR, dry_run=args.dry_run)

        # Close calibrator connections
        ssh_cal.close()
        scp_cal.close()

    # Distribute files to target cars
    targets = get_target_ips(cfg, exclude_car_id=cal_id)
    print(f"Distributing results to {len(targets)} vehicles")
    for t in targets:
        ip = t['ip']
        print(f"\n[{t.get('car_id')}] -> {ip}")
        if args.dry_run:
            print(f"[DRY RUN] Would connect to {ip} and distribute files: {scp_files}")
            continue

        try:
            ssh, scp = create_ssh_and_scp(ip, username, password)
        except Exception as e:
            print(f"[WARNING] Cannot connect to {ip}: {e}")
            continue

        # Optional: remove old .mat files
        try:
            ssh.exec_command(f"rm -f {remote_path}/*.mat")
        except Exception:
            pass

        if scp_files:
            distribute_files(scp, remote_path, scp_files, dry_run=args.dry_run)
            print(f"  [✓] Distributed {len(scp_files)} files to {ip}")

        if args.distribute_scripts:
            upload_to_calibrator(ssh, scp, scripts_path, remote_path, dry_run=args.dry_run)
            print(f"  [✓] Distributed scripts/configs to {ip}")

        ssh.close()
        scp.close()

    print("\nCalibration workflow complete")
    input('\nPress Enter to exit...')


if __name__ == '__main__':
    main()
