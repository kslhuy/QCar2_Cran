"""
Simple QUARC-only stop script
Stops only the QUARC models (hardware control) - equivalent to the QUARC part of stop_all_cars.bat
"""
import argparse
import os
import subprocess

# --- Parse command line arguments ---
parser = argparse.ArgumentParser(description="Stop QUARC models only.")
parser.add_argument('-c', '--config', type=str, default='config.txt', 
                    help="Path to config.txt file (default: config.txt)")
args = parser.parse_args()

# --- Read configuration from config.txt ---
def read_config(config_path):
    """Read configuration from config.txt file"""
    config = {}
    
    # If path is relative, look in parent directory
    if not os.path.isabs(config_path):
        parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config_full_path = os.path.join(parent_dir, config_path)
    else:
        config_full_path = config_path
    
    print(f"Reading configuration from: {config_full_path}")
    
    if not os.path.exists(config_full_path):
        # Fallback: try current working directory
        fallback_path = os.path.join(os.getcwd(), 'config.txt')
        if os.path.exists(fallback_path):
            config_full_path = fallback_path
        else:
            raise FileNotFoundError(f"Config file not found at {config_full_path}")
    
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

# --- Stop QUARC models ---
def stop_quarc_models(ip, car_id):
    """Stop QUARC models using quarc_run - same as stop_all_cars.bat"""
    print(f"\n[QCar {car_id}] Stopping QUARC models on {ip}...")
    
    quarc_target = f"tcpip://{ip}:17000"
    cmd = ["quarc_run", "-q", "-Q", "-t", quarc_target, "*.rt-linux_qcar2"]
    
    print(f"  [→] Running: quarc_run -q -Q -t {quarc_target} *.rt-linux_qcar2")
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        
        if result.returncode == 0:
            print(f"  [✓] QUARC models stopped successfully")
            return True
        else:
            print(f"  [i] quarc_run returned code {result.returncode} (models may already be stopped)")
            return True  # Consider it successful - models might already be stopped
            
    except subprocess.TimeoutExpired:
        print(f"  [⚠] quarc_run command timed out")
        return False
    except FileNotFoundError:
        print(f"  [✗] quarc_run command not found - ensure QUARC is installed")
        return False
    except Exception as e:
        print(f"  [✗] Error: {e}")
        return False

# --- Main ---
try:
    config = read_config(args.config)
    QCAR_IPS = config.get('QCAR_IPS', [])
    
    if not QCAR_IPS:
        print("[ERROR] No QCar IPs found in configuration")
        exit(1)
    
    print("="*60)
    print(" QUARC Models Stop (Hardware Control)")
    print("="*60)
    print(f"Stopping QUARC models on {len(QCAR_IPS)} QCars")
    print("="*60)
    
    stopped_count = 0
    for idx, ip in enumerate(QCAR_IPS):
        if stop_quarc_models(ip, idx):
            stopped_count += 1
    
    print(f"\n{'='*60}")
    print(f" QUARC Stop Complete: {stopped_count}/{len(QCAR_IPS)} successful")
    print("="*60)

except Exception as e:
    print(f"[ERROR] {e}")
    input("\nPress Enter to exit...")
    exit(1)

input("\nPress Enter to exit...")