"""
Quick Diagnostic Script - Monitor Vehicle Status
Run this to see real-time status without full logs
"""
import os
import time
import sys

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

def read_latest_log_lines(log_file, num_lines=20):
    """Read last N lines from log file"""
    try:
        with open(log_file, 'r') as f:
            lines = f.readlines()
            return lines[-num_lines:] if len(lines) > num_lines else lines
    except:
        return []

def extract_state_from_log(lines):
    """Extract current state from log lines"""
    for line in reversed(lines):
        if 'State transition:' in line or 'state:' in line.lower():
            return line.strip()
    return "Unknown"

def extract_performance_from_log(lines):
    """Extract performance metrics"""
    for line in reversed(lines):
        if 'Performance:' in line:
            return line.strip()
    return "No performance data"

def extract_connection_from_log(lines):
    """Extract connection status"""
    for line in reversed(lines):
        if 'connected' in line.lower() or 'connection' in line.lower():
            return line.strip()
    return "No connection info"

def main():
    print("Vehicle Diagnostic Monitor")
    print("=" * 70)
    print("Press Ctrl+C to exit")
    print()
    
    # Look for log files
    log_dir = "logs"
    vehicle_log = os.path.join(log_dir, "vehicle_0.log")
    
    if not os.path.exists(vehicle_log):
        print(f"❌ Log file not found: {vehicle_log}")
        print("Make sure the vehicle controller is running!")
        return 1
    
    try:
        while True:
            clear_screen()
            print("=" * 70)
            print("🚗 VEHICLE DIAGNOSTIC MONITOR")
            print("=" * 70)
            print(f"Time: {time.strftime('%H:%M:%S')}")
            print()
            
            # Read recent log lines
            lines = read_latest_log_lines(vehicle_log, 50)
            
            if not lines:
                print("⚠️  No log data available")
                time.sleep(2)
                continue
            
            # Extract key information
            print("📊 RECENT STATUS:")
            print("-" * 70)
            
            # Last few important lines
            important_keywords = ['State', 'WARNING', 'ERROR', 'Performance', 
                                'connected', 'transition', 'position']
            
            recent_important = []
            for line in reversed(lines):
                if any(kw in line for kw in important_keywords):
                    recent_important.append(line.strip())
                    if len(recent_important) >= 10:
                        break
            
            for line in reversed(recent_important):
                # Color code the output
                if 'ERROR' in line:
                    print(f"❌ {line}")
                elif 'WARNING' in line:
                    print(f"⚠️  {line}")
                elif 'State' in line or 'transition' in line:
                    print(f"🔄 {line}")
                elif 'Performance' in line:
                    print(f"📈 {line}")
                elif 'connected' in line.lower():
                    print(f"🔌 {line}")
                else:
                    print(f"ℹ️  {line}")
            
            print()
            print("-" * 70)
            print("Press Ctrl+C to exit | Updating every 2 seconds...")
            
            time.sleep(2)
            
    except KeyboardInterrupt:
        print("\n\n✅ Monitor stopped")
        return 0

if __name__ == '__main__':
    sys.exit(main())
