#!/usr/bin/env python3
"""
Simple GPS Client Test

This script tests the GPS time synchronization by creating a simple client
that connects to the GPS time server and requests time synchronization.

Usage: python test_gps_client.py
"""

import sys
import os
import time

# Add the GPS module path
sys.path.append(os.path.join(os.path.dirname(__file__), 'src', 'GPS_sim'))
from src.GPS_sim.md_gps_sync import GPSSync

try:
    
    from md_logging_config import get_gps_logger
except ImportError as e:
    print(f"Error importing GPS modules: {e}")
    print("Make sure you're running from the fleet_framework directory")
    sys.exit(1)


def test_gps_connection():
    """Test GPS server connection and synchronization."""
    print("🛰️  Testing GPS Connection...")
    print("=" * 50)
    
    # Create GPS sync client
    gps_client = GPSSync(
        gps_server_ip='127.0.0.1',
        gps_server_port=8001,
        vehicle_id=999  # Test vehicle ID
    )
    
    try:
        # Test multiple sync requests
        for i in range(5):
            print(f"\nSync attempt {i+1}/5:")
            
            # Sync with GPS server
            gps_client.sync_with_gps()
            
            # Get synchronized time
            sync_time = gps_client.get_synced_time()
            local_time = time.time()
            
            print(f"  Local time: {local_time:.6f}")
            print(f"  GPS sync time: {sync_time:.6f}")
            print(f"  Time offset: {gps_client.gps_time_offset:.6f} sec")
            
            # Wait between requests
            time.sleep(1.0)
        
        print("\n✅ GPS connection test completed successfully!")
        print(f"Final GPS offset: {gps_client.gps_time_offset:.6f} seconds")
        
    except Exception as e:
        print(f"\n❌ GPS connection test failed: {e}")
        return False
    finally:
        # Clean up resources
        gps_client.cleanup()
    
    return True


def main():
    """Main test function."""
    print("GPS Client Test Tool")
    print("Ensure GPS server is running on 127.0.0.1:8001")
    print()
    
    success = test_gps_connection()
    
    if success:
        print("\n🎉 Test passed! GPS server is responding correctly.")
        return 0
    else:
        print("\n💥 Test failed! Check if GPS server is running.")
        return 1


if __name__ == "__main__":
    sys.exit(main())