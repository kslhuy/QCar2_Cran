#!/usr/bin/env python3
"""
QCar Fleet Controller - Main Entry Point.

This is the main entry point for the QCar Fleet Controller GUI.
Run this script to start the application.

Usage:
    python app_main.py                    # Default configuration
    python app_main.py --cars 3           # 3 cars
    python app_main.py --port 6000        # Custom base port
    python app_main.py --help             # Show help
"""

import argparse
import sys


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='QCar Fleet Controller GUI',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python app_main.py                   # Start with defaults (5 cars, port 5000)
    python app_main.py --cars 3          # Support 3 vehicles
    python app_main.py --port 6000       # Use port 6000 as base
    python app_main.py --ip 192.168.1.1  # Bind to specific IP
        """
    )
    
    parser.add_argument(
        '--cars', '-c',
        type=int,
        default=5,
        help='Number of cars to support (default: 5)'
    )
    
    parser.add_argument(
        '--port', '-p',
        type=int,
        default=5000,
        help='Base port number (default: 5000)'
    )
    
    parser.add_argument(
        '--ip', '-i',
        type=str,
        default='0.0.0.0',
        help='IP address to listen on (default: 0.0.0.0)'
    )
    
    parser.add_argument(
        '--version', '-v',
        action='version',
        version='QCar Fleet Controller v2.0.0'
    )
    
    return parser.parse_args()


def main():
    """Main entry point."""
    args = parse_args()
    
    try:
        from qcar_gui import create_app
    except ImportError:
        # If running from this directory
        sys.path.insert(0, '.')
        from qcar_gui import create_app
    
    print(f"""
╔══════════════════════════════════════════════════════════╗
║            QCar Fleet Controller v2.0.0                 ║
╠══════════════════════════════════════════════════════════╣
║  Vehicles:    {args.cars:<4}                                     ║
║  Base Port:   {args.port:<5}                                    ║
║  Listen IP:   {args.ip:<15}                          ║
║  Port Range:  {args.port}-{args.port + args.cars - 1:<5}                              ║
╚══════════════════════════════════════════════════════════╝
    """)
    
    # Create and run application
    app = create_app(
        num_cars=args.cars,
        host_ip=args.ip,
        base_port=args.port
    )
    
    # Log startup info
    app.log("=" * 50, 'INFO')
    app.log("QCar Fleet Controller v2.0.0", 'SUCCESS')
    app.log("=" * 50, 'INFO')
    app.log(f"Configuration:", 'INFO')
    app.log(f"  • Vehicles: {args.cars}", 'INFO')
    app.log(f"  • Port Range: {args.port}-{args.port + args.cars - 1}", 'INFO')
    app.log(f"  • Host IP: {args.ip}", 'INFO')
    app.log("=" * 50, 'INFO')
    app.log("Features:", 'INFO')
    app.log("  • Command validation & error handling", 'INFO')
    app.log("  • Platoon control with formation setup", 'INFO')
    app.log("  • V2V communication support", 'INFO')
    app.log("  • Manual control (keyboard/wheel)", 'INFO')
    app.log("  • Real-time telemetry monitoring", 'INFO')
    app.log("=" * 50, 'INFO')
    app.log("Waiting for vehicle connections...", 'INFO')
    
    # Run the main loop
    app.root.mainloop()


if __name__ == '__main__':
    main()
