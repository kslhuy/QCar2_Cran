#!/usr/bin/env python3
"""
Fleet Logging Control Script

This script provides an easy way to control logging for different modules
in the fleet simulation system for optimal performance.

Usage Examples:
    # Disable all logging for maximum performance
    python logging_control.py --disable-all
    
    # Enable only timing logs for performance monitoring
    python logging_control.py --performance-only
    
    # Enable only essential logs (warnings and errors)
    python logging_control.py --essential-only
    
    # Custom configuration
    python logging_control.py --disable observer timing state
    python logging_control.py --enable communication gps
    
    # Show current status
    python logging_control.py --status
"""

import argparse
import sys
import os

# Add the current directory to Python path to import md_logging_config
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from md_logging_config import (
    fleet_logging, 
    disable_all_logging, 
    enable_all_logging, 
    enable_essential_only,
    enable_performance_monitoring,
    set_module_logging,
    print_logging_status,
    get_logging_status
)

def main():
    parser = argparse.ArgumentParser(
        description='Control Fleet Logging Configuration',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Available Modules:
  fleet         - General fleet operations
  communication - Vehicle communication send/receive
  gps           - GPS synchronization events
  control       - Vehicle control operations
  observer      - Observer state estimation and timing
  vehicle       - Individual vehicle operations
  timing        - Performance timing logs
  state         - State update logs
  debug         - Debug level logs

Examples:
  %(prog)s --disable-all                    # Maximum performance
  %(prog)s --performance-only               # Only timing logs
  %(prog)s --essential-only                 # Only warnings/errors
  %(prog)s --disable observer timing        # Disable specific modules
  %(prog)s --enable communication gps       # Enable specific modules
  %(prog)s --status                         # Show current status
        """
    )
    
    # Preset configurations
    preset_group = parser.add_mutually_exclusive_group()
    preset_group.add_argument('--disable-all', action='store_true',
                             help='Disable all logging for maximum performance')
    preset_group.add_argument('--enable-all', action='store_true', 
                             help='Enable all logging')
    preset_group.add_argument('--essential-only', action='store_true',
                             help='Enable only essential logging (warnings and errors)')
    preset_group.add_argument('--performance-only', action='store_true',
                             help='Enable only timing and performance related logging')
    
    # Individual module controls
    parser.add_argument('--enable', nargs='+', metavar='MODULE',
                       help='Enable logging for specific modules')
    parser.add_argument('--disable', nargs='+', metavar='MODULE',
                       help='Disable logging for specific modules')
    
    # Status and information
    parser.add_argument('--status', action='store_true',
                       help='Show current logging status')
    parser.add_argument('--console', action='store_true',
                       help='Enable console output')
    parser.add_argument('--no-console', action='store_true',
                       help='Disable console output')
    
    args = parser.parse_args()
    
    # Handle preset configurations
    if args.disable_all:
        print("🚀 Disabling all logging for maximum performance...")
        disable_all_logging()
        
    elif args.enable_all:
        print("📝 Enabling all logging...")
        enable_all_logging()
        
    elif args.essential_only:
        print("⚠️  Enabling essential logging only (warnings and errors)...")
        enable_essential_only()
        
    elif args.performance_only:
        print("⏱️  Enabling performance monitoring logging only...")
        enable_performance_monitoring()
    
    # Handle individual module controls
    if args.enable:
        valid_modules = set(fleet_logging.module_logging_enabled.keys())
        for module in args.enable:
            if module in valid_modules:
                print(f"✅ Enabling {module} logging...")
                set_module_logging(module, True)
            else:
                print(f"❌ Unknown module: {module}. Valid modules: {', '.join(valid_modules)}")
    
    if args.disable:
        valid_modules = set(fleet_logging.module_logging_enabled.keys())
        for module in args.disable:
            if module in valid_modules:
                print(f"❌ Disabling {module} logging...")
                set_module_logging(module, False)
            else:
                print(f"❌ Unknown module: {module}. Valid modules: {', '.join(valid_modules)}")
    
    # Console output control
    if args.console:
        print("🖥️  Enabling console output...")
        fleet_logging.enable_console_output(True)
    elif args.no_console:
        print("🔇 Disabling console output...")
        fleet_logging.enable_console_output(False)
    
    # Show status (always show if explicitly requested or if no other action)
    if args.status or (not any([args.disable_all, args.enable_all, args.essential_only, 
                               args.performance_only, args.enable, args.disable, 
                               args.console, args.no_console])):
        print_logging_status()
        
        # Show performance impact estimates
        status = get_logging_status()
        enabled_count = sum(1 for enabled in status.values() if enabled)
        total_count = len(status)
        
        print(f"\n📊 Performance Impact Estimate:")
        if enabled_count == 0:
            print("   🚀 MAXIMUM PERFORMANCE - No logging overhead")
        elif enabled_count <= 2:
            print("   ⚡ HIGH PERFORMANCE - Minimal logging overhead")
        elif enabled_count <= 5:
            print("   🔄 MODERATE PERFORMANCE - Some logging overhead")
        else:
            print("   📝 FULL LOGGING - Maximum logging overhead")
        
        print(f"   Active modules: {enabled_count}/{total_count}")

def create_performance_profiles():
    """Create predefined performance profiles for common use cases."""
    profiles = {
        'development': {
            'description': 'Full logging for development and debugging',
            'config': {module: True for module in fleet_logging.module_logging_enabled.keys()}
        },
        'testing': {
            'description': 'Essential logs plus timing for testing',
            'config': {
                'fleet': False,
                'communication': True,
                'gps': False,
                'control': True,
                'observer': True,
                'vehicle': False,
                'timing': True,
                'state': False,
                'debug': False
            }
        },
        'production': {
            'description': 'Minimal logging for production runs',
            'config': {
                'fleet': False,
                'communication': False,
                'gps': False,
                'control': False,
                'observer': False,
                'vehicle': False,
                'timing': False,
                'state': False,
                'debug': False
            }
        },
        'performance': {
            'description': 'Only timing logs for performance analysis',
            'config': {
                'fleet': False,
                'communication': False,
                'gps': False,
                'control': False,
                'observer': True,
                'vehicle': False,
                'timing': True,
                'state': False,
                'debug': False
            }
        }
    }
    return profiles

if __name__ == "__main__":
    main()
