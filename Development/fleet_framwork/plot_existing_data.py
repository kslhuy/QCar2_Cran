"""
Script to plot existing CSV data using the DataLogger visualization functions.
This script demonstrates how to use the FleetDataVisualizer to plot your existing data.
"""

import os
import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Add the current directory to path to import DataLogger
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from DataLogger import FleetDataVisualizer

# ==== EASY CONFIGURATION ====
# To quickly change the default data directory, modify this path:
# Using relative path - points to the most recent run folder
DEFAULT_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data_logs", "run_20251019_134736")
# Set to None to always show directory selection menu
# Example: DEFAULT_DATA_DIR = None
# Or change the run folder name to another run:
# DEFAULT_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data_logs", "run_20251019_125521")
# ================================

def get_available_data_directories():
    """Get list of available data directories."""
    # Common data directories to check - using relative path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    possible_data_dirs = [
        os.path.join(script_dir, "data_logs"),
    ]
    
    available_dirs = []
    
    for base_dir in possible_data_dirs:
        if os.path.exists(base_dir):
            # Get all subdirectories that contain CSV files
            for item in os.listdir(base_dir):
                item_path = os.path.join(base_dir, item)
                if os.path.isdir(item_path):
                    # Check if it contains CSV files
                    csv_files = [f for f in os.listdir(item_path) if f.endswith('.csv')]
                    if csv_files:
                        available_dirs.append(item_path)
    
    return available_dirs

def select_data_directory():
    """Allow user to select a data directory."""
    print("\n=== Available Data Directories ===")
    
    available_dirs = get_available_data_directories()
    
    if not available_dirs:
        print("No data directories found!")
        
        # Fallback: ask user to enter path manually
        print("\nEnter the full path to your data directory:")
        custom_path = input("Path: ").strip().strip('"').strip("'")
        if custom_path and os.path.exists(custom_path):
            return custom_path
        else:
            print("Invalid path provided.")
            return None
    
    # Show available directories
    for i, dir_path in enumerate(available_dirs, 1):
        dir_name = os.path.basename(dir_path)
        parent_name = os.path.basename(os.path.dirname(dir_path))
        print(f"{i}. {parent_name}/{dir_name}")
        print(f"   Path: {dir_path}")
        
        # Show what files are in this directory
        files = [f for f in os.listdir(dir_path) if f.endswith('.csv')]
        print(f"   Files: {', '.join(files[:3])}" + ("..." if len(files) > 3 else ""))
        print()
    
    print(f"{len(available_dirs) + 1}. Enter custom path")
    
    try:
        choice = input(f"Select directory (1-{len(available_dirs) + 1}): ").strip()
        choice_num = int(choice)
        
        if 1 <= choice_num <= len(available_dirs):
            selected_dir = available_dirs[choice_num - 1]
            print(f"Selected: {selected_dir}")
            return selected_dir
        elif choice_num == len(available_dirs) + 1:
            # Custom path
            print("Enter the full path to your data directory:")
            custom_path = input("Path: ").strip().strip('"').strip("'")
            if custom_path and os.path.exists(custom_path):
                return custom_path
            else:
                print("Invalid path provided.")
                return None
        else:
            print("Invalid selection.")
            return None
            
    except (ValueError, KeyboardInterrupt):
        print("Invalid input or cancelled.")
        return None

def detect_available_vehicles(data_dir):
    """Detect which vehicle IDs have data in the directory."""
    vehicle_ids = set()
    
    if not os.path.exists(data_dir):
        return []
    
    files = os.listdir(data_dir)
    
    # Check for fleet_states files and local_state files
    for file in files:
        if file.startswith("fleet_states_vehicle_") and file.endswith(".csv"):
            # Extract vehicle ID from filename
            try:
                vehicle_id = int(file.split("_")[-1].split(".")[0])
                vehicle_ids.add(vehicle_id)
            except ValueError:
                continue
                
        elif file.startswith("local_state_vehicle_") and file.endswith(".csv"):
            # Extract vehicle ID from filename
            try:
                vehicle_id = int(file.split("_")[-1].split(".")[0])
                vehicle_ids.add(vehicle_id)
            except ValueError:
                continue
    
    return sorted(list(vehicle_ids))

def get_user_preferences():
    """Get user preferences for plotting."""
    preferences = {}
    
    # Ask about saving figures
    while True:
        save_choice = input("Save figures to files? (y/n) [default: y]: ").strip().lower()
        if save_choice in ['', 'y', 'yes']:
            preferences['save_fig'] = True
            break
        elif save_choice in ['n', 'no']:
            preferences['save_fig'] = False
            break
        else:
            print("Please enter 'y' for yes or 'n' for no.")
    
    # Ask about showing GPS data
    while True:
        gps_choice = input("Show GPS data in individual trajectories? (y/n) [default: y]: ").strip().lower()
        if gps_choice in ['', 'y', 'yes']:
            preferences['show_gps'] = True
            break
        elif gps_choice in ['n', 'no']:
            preferences['show_gps'] = False
            break
        else:
            print("Please enter 'y' for yes or 'n' for no.")
    
    return preferences

def get_fleet_plotting_preferences():
    """Get user preferences for fleet plotting (trajectories, velocity, heading)."""
    preferences = {}
    
    # Ask about showing velocity
    while True:
        vel_choice = input("Show fleet velocity plots? (y/n) [default: y]: ").strip().lower()
        if vel_choice in ['', 'y', 'yes']:
            preferences['show_velocity'] = True
            break
        elif vel_choice in ['n', 'no']:
            preferences['show_velocity'] = False
            break
        else:
            print("Please enter 'y' for yes or 'n' for no.")
    
    # Ask about showing heading
    while True:
        heading_choice = input("Show fleet heading plots? (y/n) [default: n]: ").strip().lower()
        if heading_choice in ['', 'n', 'no']:
            preferences['show_heading'] = False
            break
        elif heading_choice in ['y', 'yes']:
            preferences['show_heading'] = True
            break
        else:
            print("Please enter 'y' for yes or 'n' for no.")
    
    # Ask about saving
    while True:
        save_choice = input("Save fleet plots to files? (y/n) [default: y]: ").strip().lower()
        if save_choice in ['', 'y', 'yes']:
            preferences['save_fig'] = True
            break
        elif save_choice in ['n', 'no']:
            preferences['save_fig'] = False
            break
        else:
            print("Please enter 'y' for yes or 'n' for no.")
    
    return preferences

def select_vehicles_to_plot(available_vehicles):
    """Allow user to select which vehicles to plot."""
    if not available_vehicles:
        print("No vehicles found in the data.")
        return []
    
    print(f"\nAvailable vehicles: {available_vehicles}")
    print("Options:")
    print("1. Plot all vehicles")
    print("2. Select specific vehicles")
    
    while True:
        choice = input("Enter choice (1 or 2): ").strip()
        
        if choice == "1":
            return available_vehicles
        elif choice == "2":
            selected = []
            print(f"Available vehicles: {available_vehicles}")
            print("Enter vehicle IDs separated by commas (e.g., 0,1,2) or 'all' for all vehicles:")
            
            selection = input("Vehicle IDs: ").strip()
            
            if selection.lower() == 'all':
                return available_vehicles
            
            try:
                vehicle_ids = [int(x.strip()) for x in selection.split(',')]
                selected = [vid for vid in vehicle_ids if vid in available_vehicles]
                
                if selected:
                    print(f"Selected vehicles: {selected}")
                    return selected
                else:
                    print("No valid vehicle IDs selected.")
                    continue
                    
            except ValueError:
                print("Invalid input. Please enter numbers separated by commas.")
                continue
        else:
            print("Please enter '1' or '2'.")

def select_observer_vehicle(available_vehicles):
    """Allow user to select observer vehicle for fleet plots."""
    if not available_vehicles:
        return None
    
    if len(available_vehicles) == 1:
        print(f"Only one vehicle available, using vehicle {available_vehicles[0]} as observer.")
        return available_vehicles[0]
    
    print(f"\nSelect observer vehicle for fleet trajectories:")
    print(f"Available vehicles: {available_vehicles}")
    
    while True:
        try:
            observer_id = int(input("Enter observer vehicle ID: ").strip())
            if observer_id in available_vehicles:
                return observer_id
            else:
                print(f"Vehicle {observer_id} not available. Please select from: {available_vehicles}")
        except ValueError:
            print("Please enter a valid number.")

def plot_your_data(data_dir=None):
    """Plot the existing CSV data files with user-selected options."""
    
    # If no data directory provided, use the default or let user select
    if data_dir is None:
        if DEFAULT_DATA_DIR and os.path.exists(DEFAULT_DATA_DIR):
            data_dir = DEFAULT_DATA_DIR
            print(f"Using default directory: {data_dir}")
        else:
            data_dir = select_data_directory()
            if data_dir is None:
                return
    
    print(f"Loading data from: {data_dir}")
    
    # Check if data directory exists
    if not os.path.exists(data_dir):
        print(f"Error: Data directory not found: {data_dir}")
        return
    
    # List available files
    files = os.listdir(data_dir)
    print(f"Available files: {files}")
    
    # Detect available vehicles
    available_vehicles = detect_available_vehicles(data_dir)
    print(f"Detected vehicles: {available_vehicles}")
    
    if not available_vehicles:
        print("No vehicle data found in directory.")
        return
    
    # Get user preferences
    preferences = get_user_preferences()
    
    # Select vehicles to plot
    selected_vehicles = select_vehicles_to_plot(available_vehicles)
    if not selected_vehicles:
        print("No vehicles selected for plotting.")
        return
    
    # Select observer vehicle for fleet plots
    observer_vehicle = select_observer_vehicle(available_vehicles)
    if observer_vehicle is None:
        print("No observer vehicle selected.")
        return
    
    # Create visualizer
    visualizer = FleetDataVisualizer(data_dir)
    
    try:
        # Plot individual vehicle trajectories
        for vehicle_id in selected_vehicles:
            print(f"\nPlotting Vehicle {vehicle_id} Individual Trajectory...")
            try:
                visualizer.plot_vehicle_trajectory(
                    vehicle_id=vehicle_id, 
                    show_gps=preferences['show_gps'], 
                    save_fig=preferences['save_fig']
                )
            except Exception as e:
                print(f"Error plotting vehicle {vehicle_id} trajectory: {e}")
        
        # Plot fleet trajectories from observer's perspective
        print(f"\nPlotting Fleet States (from Vehicle {observer_vehicle}'s perspective)...")
        print("Getting fleet plotting preferences...")
        fleet_preferences = get_fleet_plotting_preferences()
        
        try:
            # Use the enhanced plotting function with user preferences
            visualizer.plot_fleet_trajectories_and_states(
                observer_vehicle_id=observer_vehicle, 
                save_fig=fleet_preferences['save_fig'],
                show_velocity=fleet_preferences['show_velocity'],
                show_heading=fleet_preferences['show_heading']
            )
        except Exception as e:
            print(f"Error plotting fleet states: {e}")
            # Fallback to basic trajectory plotting
            print("Falling back to basic trajectory plotting...")
            try:
                visualizer.plot_fleet_trajectories(
                    observer_vehicle_id=observer_vehicle, 
                    save_fig=preferences['save_fig']
                )
            except Exception as e2:
                print(f"Error with fallback plotting: {e2}")
        
        # Plot state comparisons for selected vehicles
        for vehicle_id in selected_vehicles:
            print(f"\nPlotting State Comparison for Vehicle {vehicle_id}...")
            try:
                visualizer.plot_state_comparison(
                    vehicle_id=vehicle_id, 
                    save_fig=preferences['save_fig']
                )
            except Exception as e:
                print(f"Error plotting state comparison for vehicle {vehicle_id}: {e}")
        
        print("\nAll requested plots generated successfully!")
        
    except Exception as e:
        print(f"Error during plotting: {e}")
        import traceback
        traceback.print_exc()

def quick_plot_trajectories(data_dir=None):
    """Quick function to plot fleet trajectories with user options."""
    
    if data_dir is None:
        if DEFAULT_DATA_DIR and os.path.exists(DEFAULT_DATA_DIR):
            data_dir = DEFAULT_DATA_DIR
            print(f"Using default directory: {data_dir}")
        else:
            data_dir = select_data_directory()
            if data_dir is None:
                return
    
    # Detect available vehicles
    available_vehicles = detect_available_vehicles(data_dir)
    print(f"Detected vehicles: {available_vehicles}")
    
    if not available_vehicles:
        print("No vehicle data found in directory.")
        return
    
    # Select observer vehicle
    observer_vehicle = select_observer_vehicle(available_vehicles)
    if observer_vehicle is None:
        print("No observer vehicle selected.")
        return
    
    # Ask about saving
    save_choice = input("Save figure to file? (y/n) [default: y]: ").strip().lower()
    save_fig = save_choice in ['', 'y', 'yes']
    
    print(f"Quick plotting fleet trajectories from vehicle {observer_vehicle}'s perspective...")
    visualizer = FleetDataVisualizer(data_dir)
    
    try:
        visualizer.plot_fleet_trajectories(observer_vehicle_id=observer_vehicle, save_fig=save_fig)
        print("Fleet trajectories plotted successfully!")
    except Exception as e:
        print(f"Error plotting fleet trajectories: {e}")
        import traceback
        traceback.print_exc()

def custom_quick_plot(data_dir=None):
    """Custom quick plot function to show basic trajectory data with user options."""
    
    if data_dir is None:
        if DEFAULT_DATA_DIR and os.path.exists(DEFAULT_DATA_DIR):
            data_dir = DEFAULT_DATA_DIR
            print(f"Using default directory: {data_dir}")
        else:
            data_dir = select_data_directory()
            if data_dir is None:
                return
    
    # Detect available vehicles
    available_vehicles = detect_available_vehicles(data_dir)
    print(f"Detected vehicles: {available_vehicles}")
    
    if not available_vehicles:
        print("No vehicle data found in directory.")
        return
    
    # Select observer vehicle (we'll use their fleet data)
    observer_vehicle = select_observer_vehicle(available_vehicles)
    if observer_vehicle is None:
        print("No observer vehicle selected.")
        return
    
    # Ask about saving
    save_choice = input("Save figure to file? (y/n) [default: y]: ").strip().lower()
    save_fig = save_choice in ['', 'y', 'yes']
    
    # Load fleet data from the selected observer
    fleet_csv = os.path.join(data_dir, f"fleet_states_vehicle_{observer_vehicle}.csv")
    local_csv = os.path.join(data_dir, f"local_state_vehicle_{observer_vehicle}.csv")
    
    if os.path.exists(fleet_csv):
        print(f"Loading fleet states data from vehicle {observer_vehicle}...")
        fleet_data = pd.read_csv(fleet_csv)
        print(f"Fleet data shape: {fleet_data.shape}")
        print(f"Fleet data columns: {fleet_data.columns.tolist()}")
        
        # Find all vehicle columns in the data
        vehicle_x_cols = [col for col in fleet_data.columns if col.endswith('_x') and col.startswith('vehicle_')]
        vehicle_y_cols = [col for col in fleet_data.columns if col.endswith('_y') and col.startswith('vehicle_')]
        
        # Extract vehicle IDs from column names
        data_vehicle_ids = []
        for col in vehicle_x_cols:
            try:
                vid = int(col.split('_')[1])
                if f"vehicle_{vid}_y" in vehicle_y_cols:
                    data_vehicle_ids.append(vid)
            except (ValueError, IndexError):
                continue
        
        data_vehicle_ids.sort()
        print(f"Vehicles found in fleet data: {data_vehicle_ids}")
        
        if not data_vehicle_ids:
            print("No vehicle trajectory data found in fleet CSV.")
            return
        
        # Simple trajectory plot
        plt.figure(figsize=(12, 8))
        
        colors = ['b', 'r', 'g', 'c', 'm', 'y', 'k', 'orange', 'purple', 'brown']
        markers = ['o', 's', '^', 'D', 'v', '<', '>', 'p', '*', 'h']
        
        for i, vid in enumerate(data_vehicle_ids):
            x_col = f'vehicle_{vid}_x'
            y_col = f'vehicle_{vid}_y'
            
            if x_col in fleet_data.columns and y_col in fleet_data.columns:
                color = colors[i % len(colors)]
                marker = markers[i % len(markers)]
                
                # Plot trajectory
                plt.plot(fleet_data[x_col], fleet_data[y_col], 
                        color=color, linewidth=2, label=f'Vehicle {vid}', 
                        marker=marker, markersize=2, alpha=0.7)
                
                # Mark start and end points
                plt.plot(fleet_data[x_col].iloc[0], fleet_data[y_col].iloc[0], 
                        color=color, marker='o', markersize=10, alpha=0.8)
                plt.plot(fleet_data[x_col].iloc[-1], fleet_data[y_col].iloc[-1], 
                        color=color, marker='x', markersize=12, alpha=0.8)
        
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.title(f'Fleet Trajectories - Quick Plot (Observer: Vehicle {observer_vehicle})')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        if save_fig:
            # Save plot
            plot_path = os.path.join(data_dir, f"quick_fleet_trajectories_observer_{observer_vehicle}.png")
            plt.savefig(plot_path, dpi=300, bbox_inches='tight')
            print(f"Plot saved to: {plot_path}")
        
        plt.show()
        
    else:
        print(f"Fleet CSV file not found: {fleet_csv}")
        
    if os.path.exists(local_csv):
        print(f"\nLoading local state data from vehicle {observer_vehicle}...")
        local_data = pd.read_csv(local_csv)
        print(f"Local data shape: {local_data.shape}")
        print(f"Local data columns: {local_data.columns.tolist()}")
    else:
        print(f"Local CSV file not found: {local_csv}")

def advanced_plot_menu(data_dir=None):
    """Advanced plotting menu with full control over options."""
    
    if data_dir is None:
        if DEFAULT_DATA_DIR and os.path.exists(DEFAULT_DATA_DIR):
            data_dir = DEFAULT_DATA_DIR
            print(f"Using default directory: {data_dir}")
        else:
            data_dir = select_data_directory()
            if data_dir is None:
                return
    
    # Detect available vehicles
    available_vehicles = detect_available_vehicles(data_dir)
    print(f"Detected vehicles: {available_vehicles}")
    
    if not available_vehicles:
        print("No vehicle data found in directory.")
        return
    
    # Create visualizer
    visualizer = FleetDataVisualizer(data_dir)
    
    while True:
        print(f"\n=== Advanced Plotting Menu ===")
        print(f"Data directory: {os.path.basename(data_dir)}")
        print(f"Available vehicles: {available_vehicles}")
        print("\nPlot Options:")
        print("1. Individual vehicle trajectory")
        print("2. Fleet states (trajectories + velocity/heading)")
        print("3. Fleet trajectories only (classic)")
        print("4. State comparison")
        print("5. Custom quick plot")
        print("6. Change data directory")
        print("7. Exit")
        
        try:
            choice = input("Enter choice (1-7): ").strip()
            
            if choice == "1":
                # Individual vehicle trajectory
                selected_vehicles = select_vehicles_to_plot(available_vehicles)
                if not selected_vehicles:
                    continue
                
                preferences = get_user_preferences()
                
                for vehicle_id in selected_vehicles:
                    print(f"\nPlotting Vehicle {vehicle_id} Individual Trajectory...")
                    try:
                        visualizer.plot_vehicle_trajectory(
                            vehicle_id=vehicle_id,
                            show_gps=preferences['show_gps'],
                            save_fig=preferences['save_fig']
                        )
                    except Exception as e:
                        print(f"Error plotting vehicle {vehicle_id}: {e}")
                        
            elif choice == "2":
                # Enhanced fleet states plotting
                observer_vehicle = select_observer_vehicle(available_vehicles)
                if observer_vehicle is None:
                    continue
                
                print("Getting fleet plotting preferences...")
                fleet_preferences = get_fleet_plotting_preferences()
                
                print(f"\nPlotting Fleet States (Observer: Vehicle {observer_vehicle})...")
                try:
                    visualizer.plot_fleet_trajectories_and_states(
                        observer_vehicle_id=observer_vehicle,
                        save_fig=fleet_preferences['save_fig'],
                        show_velocity=fleet_preferences['show_velocity'],
                        show_heading=fleet_preferences['show_heading']
                    )
                except Exception as e:
                    print(f"Error plotting fleet states: {e}")
                    
            elif choice == "3":
                # Classic fleet trajectories only
                observer_vehicle = select_observer_vehicle(available_vehicles)
                if observer_vehicle is None:
                    continue
                
                save_choice = input("Save figure to file? (y/n) [default: y]: ").strip().lower()
                save_fig = save_choice in ['', 'y', 'yes']
                
                print(f"\nPlotting Fleet Trajectories Only (Observer: Vehicle {observer_vehicle})...")
                try:
                    visualizer.plot_fleet_trajectories(
                        observer_vehicle_id=observer_vehicle,
                        save_fig=save_fig
                    )
                except Exception as e:
                    print(f"Error plotting fleet trajectories: {e}")
                    
            elif choice == "4":
                # State comparison
                selected_vehicles = select_vehicles_to_plot(available_vehicles)
                if not selected_vehicles:
                    continue
                
                save_choice = input("Save figures to files? (y/n) [default: y]: ").strip().lower()
                save_fig = save_choice in ['', 'y', 'yes']
                
                for vehicle_id in selected_vehicles:
                    print(f"\nPlotting State Comparison for Vehicle {vehicle_id}...")
                    try:
                        visualizer.plot_state_comparison(
                            vehicle_id=vehicle_id,
                            save_fig=save_fig
                        )
                    except Exception as e:
                        print(f"Error plotting state comparison for vehicle {vehicle_id}: {e}")
                        
            elif choice == "5":
                # Custom quick plot
                custom_quick_plot(data_dir)
                
            elif choice == "6":
                # Change data directory
                new_data_dir = select_data_directory()
                if new_data_dir:
                    data_dir = new_data_dir
                    available_vehicles = detect_available_vehicles(data_dir)
                    visualizer = FleetDataVisualizer(data_dir)
                    print(f"Changed to directory: {data_dir}")
                    print(f"New available vehicles: {available_vehicles}")
                    
            elif choice == "7":
                break
                
            else:
                print("Invalid choice, please try again.")
                
        except KeyboardInterrupt:
            print("\nOperation cancelled by user.")
            break

def browse_and_plot():
    """Browse available data directories and plot selected one."""
    print("\n=== Browse Data Directories ===")
    
    while True:
        data_dir = select_data_directory()
        if data_dir is None:
            print("No directory selected. Exiting...")
            break
        
        print(f"\nSelected directory: {data_dir}")
        print("\nWhat would you like to do with this data?")
        print("1. Full plotting suite (all plots)")
        print("2. Quick fleet trajectories only") 
        print("3. Custom quick plot")
        print("4. Advanced plotting menu")
        print("5. Select different directory")
        print("6. Exit")
        
        try:
            choice = input("Enter choice (1-6): ").strip()
            
            if choice == "1":
                plot_your_data(data_dir)
                break
            elif choice == "2":
                quick_plot_trajectories(data_dir)
                break
            elif choice == "3":
                custom_quick_plot(data_dir)
                break
            elif choice == "4":
                advanced_plot_menu(data_dir)
                break
            elif choice == "5":
                continue  # Go back to directory selection
            elif choice == "6":
                break
            else:
                print("Invalid choice, please try again.")
                
        except KeyboardInterrupt:
            print("\nOperation cancelled by user.")
            break

if __name__ == "__main__":
    print("=== Plotting Existing Data ===")
    print("Choose an option:")
    print("1. Full plotting suite (all plots with user options)")
    print("2. Quick fleet trajectories only")
    print("3. Custom quick plot")
    print("4. Browse and select data directory")
    print("5. Advanced plotting menu")
    
    try:
        choice = input("Enter choice (1-5): ").strip()
        
        if choice == "1":
            plot_your_data()
        elif choice == "2":
            quick_plot_trajectories()
        elif choice == "3":
            custom_quick_plot()
        elif choice == "4":
            browse_and_plot()
        elif choice == "5":
            advanced_plot_menu()
        else:
            print("Invalid choice, running advanced menu...")
            advanced_plot_menu()
            
    except KeyboardInterrupt:
        print("\nOperation cancelled by user.")
    except Exception as e:
        print(f"Error: {e}")
        print("Running advanced menu as fallback...")
        advanced_plot_menu()
