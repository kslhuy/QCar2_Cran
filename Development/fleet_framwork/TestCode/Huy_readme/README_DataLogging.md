# Fleet Observer Data Logging and Plotting System

This system provides comprehensive data logging and visualization capabilities for the Fleet Observer system, offering two main approaches for data analysis and plotting:

## Overview

### Option 1: Enhanced Log Files for Python Plotting
- Real-time CSV logging for immediate data access
- JSON format for structured data storage
- Python-based plotting with matplotlib
- Automatic plot generation and saving

### Option 2: MATLAB Data Export
- Export to `.mat` format for MATLAB analysis
- Comprehensive MATLAB plotting functions
- Professional-quality figures
- Advanced analysis capabilities

## Features

### Data Logger (`DataLogger.py`)
- **ObserverDataLogger**: Logs both local state and fleet state data
- **Real-time CSV writing**: Immediate data availability
- **Multiple formats**: CSV, JSON, and MATLAB .mat files
- **FleetDataVisualizer**: Python plotting capabilities

### Enhanced VehicleObserver
- Integrated data logging
- Automatic data export functions
- Built-in plotting methods

### MATLAB Support
- Comprehensive plotting script (`plot_observer_data.m`)
- Multiple plot types and analysis functions
- Professional figure export

## Quick Start

### 1. Basic Integration

```python
from VehicleObserver import VehicleObserver

# Create observer with integrated data logging
observer = VehicleObserver(
    vehicle_id=0, 
    fleet_size=2, 
    config=your_config,
    initial_pose=np.array([0, 0, 0])
)

# Use observer normally - data is logged automatically
observer.update_local_state(measured_state, control_input, timestamp, motor_tach)
observer.update_distributed_estimates(control, local_state, timestamp)

# Export data and generate plots
data_dir = observer.save_data_for_plotting()  # Saves CSV and JSON
matlab_file = observer.export_matlab_data()  # Saves .mat file
observer.plot_trajectories()  # Generates Python plots
```

### 2. Standalone Plotting from Existing Data

```python
from DataLogger import FleetDataVisualizer

# Create visualizer for existing data
visualizer = FleetDataVisualizer("path/to/data/directory")

# Generate plots
visualizer.plot_vehicle_trajectory(vehicle_id=0, show_gps=True)
visualizer.plot_fleet_trajectories(observer_vehicle_id=0)
visualizer.plot_state_comparison(vehicle_id=0)
```

### 3. MATLAB Analysis

```matlab
% Load the MATLAB plotting functions
addpath('path/to/fleet_framework');

% Plot data from .mat file
plot_observer_data('observer_data_vehicle_0_20231201_143052.mat');

% With custom options
options.save_figures = true;
options.show_gps = true;
options.fleet_comparison = true;
options.figure_format = 'pdf';
plot_observer_data('your_data_file.mat', options);
```

## File Structure

```
data_logs/
├── run_YYYYMMDD_HHMMSS/
│   ├── local_state_vehicle_0.csv          # Real-time local state data
│   ├── local_state_vehicle_1.csv
│   ├── fleet_states_vehicle_0.csv         # Real-time fleet estimates
│   ├── fleet_states_vehicle_1.csv
│   ├── observer_data_vehicle_0_*.json     # Structured data
│   ├── observer_data_vehicle_0_*.mat      # MATLAB format
│   ├── vehicle_0_trajectory.png           # Generated plots
│   ├── fleet_trajectories_observer_0.png
│   └── figures_vehicle_0/                 # MATLAB figures
│       ├── vehicle_0_trajectory.png
│       ├── vehicle_0_state_components.png
│       └── ...
```

## Data Formats

### CSV Files (Real-time logging)

#### Local State (`local_state_vehicle_N.csv`)
```csv
timestamp,vehicle_id,x,y,theta,velocity,gps_available,dt,gps_x,gps_y,gps_theta,gps_velocity,control_steering,control_acceleration
1.000,0,1.234,2.345,0.123,1.500,True,0.010,1.230,2.340,0.120,1.495,0.100,0.200
```

#### Fleet States (`fleet_states_vehicle_N.csv`)
```csv
timestamp,observer_vehicle_id,vehicle_0_x,vehicle_0_y,vehicle_0_theta,vehicle_0_velocity,vehicle_1_x,vehicle_1_y,vehicle_1_theta,vehicle_1_velocity
1.000,0,1.234,2.345,0.123,1.500,3.456,4.567,0.234,1.600
```

### MATLAB .mat Files

Contains structured data with the following fields:
- `metadata`: Run information
- `local_states`: Timestamps and state components
- `gps_measurements`: GPS data with NaN for missing measurements
- `control_inputs`: Steering and acceleration commands
- `fleet_estimates`: Complete fleet state history
- `vehicle_N_estimates`: Individual vehicle estimates

## Plot Types

### Python Plots
1. **Vehicle Trajectory**: 2D path with start/end markers and GPS points
2. **Fleet Trajectories**: All vehicles on same plot with different colors
3. **State Comparison**: Time series comparison of estimated vs GPS measurements

### MATLAB Plots
1. **Vehicle Trajectory**: 2D trajectory with GPS overlay
2. **State Components**: Time series of x, y, theta, velocity
3. **Control Inputs**: Steering and acceleration over time
4. **GPS Comparison**: Detailed estimation vs measurement comparison
5. **Fleet Trajectories**: All vehicle paths on single plot
6. **Fleet Evolution**: Time series of all fleet states
7. **Error Analysis**: Statistical analysis of estimation errors

## Usage Examples

### Demo Script
Run the comprehensive demo:
```bash
cd Development/fleet_framework
python demo_plotting.py
```

### Custom Integration
```python
# In your main simulation loop
for timestamp in simulation_timestamps:
    # Update observer (automatically logs data)
    observer.update_local_state(measured_state, control_input, timestamp, motor_tach)
    observer.update_distributed_estimates(control, local_state, timestamp)

# At the end of simulation
data_dir = observer.save_data_for_plotting()
matlab_file = observer.export_matlab_data()
observer.close_data_logger()

# Generate plots
observer.plot_trajectories(show_gps=True, save_fig=True)
```

### Batch Analysis
```python
# Analyze multiple runs
import glob

data_dirs = glob.glob("data_logs/run_*/")
for data_dir in data_dirs:
    visualizer = FleetDataVisualizer(data_dir)
    # Generate all plots for this run
    for vehicle_id in range(fleet_size):
        visualizer.plot_vehicle_trajectory(vehicle_id)
        visualizer.plot_state_comparison(vehicle_id)
```

## Configuration Options

### DataLogger Options
```python
logger = ObserverDataLogger(
    vehicle_id=0,
    fleet_size=2,
    log_dir="custom_logs"  # Custom directory
)
```

### Plotting Options
```python
# Python plotting
visualizer.plot_vehicle_trajectory(
    vehicle_id=0,
    show_gps=True,      # Show GPS measurements
    save_fig=True,      # Save figure to file
    filename="custom.png"  # Custom filename
)
```

```matlab
% MATLAB plotting options
options.save_figures = true;        % Save figures
options.show_gps = true;           % Show GPS data
options.fleet_comparison = true;   % Include fleet plots
options.figure_format = 'png';     % 'png', 'pdf', 'eps'
```

## Dependencies

### Python
- numpy
- matplotlib
- pandas
- scipy
- threading
- json
- csv

### MATLAB
- Base MATLAB installation
- No additional toolboxes required

## Installation

1. Copy the new files to your fleet_framework directory:
   - `DataLogger.py`
   - `plot_observer_data.m`
   - `demo_plotting.py`

2. Update your VehicleObserver import:
   ```python
   from DataLogger import ObserverDataLogger
   ```

3. Install Python dependencies:
   ```bash
   pip install matplotlib pandas scipy
   ```

## Performance Considerations

- CSV files are written in real-time with automatic flushing
- Memory usage scales with simulation length
- Large datasets may require chunked processing
- MATLAB .mat files are memory-efficient for large datasets

## Troubleshooting

### Common Issues

1. **Import errors**: Ensure DataLogger.py is in the same directory as VehicleObserver.py
2. **Missing plots**: Check that matplotlib is installed and data files exist
3. **MATLAB errors**: Verify the .mat file path and MATLAB data structure
4. **Performance**: For very long simulations, consider periodic data saving

### Debug Tips

1. Check data file existence and size
2. Verify timestamp consistency
3. Ensure proper data format (no NaN where unexpected)
4. Check file permissions for writing

## Advanced Usage

### Custom Data Processing
```python
# Access raw data from logger
local_data = observer.data_logger.local_state_data
fleet_data = observer.data_logger.fleet_state_data

# Custom analysis
import pandas as pd
df = pd.read_csv("local_state_vehicle_0.csv")
# Your custom analysis here
```

### Batch MATLAB Processing
```matlab
% Process multiple files
data_files = dir('data_logs/**/observer_data_vehicle_*.mat');
for i = 1:length(data_files)
    file_path = fullfile(data_files(i).folder, data_files(i).name);
    plot_observer_data(file_path);
end
```

This system provides a comprehensive solution for both real-time monitoring and post-simulation analysis of your fleet observer data.
