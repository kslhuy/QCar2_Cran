# Reference Scan Data

LiDAR calibration and reference scan data for environment mapping.

## Files

- **calibrate.bat** - Batch script to run calibration on QCar hardware
- **angles_new.mat** - MATLAB data file containing LiDAR angle measurements
- **distance_new.mat** - MATLAB data file containing LiDAR distance measurements

## Purpose

This folder contains reference LiDAR scans used for:
- Environment calibration and baseline measurements
- Obstacle detection validation
- SLAM initialization
- Position estimation

The `.mat` files are generated during calibration and used by the localization system.
