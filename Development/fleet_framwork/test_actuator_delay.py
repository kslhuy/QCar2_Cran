#!/usr/bin/env python3
"""
Test script to demonstrate actuator delay functionality in VehicleFollowerController.
This shows how the speed command is filtered through a first-order lag with time constant tau.
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os
sys.path.append(os.path.dirname(__file__))

from VehicleFollowerController import VehicleFollowerController

def test_actuator_delay():
    """Test the actuator delay functionality with different tau values."""
    
    print("Testing Actuator Delay Functionality")
    print("=" * 50)
    
    # Create a follower controller
    controller = VehicleFollowerController(
        vehicle_id=1, 
        controller_type="CACC",
        config={'road_type': 'OpenRoad'}
    )
    
    print(f"Default tau value: {controller.tau} seconds")
    
    # Test parameters
    dt = 0.01  # 100 Hz control loop
    time_steps = 500  # 5 seconds of simulation
    
    # Create step input (sudden speed command change)
    time_array = np.arange(0, time_steps * dt, dt)
    raw_speed_commands = np.zeros_like(time_array)
    
    # Step input at t=1 second
    step_start_idx = int(1.0 / dt)
    raw_speed_commands[step_start_idx:] = 0.5  # Step to 0.5 m/s
    
    # Test with different tau values
    tau_values = [0.05, 0.1, 0.2]  # Different time constants
    
    plt.figure(figsize=(12, 8))
    
    for i, tau in enumerate(tau_values):
        controller.tau = tau
        controller.reset_actuator_delay()
        
        delayed_commands = []
        
        for speed_cmd in raw_speed_commands:
            delayed_cmd = controller._apply_actuator_delay(speed_cmd, dt)
            delayed_commands.append(delayed_cmd)
        
        delayed_commands = np.array(delayed_commands)
        
        # Plot results
        plt.subplot(2, 2, i+1)
        plt.plot(time_array, raw_speed_commands, 'b--', label='Raw Command', linewidth=2)
        plt.plot(time_array, delayed_commands, 'r-', label=f'Delayed (τ={tau}s)', linewidth=2)
        plt.xlabel('Time (s)')
        plt.ylabel('Speed Command (m/s)')
        plt.title(f'Actuator Delay with τ = {tau}s')
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.ylim(-0.1, 0.6)
        
        # Calculate settling time (95% of final value)
        final_value = 0.5
        target_value = 0.95 * final_value
        settling_idx = np.where(delayed_commands >= target_value)[0]
        if len(settling_idx) > 0:
            settling_time = time_array[settling_idx[0]] - 1.0  # Subtract step start time
            print(f"τ = {tau}s: Settling time (95%) = {settling_time:.3f}s (≈ {settling_time/tau:.1f}τ)")
    
    # Show the effect on a sinusoidal input
    plt.subplot(2, 2, 4)
    controller.tau = 0.1  # Use default tau
    controller.reset_actuator_delay()
    
    # Sinusoidal input
    freq = 2.0  # Hz
    sin_commands = 0.3 * np.sin(2 * np.pi * freq * time_array)
    delayed_sin_commands = []
    
    for speed_cmd in sin_commands:
        delayed_cmd = controller._apply_actuator_delay(speed_cmd, dt)
        delayed_sin_commands.append(delayed_cmd)
    
    delayed_sin_commands = np.array(delayed_sin_commands)
    
    plt.plot(time_array, sin_commands, 'b--', label='Raw Sine Input', linewidth=2)
    plt.plot(time_array, delayed_sin_commands, 'r-', label=f'Delayed (τ={controller.tau}s)', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Speed Command (m/s)')
    plt.title(f'Sine Wave Response (f={freq}Hz, τ={controller.tau}s)')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    plt.tight_layout()
    plt.suptitle('Actuator Delay Effects on Speed Commands', y=0.98)
    plt.show()
    
    print("\nActuator delay implementation completed successfully!")
    print("The delay follows the equation: delayed_cmd = (1-α)×prev_delayed_cmd + α×cmd")
    print(f"where α = dt/(τ+dt) and τ = {controller.tau}s")

if __name__ == "__main__":
    test_actuator_delay()