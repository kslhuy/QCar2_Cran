"""
Fake Vehicle using REAL VehicleLogic Class - MODULAR REFACTOR
This entry point uses the qcar.simulation package for vehicle dynamics and disturbances.
"""
import sys
import os
import time
import socket
import json
import threading
import math
import numpy as np
from threading import Event
from typing import Dict, Any, Optional

# Add parent directory to path to import qcar modules if needed
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

# Import the REAL VehicleLogic and related classes
from vehicle_logic import VehicleLogic
from config_main import VehicleMainConfig
from StateMachine.vehicle_state_machine import VehicleStateMachine
from StateMachine.vehicle_state import VehicleState
from fake_initializing_state import FakeInitializingState

# Import modular simulation components
from simulation.mock_vehicle import MockQCar
from simulation.config import SimulationConfig

class FakeVehicleWithRealLogic:
    """Fake vehicle that uses the real VehicleLogic class with modular MockQCar"""
    
    def __init__(self, car_id: int, host_ip: str, base_port: int, 
                 dynamic_model_type: Optional[int] = None, 
                 vehicle_params: Optional[str] = None, 
                 tire_model: Optional[str] = None):
        self.car_id = car_id
        self.host_ip = host_ip
        self.base_port = base_port
        
        # 1. Load and Configure MockQCar
        self.sim_config = SimulationConfig.get_default_config()
        try:
            loaded = SimulationConfig.load_config('parameters.yaml')
            if loaded:
                self.sim_config.update(loaded)
        except Exception:
            print("⚠️ Could not load parameters.yaml, using defaults/args")

        # Override with arguments
        self.sim_config['vehicle']['id'] = car_id
        
        # Map dynamic_model_type int to string config ONLY if provided
        if dynamic_model_type is not None:
            model_map = {0: 'kinematic', 1: 'dynamic', 2: 'qlpv_legacy', 3: 'qlpv_matrix'}
            self.sim_config['vehicle']['model_type'] = model_map.get(dynamic_model_type, 'kinematic')
            
        if vehicle_params is not None:
            self.sim_config['vehicle']['params_file'] = vehicle_params
            
        if tire_model is not None:
            self.sim_config['vehicle']['tire_model'] = tire_model
        
        # Create mock hardware
        self.mock_qcar = MockQCar(self.sim_config)
        self.mock_gps = self.mock_qcar.gps
        
        # Create real configuration for VehicleLogic
        self.config = self._create_real_config()
        self.kill_event = Event()
        
        # Create the REAL VehicleLogic
        self.vehicle_logic = VehicleLogic(self.config, self.kill_event)
        
        # Set a reference so the fake initialization state can access our mock hardware
        self.vehicle_logic._parent_fake_vehicle = self
        
        # Components injection will happen in _inject_mock_hardware
        self._inject_mock_hardware()
        
        # Monkey patch _observer_update
        self._patch_observer_update()

        # Initialize state for main loop
        self.running = True
        self.ground_station_client = None
        self.start_time = time.time()
        self._fake_initialization_replaced = False
        
        print(f"✅ Real VehicleLogic initialized for Car {car_id} using modular MockQCar")

    def _create_real_config(self) -> VehicleMainConfig:
        """Create real configuration for VehicleLogic"""
        config = VehicleMainConfig()
        config.network.car_id = self.car_id
        config.network.host_ip = self.host_ip
        config.network.base_port = self.base_port
        config.logging.enable_telemetry_logging = True
        config.timing.controller_update_rate = 200
        config.timing.telemetry_send_rate = 20
        config.timing.tf = 500.0

        config.path.path_number = 2 
        return config
    
    def _replace_initialization_state_only(self):
        """Replace only the INITIALIZING state with fake version"""
        try:
            if self._fake_initialization_replaced:
                return

            import time
            start_time = time.time()
            while not hasattr(self.vehicle_logic, 'state_machine') and (time.time() - start_time) < 5.0:
                time.sleep(0.1)
            
            if not hasattr(self.vehicle_logic, 'state_machine'):
                print("🔧 State machine not found, VehicleLogic may not be fully initialized")
                return
            
            from fake_initializing_state import FakeInitializingState
            fake_init_state = FakeInitializingState(self.vehicle_logic)
            self.vehicle_logic.state_machine.state_handlers[VehicleState.INITIALIZING] = fake_init_state
            self._fake_initialization_replaced = True
            
            if self.vehicle_logic.state_machine.state == VehicleState.INITIALIZING:
                print(f"[!] State machine already in INITIALIZING - calling fake enter() now")
                fake_init_state.enter()
                
        except Exception as e:
            print(f"❌ Car {self.car_id}: Failed to replace initialization state: {e}")

    def _inject_mock_hardware(self):
        """Mock hardware injection deferred to initialization state"""
        print(f"🔧 Car {self.car_id}: Mock hardware injection deferred to initialization state")
    
    def _patch_observer_update(self):
        """Monkey patch VehicleLogic._observer_update to inject ground truth provider"""
        original_observer_update = self.vehicle_logic._observer_update
        
        def patched_observer_update(dt: float):
            original_observer_update(dt)
            try:
                if hasattr(self.vehicle_logic, 'vehicle_observer'):
                    est = self.vehicle_logic.vehicle_observer.get_local_estimator()
                    
                    # Inject Ground Truth Provider if missing
                    if est and hasattr(est, 'set_ground_truth_provider') and getattr(est, 'ground_truth_provider', None) is None:
                        print(f"✅ [SIM] Injecting MockQCar as Ground Truth Provider into Observer")
                        est.set_ground_truth_provider(self.mock_qcar)

                    # Patch internal observer update for true steering
                    if est and hasattr(est, 'observer') and not getattr(est, '_patched_steering', False):
                        print(f"✅ [SIM] Monkey-patching Observer.update to inject true steering")
                        original_obs_update = est.observer.update
                        
                        def patched_obs_update(measurement, control_input, **kwargs):
                            true_delta = self.mock_qcar.current_steering_angle
                            u_flat = np.array(control_input).reshape(-1)
                            throttle_cmd = u_flat[1] if len(u_flat) > 1 else 0.0
                            steering_cmd = u_flat[0]
                            u_new = np.array([steering_cmd, throttle_cmd, true_delta])
                            return original_obs_update(measurement, u_new, **kwargs)
                        
                        est.observer.update = patched_obs_update
                        est._patched_steering = True
            except Exception:
                pass
                
        self.vehicle_logic._observer_update = patched_observer_update

    def start_simulation(self):
        """Start the fake vehicle simulation"""
        print("\\n" + "="*60)
        print("[SIM] Starting Real VehicleLogic with Modular Mock Hardware")
        print("="*60)
        self._replace_initialization_state_only()
    
    def run(self):
        """Run the real VehicleLogic directly"""
        try:
            self._replace_initialization_state_only()
            self.vehicle_logic.run()
        except Exception as e:
            print(f"❌ Car {self.car_id}: VehicleLogic error - {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.running = False
            self.kill_event.set()
    
    def stop(self):
        """Stop the simulation"""
        self.running = False
        self.kill_event.set()
        if self.ground_station_client:
            try:
                self.ground_station_client.close()
            except Exception:
                pass

def main():
    """Main entry point"""
    import sys
    
    # Defaults
    car_id = 0
    host_ip = '127.0.0.1'
    base_port = 5000
    dynamic_model_type = None 
    vehicle_params = None
    tire_model = None
    
    # Parse args (Backward functionality)
    args = sys.argv[1:]
    if len(args) > 0 and args[0].isdigit() and int(args[0]) < 1000:
        car_id = int(args[0])
        args.pop(0)

    for arg in args:
        val = arg.lower()
        if val in ['0', 'kinematic', 'ks']: dynamic_model_type = 0
        elif val in ['1', 'dynamic', 'st']: dynamic_model_type = 1
        elif val in ['2', 'qlpv']: dynamic_model_type = 2
        elif val in ['3', 'qlpv_matrix']: dynamic_model_type = 3
        
        elif val in ['pacejka', 'dynamic_linear', 'static_linear']: tire_model = val
        elif val in ['qcar', 'vehicle1', 'vehicle2', 'vehicle3', 'vehicle4']: vehicle_params = val
        
        elif arg.isdigit() and int(arg) > 1000: base_port = int(arg)
        elif '.' in arg or val == 'localhost': host_ip = arg
    
    print(f"🚀 Starting Simulation for Car {car_id}...")
    try:
        vehicle = FakeVehicleWithRealLogic(car_id, host_ip, base_port, 
                                          dynamic_model_type=dynamic_model_type,
                                          vehicle_params=vehicle_params,
                                          tire_model=tire_model)
    except Exception as e:
        print(f"❌ Failed to create vehicle: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    # Check for CLI disturbance overrides
    full_args = sys.argv[1:]
    if any('dist_sine' in a for a in full_args):
        print("🌊 Injecting SINE disturbance on v_y")
        vehicle.mock_qcar.disturbance_gen.set_disturbance('vy', 'sine', value=0.5, freq=0.5)
        
    if any('dist_step' in a for a in full_args):
        print("🌊 Injecting STEP disturbance on v_y")
        vehicle.mock_qcar.disturbance_gen.set_disturbance('vy', 'step', value=0.5)
        
    if any('dist_vx' in a for a in full_args):
        print("🌊 Injecting SINE disturbance on v_x")
        vehicle.mock_qcar.disturbance_gen.set_disturbance('vx', 'sine', value=0.2, freq=0.2)

    vehicle.start_simulation()
    
    try:
        vehicle.run()
    except KeyboardInterrupt:
        print(f"\n🛑 Shutting down...")
    finally:
        vehicle.stop()
        return 0

if __name__ == '__main__':
    sys.exit(main())
