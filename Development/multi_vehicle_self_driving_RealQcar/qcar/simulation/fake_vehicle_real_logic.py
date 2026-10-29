"""
Fake Vehicle using REAL VehicleLogic Class - MODULAR REFACTOR
This entry point uses the qcar.simulation package for vehicle dynamics and disturbances.
"""

# Keep direct-file launches working, including uploads into a renamed robot folder.
if __package__ != "qcar.simulation":
    import importlib.util as _import_util
    import sys as _sys
    from pathlib import Path as _Path
    _root = _Path(__file__).resolve().parents[1]
    if "qcar" not in _sys.modules:
        _spec = _import_util.spec_from_file_location("qcar", _root / "__init__.py", submodule_search_locations=[str(_root)])
        _pkg = _import_util.module_from_spec(_spec)
        _sys.modules["qcar"] = _pkg
        _spec.loader.exec_module(_pkg)
    __package__ = "qcar.simulation"

import sys
import os
import time
import math
import numpy as np
from threading import Event
from typing import Optional

# Add parent directory to path to import qcar modules if needed
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Import the REAL VehicleLogic and related classes
from qcar.config_main import VehicleMainConfig
from qcar.StateMachine.vehicle_state import VehicleState

# Import modular simulation components
from qcar.simulation.mock_vehicle import MockQCar
from qcar.simulation.config import SimulationConfig
from qcar.electronics import (
    ElectronicsDataGateway,
    ElectronicsDigitalTwin,
    MockQCarElectronicsAdapter,
)

DIRECT_SPAWN_POSES_DEGREES = [
    (-1.064, -0.673, -39.775),
    (-1.491, -0.264, -39.775),
    (-1.884, 0.222, -80.0),
]


class FakeVehicleWithRealLogic:
    """Fake vehicle that uses the real VehicleLogic class with modular MockQCar"""
    
    def __init__(self, car_id: int, host_ip: str, base_port: int, 
                 dynamic_model_type: Optional[int] = None, 
                 vehicle_params: Optional[str] = None, 
                 tire_model: Optional[str] = None,
                 longitudinal_model: Optional[str] = None,
                 steering_model: Optional[str] = None,
                 use_direct_poses: bool = False,
                 local_estimator: Optional[str] = None):
        from qcar.vehicle_logic import VehicleLogic
        self.car_id = car_id
        self.host_ip = host_ip
        self.base_port = base_port
        self.initial_pose_override = self._get_direct_spawn_pose() if use_direct_poses else None
        self.initial_pose_source = (
            "initCars_Studio.py -u direct pose"
            if self.initial_pose_override is not None
            else "path calibration pose"
        )
        self.local_estimator_override = local_estimator
        
        # 1. Load and Configure MockQCar
        self.sim_config = SimulationConfig.load_config()

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

        if longitudinal_model is not None:
            self.sim_config['vehicle']['longitudinal_model'] = longitudinal_model

        if steering_model is not None:
            self.sim_config['vehicle']['steering_model'] = steering_model

        if self.initial_pose_override is not None:
            self.sim_config.setdefault('initial_state', {})
            self.sim_config['initial_state']['x'] = float(self.initial_pose_override[0])
            self.sim_config['initial_state']['y'] = float(self.initial_pose_override[1])
            self.sim_config['initial_state']['theta'] = float(self.initial_pose_override[2])
            print(
                f"[SIM] Car {self.car_id}: using initCars_Studio.py -u pose "
                f"x={self.initial_pose_override[0]:.3f}, "
                f"y={self.initial_pose_override[1]:.3f}, "
                f"theta={self.initial_pose_override[2]:.3f} rad"
            )
        
        # Create mock hardware
        self.mock_qcar = MockQCar(self.sim_config)
        self.mock_gps = self.mock_qcar.gps
        self.electronics_twin = None
        self.electronics_adapter = None
        self.electronics_gateway = None
        self.electronics_v2v_bridge = None
        electronics_config = self.sim_config.get('electronics', {})
        if electronics_config.get('enabled', False):
            self.electronics_twin = ElectronicsDigitalTwin(
                vehicle_id=self.car_id,
                config=electronics_config,
            )
            self.electronics_adapter = MockQCarElectronicsAdapter(self.electronics_twin)
            self.electronics_gateway = ElectronicsDataGateway.from_config(
                electronics_config.get('fusion')
            )
            self.mock_qcar.add_step_listener(self.electronics_adapter)
            print(
                f"[ELECTRONICS] Car {self.car_id}: digital twin enabled "
                f"(target={self.electronics_twin.hardware_manifest.profile}, "
                f"topology={self.electronics_twin.hardware_manifest.topology}, "
                f"{self.electronics_twin.sensor_compute_interface.upper()} sensor-compute, "
                f"{self.electronics_twin.vehicle_interface.upper()} vehicle link, "
                f"fusion={self.electronics_gateway.mode})"
            )
        
        # Create real configuration for VehicleLogic
        self.config = self._create_real_config()
        self.kill_event = Event()
        
        # Create the REAL VehicleLogic
        self.vehicle_logic = VehicleLogic(self.config, self.kill_event)
        
        # Set a reference so the fake initialization state can access our mock hardware
        self.vehicle_logic._parent_fake_vehicle = self
        self.vehicle_logic.electronics_twin = self.electronics_twin
        self.vehicle_logic.electronics_gateway = self.electronics_gateway
        if self.electronics_twin is not None:
            electronics_v2v_config = electronics_config.get('v2v', {})
            if electronics_v2v_config.get('enabled', True):
                self.electronics_v2v_bridge = (
                    self.vehicle_logic.attach_electronics_v2v_bridge(
                        electronics_v2v_config
                    )
                )
        
        # Components injection will happen in _inject_mock_hardware
        self._inject_mock_hardware()

        from qcar.simulation.relative_sensor import LoopbackRelativeSensor
        relative_config = self.sim_config.get('sensors', {}).get('relative', {})
        self.relative_sensor = (LoopbackRelativeSensor(self.car_id, relative_config)
                                if relative_config.get('enabled', False) else None)
        
        # Monkey patch _observer_update
        self._patch_observer_update()

        # Initialize state for main loop
        self.running = True
        self.ground_station_client = None
        self.start_time = time.time()
        self._fake_initialization_replaced = False
        
        print(f"✅ Real VehicleLogic initialized for Car {car_id} using modular MockQCar")

    def _create_real_config(self) -> VehicleMainConfig:
        """Create real configuration for VehicleLogic."""
        fleet_config_path = os.path.join(parent_dir, "fleet_config.yaml")
        config = VehicleMainConfig.from_fleet_yaml(fleet_config_path, self.car_id)

        config.network.car_id = self.car_id
        config.network.host_ip = self.host_ip
        config.network.base_port = self.base_port
        config.logging.enable_telemetry_logging = True
        config.timing.controller_update_rate = 200
        config.timing.telemetry_send_rate = 20
        config.timing.tf = 500.0

        return config

    def _get_direct_spawn_pose(self) -> Optional[np.ndarray]:
        """Return the same direct pose used by initCars_Studio.py -u."""
        if self.car_id < 0 or self.car_id >= len(DIRECT_SPAWN_POSES_DEGREES):
            raise ValueError(f"No configured direct spawn pose for car {self.car_id}")

        x, y, theta_deg = DIRECT_SPAWN_POSES_DEGREES[self.car_id]
        return np.array([x, y, math.radians(theta_deg)], dtype=float)
    
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
            
            from qcar.simulation.fake_initializing_state import FakeInitializingState
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
        """Feed independent detections before estimation; never inject plant truth into observers."""
        original_observer_update = self.vehicle_logic._observer_update

        def patched_observer_update(dt: float):
            if self.relative_sensor is not None:
                for packet in self.relative_sensor.poll(self.mock_qcar):
                    self.vehicle_logic.vehicle_observer.update_relative_measurement(
                        measurement=np.array([packet["distance"], packet["relative_velocity"]]),
                        target_id=packet["target_id"], source=packet["source"],
                        measurement_confidence=packet["confidence"],
                        timestamp_ns=packet["timestamp_ns"], relative_bearing=packet["relative_bearing"],
                    )
            original_observer_update(dt)

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
            raise
        finally:
            self.stop()
    
    def stop(self):
        """Stop the simulation"""
        self.running = False
        self.kill_event.set()
        if self.relative_sensor is not None:
            self.relative_sensor.close()
        if self.ground_station_client:
            try:
                self.ground_station_client.close()
            except Exception:
                pass
        if self.electronics_twin is not None:
            try:
                self.electronics_twin.close()
            except Exception:
                pass

def main():
    """Main entry point"""
    import sys
    import argparse

    parser = argparse.ArgumentParser(
        description="Run a fake vehicle with real fleet logic (route maps require Quanser HAL).",
        epilog="Legacy positional tokens remain supported: 0 qlpv qcar pacejka localhost 5000",
    )
    parser.add_argument("legacy", nargs="*", help="Legacy car ID, model, host and port tokens")
    parser.add_argument("--car-id", type=int, default=0)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("-u", "--use-direct-poses", action="store_true")
    parser.add_argument("--longitudinal-model")
    parser.add_argument("--steering-model")
    parser.add_argument("--local-estimator", choices=("ekf", "robust_kalman_net"))
    parsed = parser.parse_args()
    
    # Defaults
    car_id = parsed.car_id
    host_ip = parsed.host
    base_port = parsed.port
    dynamic_model_type = None 
    vehicle_params = None
    tire_model = None
    longitudinal_model = parsed.longitudinal_model
    steering_model = parsed.steering_model
    local_estimator = parsed.local_estimator
    use_direct_poses = parsed.use_direct_poses
    
    # Parse args (Backward functionality)
    args = parsed.legacy
    if len(args) > 0 and args[0].isdigit() and int(args[0]) < 1000:
        car_id = int(args[0])
        args.pop(0)

    for arg in args:
        val = arg.lower()
        if val.startswith("--longitudinal-model=") or val.startswith("longitudinal_model="):
            longitudinal_model = arg.split("=", 1)[1]
            continue
        if val.startswith("--steering-model=") or val.startswith("steering_model="):
            steering_model = arg.split("=", 1)[1]
            continue
        if val.startswith("--local-estimator="):
            local_estimator = arg.split("=", 1)[1]
            if local_estimator not in {"ekf", "robust_kalman_net"}:
                raise ValueError("--local-estimator must be ekf or robust_kalman_net")
            continue

        if val in ['0', 'kinematic', 'ks']: dynamic_model_type = 0
        elif val in ['1', 'dynamic', 'st']: dynamic_model_type = 1
        elif val in ['2', 'qlpv']: dynamic_model_type = 2
        elif val in ['3', 'qlpv_matrix']: dynamic_model_type = 3
        elif val in ['-u', '--use-direct-poses', 'use_direct_poses', 'direct_poses', 'direct']:
            use_direct_poses = True
        
        elif val in ['pacejka', 'dynamic_linear', 'static_linear']: tire_model = val
        elif val in ['qcar', 'vehicle1', 'vehicle2', 'vehicle3', 'vehicle4']: vehicle_params = val
        elif val in [
            'default',
            'direct_acceleration',
            'simple_acceleration',
            'qcar_real',
            'qcar_sim',
            'limo',
            'real_car_61',
            'qcar_real_61',
            'qlabs_velocity',
            'qlabs'
        ]:
            longitudinal_model = val
        elif val in ['qlabs_steering', 'steering_qlabs']:
            steering_model = val
        
        elif arg.isdigit() and int(arg) > 1000: base_port = int(arg)
        elif '.' in arg or val == 'localhost': host_ip = arg
    
    print(f"🚀 Starting Simulation for Car {car_id}...")
    try:
        vehicle = FakeVehicleWithRealLogic(car_id, host_ip, base_port, 
                                          dynamic_model_type=dynamic_model_type,
                                          vehicle_params=vehicle_params,
                                          tire_model=tire_model,
                                          longitudinal_model=longitudinal_model,
                                          steering_model=steering_model,
                                          use_direct_poses=use_direct_poses,
                                          local_estimator=local_estimator)
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
