"""
Pure Windows Python entrypoint for running the existing vehicle stack in CARLA.

This is intentionally ROS-free. CARLA provides the vehicle actor, GPS/IMU,
camera, lidar, collision sensor, and world ticks; VehicleLogic still provides
the observer, state machine, controllers, telemetry, and command handling.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import time
from threading import Event
from typing import List, Optional

import yaml

from Carla import CarlaBridgeConfig, CarlaSimulationBridge
from StateMachine import VehicleState
from StateMachine.carla_initializing_state import CarlaInitializingState
from command_types import CommandType
from config_main import VehicleMainConfig
from vehicle_logic import VehicleLogic

# python Development\multi_vehicle_self_driving_RealQcar\qcar\vehicle_main_carla.py --auto-start --spawn-point-index 0 --goal-spawn-indices 10 --draw-route --no-ground-station
kill_event = Event()
_active_bridge: Optional[CarlaSimulationBridge] = None


def signal_handler(*_args):
    print("\n[SIGNAL] CARLA bridge shutdown requested")
    kill_event.set()
    if _active_bridge is not None and _active_bridge.qcar_adapter is not None:
        try:
            _active_bridge.qcar_adapter.emergency_stop()
        except Exception:
            pass


def parse_arguments():
    parser = argparse.ArgumentParser(
        prog="vehicle_main_carla.py",
        description="Pure-Python CARLA bridge for the QCar vehicle stack",
    )

    parser.add_argument("--host", default="127.0.0.1", help="CARLA host")
    parser.add_argument("--port", type=int, default=2000, help="CARLA RPC port")
    parser.add_argument("--timeout", type=float, default=10.0, help="CARLA RPC timeout")
    parser.add_argument(
        "--fixed-dt",
        type=float,
        default=0.05,
        help="CARLA fixed delta seconds for synchronous mode",
    )
    parser.add_argument(
        "--async-mode",
        action="store_true",
        help="Do not put CARLA into synchronous mode",
    )

    parser.add_argument("--car-id", type=int, default=0, help="Vehicle/car ID")
    parser.add_argument("--config", type=str, default=None, help="Optional JSON/YAML config")
    parser.add_argument("--v-ref", type=float, default=0.6, help="Path speed reference [m/s]")
    parser.add_argument("--tf", type=float, default=600.0, help="Run duration [s]")
    parser.add_argument(
        "--controller-rate",
        type=int,
        default=None,
        help="Controller/observer rate [Hz]. Default is 1/fixed-dt.",
    )

    parser.add_argument("--ego-role-name", default="ego", help="CARLA ego role_name")
    parser.add_argument(
        "--vehicle-blueprint",
        default="vehicle.lincoln.mkz_2020",
        help="CARLA vehicle blueprint ID",
    )
    parser.add_argument(
        "--spawn-point-index",
        type=int,
        default=0,
        help="Preferred CARLA spawn point index",
    )
    parser.add_argument(
        "--goal-spawn-indices",
        default="10",
        help="Comma-separated CARLA spawn indices used as route goals",
    )
    parser.add_argument(
        "--use-existing-ego",
        action="store_true",
        help="Attach to an existing vehicle with matching role_name/ros_name",
    )
    parser.add_argument(
        "--keep-ego",
        action="store_true",
        help="Do not destroy an ego spawned by this bridge on exit",
    )
    parser.add_argument(
        "--draw-route",
        action="store_true",
        help="Draw generated CARLA route in the simulator",
    )
    parser.add_argument(
        "--route-resolution",
        type=float,
        default=2.0,
        help="Global route planner sampling resolution [m]",
    )

    parser.add_argument(
        "--path-lateral-controller",
        default="stanley",
        choices=["stanley", "pure_pursuit", "lookahead", "path", "pp_map", "mpc"],
        help="Path lateral controller to use with CARLA waypoints",
    )
    parser.add_argument(
        "--path-longitudinal-controller",
        default="pid",
        choices=["pid", "qcar2_speed"],
        help="Path longitudinal controller",
    )

    parser.add_argument(
        "--command-to-speed-gain",
        type=float,
        default=6.0,
        help="Scale QCar-style throttle command into CARLA target speed",
    )
    parser.add_argument("--max-target-speed", type=float, default=2.5)
    parser.add_argument("--max-steer-rad", type=float, default=0.55)
    parser.add_argument("--steer-sign", type=float, default=1.0)
    parser.add_argument("--speed-kp", type=float, default=0.55)
    parser.add_argument("--speed-ki", type=float, default=0.02)
    parser.add_argument("--speed-kd", type=float, default=0.04)
    parser.add_argument("--max-throttle", type=float, default=0.65)
    parser.add_argument("--max-brake", type=float, default=0.8)
    parser.add_argument("--brake-gain", type=float, default=1.2)
    parser.add_argument("--stop-brake", type=float, default=0.35)

    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=360)
    parser.add_argument("--camera-fov", type=float, default=90.0)
    parser.add_argument("--lidar-range", type=float, default=30.0)
    parser.add_argument("--lidar-channels", type=int, default=32)
    parser.add_argument("--lidar-points-per-second", type=int, default=56000)
    parser.add_argument("--lidar-rotation-frequency", type=float, default=20.0)
    parser.add_argument("--obstacle-min-distance", type=float, default=0.35)
    parser.add_argument("--obstacle-max-distance", type=float, default=12.0)
    parser.add_argument("--obstacle-lateral-window", type=float, default=2.5)

    parser.add_argument(
        "--ground-station-host",
        default=None,
        help="Optional ground station host/IP. Omit for simulator-only local run.",
    )
    parser.add_argument(
        "--ground-station-port",
        type=int,
        default=None,
        help="Ground station base port",
    )
    parser.add_argument(
        "--no-ground-station",
        action="store_true",
        help="Force-disable ground station connection even if config enables it",
    )
    parser.add_argument("--log-dir", default=None, help="Vehicle log directory")
    parser.add_argument("--data-log-dir", default=None, help="CSV/data log directory")

    parser.add_argument(
        "--auto-start",
        action="store_true",
        help="Automatically dispatch START once initialization reaches WAITING_FOR_START",
    )
    parser.add_argument(
        "--auto-start-delay",
        type=float,
        default=0.2,
        help="Delay after WAITING_FOR_START before auto-start [s]",
    )
    parser.add_argument(
        "--continue-after-collision",
        action="store_true",
        help="Log collision and stop state, but keep the process alive",
    )

    return parser.parse_args()


def _parse_int_list(value: str) -> List[int]:
    result = []
    for part in str(value or "").split(","):
        part = part.strip()
        if part:
            result.append(int(part))
    return result


def load_configuration(args) -> VehicleMainConfig:
    if not args.config:
        config = VehicleMainConfig()
    else:
        config_path = os.path.abspath(args.config)
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Config file not found: {config_path}")

        with open(config_path, "r", encoding="utf-8") as handle:
            if config_path.lower().endswith(".json"):
                raw_config = json.load(handle)
            else:
                raw_config = yaml.safe_load(handle) or {}

        if isinstance(raw_config, dict) and "vehicles" in raw_config:
            config = VehicleMainConfig.from_fleet_yaml(config_path, args.car_id)
        elif config_path.lower().endswith(".json"):
            config = VehicleMainConfig.from_json(config_path)
        else:
            config = VehicleMainConfig.from_yaml(config_path)

    controller_rate = (
        int(args.controller_rate)
        if args.controller_rate is not None
        else max(int(round(1.0 / max(float(args.fixed_dt), 1e-6))), 1)
    )

    config.vehicle.vehicle_type = "Carla"
    config.vehicle.programme_type = "Py"
    config.vehicle.probing = False
    config.network.car_id = int(args.car_id)
    config.timing.controller_update_rate = controller_rate
    config.timing.observer_rate = controller_rate
    config.timing.tf = float(args.tf)

    if args.no_ground_station:
        config.network.host_ip = None
    elif args.ground_station_host is not None:
        config.network.host_ip = args.ground_station_host

    if args.ground_station_port is not None:
        config.network.base_port = int(args.ground_station_port)

    if args.log_dir is not None:
        config.logging.log_dir = args.log_dir
    if args.data_log_dir is not None:
        config.logging.data_log_dir = args.data_log_dir

    if not config.vehicle_geometry:
        config.vehicle_geometry = {
            "wheelbase": 2.87,
            "l_f": 1.44,
            "l_r": 1.43,
            "track": 1.60,
        }

    return config


def build_bridge_config(args) -> CarlaBridgeConfig:
    return CarlaBridgeConfig(
        host=args.host,
        port=args.port,
        timeout=args.timeout,
        fixed_delta_seconds=args.fixed_dt,
        synchronous_mode=not args.async_mode,
        ego_role_name=args.ego_role_name,
        vehicle_blueprint=args.vehicle_blueprint,
        spawn_point_index=args.spawn_point_index,
        goal_spawn_indices=_parse_int_list(args.goal_spawn_indices),
        use_existing_ego=args.use_existing_ego,
        destroy_spawned_ego=not args.keep_ego,
        route_sampling_resolution=args.route_resolution,
        draw_route=args.draw_route,
        camera_width=args.camera_width,
        camera_height=args.camera_height,
        camera_fov=args.camera_fov,
        lidar_range=args.lidar_range,
        lidar_channels=args.lidar_channels,
        lidar_points_per_second=args.lidar_points_per_second,
        lidar_rotation_frequency=args.lidar_rotation_frequency,
        max_steer_rad=args.max_steer_rad,
        steer_sign=args.steer_sign,
        command_to_speed_gain=args.command_to_speed_gain,
        max_target_speed=args.max_target_speed,
        speed_kp=args.speed_kp,
        speed_ki=args.speed_ki,
        speed_kd=args.speed_kd,
        max_throttle=args.max_throttle,
        max_brake=args.max_brake,
        brake_gain=args.brake_gain,
        stop_brake=args.stop_brake,
        obstacle_min_distance=args.obstacle_min_distance,
        obstacle_max_distance=args.obstacle_max_distance,
        obstacle_lateral_window=args.obstacle_lateral_window,
    )


def install_carla_state(vehicle_logic: VehicleLogic, bridge: CarlaSimulationBridge) -> None:
    vehicle_logic._carla_bridge = bridge
    vehicle_logic.qcar = bridge.qcar_adapter
    vehicle_logic.gps = bridge.gps_adapter
    vehicle_logic.waypoint_sequence = bridge.waypoint_sequence
    vehicle_logic.node_sequence = ["carla_route"]

    carla_init = CarlaInitializingState(vehicle_logic)
    vehicle_logic.state_machine.state_handlers[VehicleState.INITIALIZING] = carla_init
    if vehicle_logic.state_machine.state == VehicleState.INITIALIZING:
        carla_init.enter()


def configure_controllers(vehicle_logic: VehicleLogic, args) -> None:
    cm = vehicle_logic.controller_manager
    cm.vehicle_type = "Carla"
    cm._path_longitudinal_type = args.path_longitudinal_controller
    cm._path_lateral_type = args.path_lateral_controller
    cm._longitudinal_type = args.path_longitudinal_controller
    cm._lateral_type = args.path_lateral_controller
    cm.clear()

    if cm.config is not None and hasattr(cm.config, "config"):
        cm.config.config["path_longitudinal_controller_type"] = (
            args.path_longitudinal_controller
        )
        cm.config.config["path_lateral_controller_type"] = args.path_lateral_controller
        cm.config.config["longitudinal_controller_type"] = (
            args.path_longitudinal_controller
        )
        cm.config.config["lateral_controller_type"] = args.path_lateral_controller

    vehicle_logic.invalidate_periodic_status_cache()


def run_carla_loop(
    vehicle_logic: VehicleLogic,
    bridge: CarlaSimulationBridge,
    args,
) -> None:
    target_dt = float(bridge.config.fixed_delta_seconds)
    if not bridge.config.synchronous_mode:
        target_dt = 1.0 / max(float(vehicle_logic.controller_rate), 1.0)

    vehicle_logic.start_time = time.time()
    vehicle_logic.vehicle_logger.set_start_time(vehicle_logic.start_time)
    vehicle_logic.loop_counter = 0
    vehicle_logic.telemetry_counter = 0
    vehicle_logic._latest_observer_state = None
    vehicle_logic._last_observer_time = 0.0
    vehicle_logic._last_control_time = 0.0

    auto_start_sent = False
    ready_since = None
    last_loop_time = time.time()

    try:
        while not kill_event.is_set():
            loop_begin = time.time()

            if hasattr(vehicle_logic, "watchdog"):
                vehicle_logic.watchdog.reset()

            bridge.tick()

            current_time = time.time()
            actual_dt = (
                target_dt
                if bridge.config.synchronous_mode
                else max(current_time - last_loop_time, 1e-6)
            )
            last_loop_time = current_time

            collision = bridge.consume_collision_report()
            if collision is not None:
                vehicle_logic.vehicle_logger.log_warning("[CARLA] Collision detected")
                if bridge.qcar_adapter is not None:
                    bridge.qcar_adapter.emergency_stop()
                vehicle_logic.state_machine.emergency_stop("CARLA collision detected")
                if not args.continue_after_collision:
                    kill_event.set()

            opponent_payload = bridge.get_opponent_payload()
            if opponent_payload:
                vehicle_logic.update_opponent_data(opponent_payload)

            if vehicle_logic._should_update_observer(current_time):
                vehicle_logic._update_sensor_data(actual_dt)
                vehicle_logic._observer_update(actual_dt)

            if vehicle_logic._should_update_control(current_time):
                if not vehicle_logic._control_logic_update(actual_dt):
                    vehicle_logic.vehicle_logger.log_error("CARLA control logic failed")
                    break

            if (
                args.auto_start
                and not auto_start_sent
                and vehicle_logic.state_machine.state == VehicleState.WAITING_FOR_START
            ):
                if ready_since is None:
                    ready_since = current_time
                elif current_time - ready_since >= float(args.auto_start_delay):
                    vehicle_logic.command_handler.dispatch_manual_event(
                        CommandType.START,
                        {"source": "vehicle_main_carla", "auto_start": True},
                        source="vehicle_main_carla",
                    )
                    auto_start_sent = True
            elif vehicle_logic.state_machine.state != VehicleState.WAITING_FOR_START:
                ready_since = None

            vehicle_logic._send_telemetry_to_ground_station()
            vehicle_logic._broadcast_periodic_status()
            vehicle_logic._process_queued_commands()
            vehicle_logic._broadcast_v2v_state()

            loop_time = time.time() - loop_begin
            if hasattr(vehicle_logic, "perf_monitor"):
                vehicle_logic.perf_monitor.log_loop_time(loop_time)

            sleep_time = target_dt - loop_time
            if sleep_time > 0.0:
                time.sleep(sleep_time)

            vehicle_logic.loop_counter += 1

            if vehicle_logic.elapsed_time() > vehicle_logic.config.timing.tf:
                vehicle_logic.vehicle_logger.logger.info("CARLA time limit reached")
                break

    except KeyboardInterrupt:
        vehicle_logic.vehicle_logger.logger.info("CARLA bridge interrupted by user")
        kill_event.set()
    except Exception as exc:
        vehicle_logic.vehicle_logger.log_error("CARLA bridge loop error", exc)
        kill_event.set()
    finally:
        vehicle_logic._shutdown()
        bridge.destroy()


def main() -> int:
    global _active_bridge

    signal.signal(signal.SIGINT, signal_handler)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, signal_handler)

    print("=" * 70)
    print(" Pure Python CARLA Bridge for QCar VehicleLogic")
    print("=" * 70)

    args = parse_arguments()
    config = load_configuration(args)

    print("\n[CONFIG] CARLA bridge")
    print(f"  CARLA: {args.host}:{args.port}")
    print(f"  Sync mode: {'enabled' if not args.async_mode else 'disabled'}")
    print(f"  fixed_dt: {args.fixed_dt:.3f} s")
    print(f"  Car ID: {config.network.car_id}")
    print(f"  v_ref: {args.v_ref:.2f} m/s")
    print(
        f"  Ground Station: {'disabled' if not config.network.is_remote_enabled else config.network.host_ip + ':' + str(config.network.port)}"
    )

    vehicle_logic = None
    bridge = None

    try:
        vehicle_logic = VehicleLogic(config, kill_event)
        vehicle_logic.v_ref = float(args.v_ref)
        vehicle_logic.controller_rate = int(config.timing.controller_update_rate)
        vehicle_logic.observer_rate = int(config.timing.observer_rate)

        bridge = CarlaSimulationBridge(
            build_bridge_config(args),
            logger=vehicle_logic.vehicle_logger,
        )
        _active_bridge = bridge
        bridge.setup()

        install_carla_state(vehicle_logic, bridge)
        configure_controllers(vehicle_logic, args)

        print("\n[READY] CARLA bridge running. Press Ctrl+C to stop.")
        if args.auto_start:
            print("[READY] Auto-start enabled")
        run_carla_loop(vehicle_logic, bridge, args)

    except Exception as exc:
        print(f"\n[ERROR] CARLA bridge failed: {exc}")
        if vehicle_logic is not None:
            vehicle_logic.vehicle_logger.log_error("CARLA bridge startup failed", exc)
            try:
                vehicle_logic._shutdown()
            except Exception:
                pass
        if bridge is not None:
            try:
                bridge.destroy()
            except Exception:
                pass
        return 1
    finally:
        _active_bridge = None

    print("\n" + "=" * 70)
    print(" CARLA bridge shutdown complete")
    print("=" * 70)
    return 0


if __name__ == "__main__":
    sys.exit(main())
