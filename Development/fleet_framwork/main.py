import os, time, sys
import signal
import multiprocessing
import threading

# Import the new simple configuration system
from simple_config import  ConfigPresets
SIMPLE_CONFIG_AVAILABLE = True


from QcarFleet import QcarFleet


class FleetApplication:
    """Main application class for fleet simulation management."""

    def __init__(self):
        self.config = None
        self.fleet = None
        self.shutdown_event = threading.Event()

    def run(self):
        """Clean application lifecycle"""
        exit_code = 0
        try:
            self._setup_signal_handlers()
            self._load_configuration()
            self._create_fleet()
            self._run_main_loop()
        except KeyboardInterrupt:
            print("\nShutdown requested...")
            exit_code = 0
        except (RuntimeError, ValueError) as e:
            print(f"\nApplication error: {e}")
            print("Exiting...")
            exit_code = 1
        except Exception as e:
            print(f"\nUnexpected error: {e}")
            import traceback
            traceback.print_exc()
            exit_code = 1
        finally:
            self._cleanup()
            if exit_code != 0:
                sys.exit(exit_code)

    def _setup_signal_handlers(self):
        """Setup signal handlers for graceful shutdown"""
        def signal_handler(signum, frame):
            print("SIGINT received. Stopping all processes...")
            self.shutdown_event.set()

        signal.signal(signal.SIGINT, signal_handler)

    def _load_configuration(self) -> None:
        """Clean config loading logic"""
        self.config = ConfigPresets.load_from_file("config.yaml")

        # Print current configuration
        print("\nCurrent Configuration:")
        self.config.print_config()

    def _create_fleet(self):
        """Fleet creation logic"""
        try:
            # Extract parameters from config (same interface as before)
            QcarNum = self.config.qcar_num
            LeaderIndex = self.config.leader_index
            DistanceBetweenEachCar = self.config.distance_between_cars
            Controller = self.config.get_controller_type_name()
            Observer = ""
            QlabType = self.config.get_road_type_name()

            # Create fleet with new process-based architecture
            self.fleet = QcarFleet(QcarNum, LeaderIndex, DistanceBetweenEachCar, Controller, Observer, QlabType, self.config)

            print("Fleet created with process-based vehicle architecture")
            print(f"Number of vehicles: {QcarNum}")
            print(f"Leader index: {LeaderIndex}")
            print(f"Controller type: {Controller}")

            # Start all vehicle processes
            # print("Starting fleet vehicle processes...")
            self.fleet.FleetBuilding()
        except (RuntimeError, ValueError) as e:
            print(f"\nFleet creation failed: {e}")
            print("Please ensure QLabs is running and configuration is valid.")
            raise  # Re-raise to trigger cleanup

    def _run_main_loop(self):
        """Clean monitoring loop"""
        print("Main Process running - monitoring fleet status")
        start_time = time.time()
        last_status_time = start_time

        while not self.shutdown_event.is_set():
            current_time = time.time()

            # Check if simulation time limit reached
            if self.config.simulation_time > 0 and (current_time - start_time) >= self.config.simulation_time:
                print(f"Simulation time limit ({self.config.simulation_time}s) reached")
                break

            # Check if all vehicle processes are still alive
            if not self.fleet.is_fleet_alive():
                print("All vehicle processes have stopped")
                break

            # Print fleet status every 5 seconds
            if current_time - last_status_time >= 5:
                status = self.fleet.get_fleet_status()
                alive_count = sum(1 for v in status.values() if v['alive'])
                print(f"Fleet status: {alive_count}/{self.config.qcar_num} vehicle processes alive")
                last_status_time = current_time

            time.sleep(0.05)  # Reduced from 0.1s to 50ms for better responsiveness

    def _cleanup(self):
        """Clean shutdown"""
        if self.fleet is not None:
            print("Stopping all vehicle processes...")
            try:
                self.fleet.FleetCanceling()
            except Exception as e:
                print(f"Error during fleet cleanup: {e}")

        # Clean up logging handlers
        try:
            from md_logging_config import cleanup_all_logging
            cleanup_all_logging()
        except Exception as e:
            print(f"Error during logging cleanup: {e}")

        print("Simulation Ends.")


def main():
    app = FleetApplication()
    app.run()


if __name__ == "__main__":
    # Set multiprocessing start method for Windows compatibility
    multiprocessing.set_start_method('spawn', force=True)
    main()