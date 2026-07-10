"""
Unit tests for IOQCar2 - physical mock path + QLabs live path.

QLabs path: auto-connects to running QLabs. If no cars exist, spawns one
(like initCars_Studio.py) then runs hardware-level read/write tests.

Run from the qcar_refactor directory:
    python -m unittest test.unit_test_io_qcar
"""
import sys
import os
import time
import threading
import unittest
import numpy as np
from contextlib import redirect_stdout
from io import StringIO

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# ===========================================================================
# QLabs spawn helper (like initCars_Studio.py, but minimal)
# ===========================================================================

def _qlabs_available():
    """True if we're in a QLabs environment (not physical)."""
    if not sys.stdin.isatty() and "pal.products.qcar" not in sys.modules:
        return False
    try:
        from pal.products.qcar import IS_PHYSICAL_QCAR
        return not IS_PHYSICAL_QCAR
    except (ImportError, EOFError):
        return False


def _port_open(port, timeout=1.0):
    import socket
    s = socket.socket()
    s.settimeout(timeout)
    try:
        s.connect(("127.0.0.1", port))
        return True
    except Exception:
        return False
    finally:
        s.close()


def _spawn_qcar_in_qlabs():
    """Spawn one QCar in QLabs via MultiAgent.
    Returns dict from readRobots() on success, empty dict on failure.
    """
    from qvl.qlabs import QuanserInteractiveLabs
    from qvl.real_time import QLabsRealTime
    from qvl.multi_agent import MultiAgent, readRobots

    try:
        qlabs = QuanserInteractiveLabs()
        qlabs.open("localhost")
        qlabs.destroy_all_spawned_actors()
        QLabsRealTime().terminate_all_real_time_models()

        time.sleep(1.0)

        print("[SPAWN] Spawning QCar2 ...")
        MultiAgent([{
            "RobotType": "QCar2",
            "Location": [1.0, 0.0, 0.005],
            "Rotation": [0, 0, 0],
            "Radians": True,
            "Scale": 0.1,
        }])
        time.sleep(3.0)
        # MultiAgent already starts the copied QCar2 RT model with the generated
        # ports. Wait until the ports that IOQCar2 will use are actually open.
        for i in range(15):
            time.sleep(2.0)
            try:
                robots = readRobots()
                if robots:
                    name = list(robots.keys())[0]
                    hil = robots[name].get("hilPort")
                    gps = robots[name].get("gpsPort")
                    if hil and gps:
                        if _port_open(hil) and _port_open(gps):
                            print(f"[SPAWN] hil port {hil} OPEN, gps port {gps} OPEN")
                            return robots
            except Exception:
                pass
        return {}
    except Exception as e:
        print(f"[SPAWN] Failed: {e}")
        return {}


# ===========================================================================
# Tests — Live QLabs (auto-spawns if needed)
# ===========================================================================

@unittest.skipUnless(_qlabs_available(), "Not in QLabs environment")
class TestIOQCar2LiveQLabs(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        import socket
        from qvl.multi_agent import readRobots

        # Check if existing robots actually have a listening hil port
        robots = {}
        need_spawn = True
        try:
            robots = readRobots()
            if robots:
                name = list(robots.keys())[0]
                hil = robots[name].get("hilPort")
                gps = robots[name].get("gpsPort")
                if hil and gps and _port_open(hil) and _port_open(gps):
                    need_spawn = False
        except Exception:
            pass

        if need_spawn:
            if robots:
                print("[SETUP] Cars found but ports dead — re-spawning ...")
            else:
                print("[SETUP] No cars found — auto-spawning ...")
            robots = _spawn_qcar_in_qlabs()

        if not robots:
            raise unittest.SkipTest("QLabs spawn failed")

        name = list(robots.keys())[0]
        cls.vehicle_id = int(name.split("_")[-1])
        cls.car_cfg = robots[name]
        print(f"[SETUP] vehicle_id={cls.vehicle_id} ({name})")
        print(
            "[SETUP] ports "
            f"hil={cls.car_cfg.get('hilPort')} "
            f"gps={cls.car_cfg.get('gpsPort')} "
            f"lidar={cls.car_cfg.get('lidarPort')} "
            f"lidarIdeal={cls.car_cfg.get('lidarIdealPort')}"
        )

        cls.config = {
            "write": {"max_throttle": 0.10, "max_steering": 0.48},
            "read": {"sensor_rate_hz": 100, "gps_rate_hz": 20},
            "timing": {"loop_rate_hz": 100},
        }
        from utils.io.io_qcar2 import IOQCar2
        cls.io = IOQCar2(cls.config, vehicle_id=cls.vehicle_id)

    @classmethod
    def tearDownClass(cls):
        if hasattr(cls, "io"):
            close = getattr(cls.io, "close", None)
            if callable(close):
                close()
            else:
                cls.io.stop()

    def setUp(self):
        self.io = self.__class__.io

    def tearDown(self):
        self.io.stop()

    def test_read_sensors(self):
        self.io._poll_sensors()
        self.assertEqual(len(self.io._sensor_data_cache.accelerometer), 3)
        self.assertTrue(np.all(np.isfinite(self.io._sensor_data_cache.accelerometer)))

    def test_read_gps(self):
        gps_output = StringIO()
        for _ in range(30):
            with redirect_stdout(gps_output):
                self.io._poll_gps()
            if self.io._sensor_data_cache.gps_valid:
                break
            time.sleep(0.5)
        gps = self.io._gps
        self.assertTrue(
            self.io._sensor_data_cache.gps_valid,
            "GPS TCP port is open but no GPS packet was received. "
            f"gps_connected={getattr(getattr(gps, '_gps_client', None), 'connected', None)} "
            f"gpsPort={self.car_cfg.get('gpsPort')}",
        )
        self.assertEqual(len(self.io._sensor_data_cache.gps_position), 3)

    def test_read_lidar_status(self):
        gps_output = StringIO()
        with redirect_stdout(gps_output):
            gps = self.io._ensure_gps()
        lidar = getattr(gps, "lidar", None)
        if lidar is None or not hasattr(lidar, "_lidar"):
            output = " ".join(gps_output.getvalue().split())
            self.skipTest(
                "QCarGPS did not create a usable virtual QCarLidar client; "
                f"lidarPort={self.car_cfg.get('lidarPort')} output={output}"
            )

        lidar_valid = False
        lidar_output = StringIO()
        for _ in range(30):
            try:
                with redirect_stdout(lidar_output):
                    lidar_valid = bool(gps.readLidar())
            except Exception as e:
                self.skipTest(f"QCarGPS.readLidar() failed: {e}")
            if lidar_valid:
                break
            time.sleep(0.2)

        if not lidar_valid:
            output = " ".join(lidar_output.getvalue().split())
            self.skipTest(
                "Virtual lidar client connected object exists but produced no scan; "
                f"lidarPort={self.car_cfg.get('lidarPort')} output={output}"
            )

        self.assertEqual(len(gps.angles), len(gps.distances))
        self.assertTrue(np.all(np.isfinite(gps.distances)))

    def test_read_to_cache_then_read(self):
        self.io.read_sensors()
        data = self.io.read()
        self.assertIsNotNone(data)
        self.assertTrue(np.all(np.isfinite(data.accelerometer)))

    def test_write_zero(self):
        from core.types import ControlCommand
        self.io.write(ControlCommand(throttle=0.0, steering=0.0, target_velocity=0.0))

    def test_stop(self):
        self.io.stop()

    def test_continuous_poll_loop(self):
        for _ in range(20):
            self.io.read_sensors()
            data = self.io.read()
            self.assertIsNotNone(data)
            self.assertTrue(np.all(np.isfinite(data.accelerometer)))
            time.sleep(0.01)

    def test_background_poll_thread_main_thread_reads_and_plots(self):
        from core.types import ControlCommand

        stop_event = threading.Event()
        poll_errors = []
        rng = np.random.default_rng(42)

        def poll_worker():
            next_tick = time.monotonic()
            poll_output = StringIO()
            while not stop_event.is_set():
                try:
                    with redirect_stdout(poll_output):
                        self.io.read_to_cache()
                except Exception as e:
                    poll_errors.append(e)
                    stop_event.set()
                    break
                next_tick += 0.01
                time.sleep(max(0.0, next_tick - time.monotonic()))

        worker = threading.Thread(target=poll_worker, name="qcar-io-poll", daemon=True)
        worker.start()

        samples = []
        try:
            end_time = time.monotonic() + 20.0
            next_command_time = 0.0
            command = ControlCommand(throttle=0.0, steering=0.0, target_velocity=0.0)
            while time.monotonic() < end_time:
                elapsed = time.monotonic()
                if elapsed >= next_command_time:
                    command = ControlCommand(
                        throttle=float(rng.uniform(-1, 1)),
                        steering=float(rng.uniform(-1, 1)),
                        target_velocity=0.0,
                        source="unit_test_plot_random",
                    )
                    self.io.write(command)
                    next_command_time = elapsed + 3.0

                data = self.io.read()
                last_command = self.io.get_last_command()
                samples.append((
                    time.time(),
                    data.sensor_timestamp,
                    float(data.motor_tach),
                    float(data.gyro_z),
                    float(data.accelerometer[0]),
                    float(data.accelerometer[1]),
                    float(data.accelerometer[2]),
                    bool(data.gps_valid),
                    float(data.gps_position[0]),
                    float(data.gps_position[1]),
                    float(data.gps_position[2]),
                    float(data.gps_timestamp),
                    float(last_command.throttle),
                    float(last_command.steering),
                ))
                time.sleep(0.01)
        finally:
            self.io.write(ControlCommand(throttle=0.0, steering=0.0, target_velocity=0.0))
            stop_event.set()
            worker.join(timeout=2.0)

        self.assertFalse(poll_errors, f"background poll failed: {poll_errors}")
        self.assertGreaterEqual(len(samples), 20)

        sensor_timestamps = np.array([row[1] for row in samples], dtype=float)
        self.assertGreater(sensor_timestamps.max(), 0.0)
        self.assertGreater(np.unique(sensor_timestamps).size, 3)

        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as e:
            self.skipTest(f"matplotlib unavailable for plot artifact: {e}")

        t0 = samples[0][0]
        t = np.array([row[0] - t0 for row in samples], dtype=float)
        motor_tach = np.array([row[2] for row in samples], dtype=float)
        gyro_z = np.array([row[3] for row in samples], dtype=float)
        accel = np.array([[row[4], row[5], row[6]] for row in samples], dtype=float)
        gps_valid = np.array([1.0 if row[7] else 0.0 for row in samples], dtype=float)
        gps_position = np.array([[row[8], row[9], row[10]] for row in samples], dtype=float)
        gps_timestamps = np.array([row[11] for row in samples], dtype=float)
        commands = np.array([[row[12], row[13]] for row in samples], dtype=float)

        gps_packet_event = np.zeros_like(gps_timestamps)
        gps_packet_event[1:] = gps_timestamps[1:] > gps_timestamps[:-1]
        packet_times = t[gps_packet_event > 0.5]
        if packet_times.size >= 2:
            gps_intervals = np.diff(packet_times)
            observed_gps_rate = 1.0 / float(np.mean(gps_intervals))
            print(
                "[GPS] observed packet rate "
                f"{observed_gps_rate:.2f} Hz over {packet_times.size} packets "
                f"(mean interval {float(np.mean(gps_intervals)):.3f}s)"
            )
        else:
            print(f"[GPS] observed packet count {packet_times.size}; rate unavailable")

        fig, axes = plt.subplots(7, 1, figsize=(10, 14), sharex=True)
        axes[0].plot(t, motor_tach, label="motor_tach")
        axes[0].legend(loc="upper right")
        axes[1].step(t, commands[:, 0], where="post", label="throttle_cmd")
        axes[1].step(t, commands[:, 1], where="post", label="steering_cmd")
        axes[1].legend(loc="upper right")
        axes[2].plot(t, gyro_z, label="gyro_z")
        axes[2].legend(loc="upper right")
        axes[3].plot(t, accel[:, 0], label="ax")
        axes[3].plot(t, accel[:, 1], label="ay")
        axes[3].plot(t, accel[:, 2], label="az")
        axes[3].legend(loc="upper right")
        axes[4].step(t, gps_valid, where="post", label="gps_valid")
        axes[4].legend(loc="upper right")
        axes[5].step(t, gps_packet_event, where="post", label="gps_packet_event")
        axes[5].legend(loc="upper right")
        axes[6].plot(t, gps_position[:, 0], label="gps_x")
        axes[6].plot(t, gps_position[:, 1], label="gps_y")
        axes[6].plot(t, gps_position[:, 2], label="gps_yaw")
        axes[6].set_xlabel("main-thread read time [s]")
        axes[6].legend(loc="upper right")
        fig.tight_layout()

        output_dir = os.path.join(os.path.dirname(__file__), "artifacts")
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, "qcar_io_background_buffer.png")
        fig.savefig(output_path, dpi=120)
        plt.close(fig)
        print(f"[PLOT] saved {output_path}")

        # Save raw data as CSV artifact for future regression tests
        import csv
        csv_path = os.path.join(output_dir, "qcar_io_background_buffer.csv")
        csv_header = [
            "wall_time", "sensor_timestamp", "motor_tach", "gyro_z",
            "accel_x", "accel_y", "accel_z",
            "gps_valid", "gps_x", "gps_y", "gps_yaw", "gps_timestamp",
            "cmd_throttle", "cmd_steering",
        ]
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(csv_header)
            for rel_t, sample in zip(t, samples):
                row = [float(rel_t)] + list(sample[1:])
                writer.writerow(row)
        print(f"[CSV] saved {csv_path} ({len(samples)} rows)")


if __name__ == "__main__":
    if _qlabs_available():
        _spawn_qcar_in_qlabs()
    unittest.main()
