"""
Unit tests for QCar2IO — physical mock path + QLabs live path.

QLabs path: auto-connects to running QLabs. If no cars exist, spawns one
(like initCars_Studio.py) then runs hardware-level read/write tests.

Run from the qcar_refactor directory:
    python -m unittest test.unit_test_io_qcar
"""
import sys
import os
import time
import types
import unittest
from unittest.mock import patch
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


# ===========================================================================
# Mock QCar hardware (for physical-path unit tests)
# ===========================================================================

class MockQCar:
    def __init__(self, readMode=1, frequency=100, hilPort=None):
        self.readMode = readMode
        self.frequency = frequency
        self.hilPort = hilPort
        self.motorTach = 0.0
        self.gyroscope = [0.0, 0.0, 0.0]
        self.accelerometer = [0.0, 0.0, 0.0]
        self._last_write = None
        self._read_should_fail = False
        self._read_count = 0

    def read(self):
        if self._read_should_fail:
            raise OSError("Mock QCar read failure")
        self._read_count += 1

    def write(self, throttle=None, steering=None):
        self._last_write = (throttle, steering)


class MockQCarGPS:
    def __init__(self, initialPose=None, calibrate=False,
                 gpsPort=None, lidarIdealPort=None):
        self.initialPose = initialPose
        self.calibrate = calibrate
        self.gpsPort = gpsPort
        self.lidarIdealPort = lidarIdealPort
        self.position = [0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0]
        self._read_gps_result = True
        self._read_gps_should_fail = False

    def readGPS(self):
        if self._read_gps_should_fail:
            raise OSError("Mock GPS read failure")
        return self._read_gps_result


def mock_readRobots():
    return {
        "QC2_0": {"hilPort": 18080, "gpsPort": 18081, "lidarIdealPort": 18082},
        "QC2_1": {"hilPort": 18090, "gpsPort": 18091, "lidarIdealPort": 18092},
    }


def _inject_mocks(is_physical=False):
    pal = types.ModuleType("pal")
    pal_products = types.ModuleType("pal.products")
    qcar_mod = types.ModuleType("pal.products.qcar")
    qcar_mod.QCar = MockQCar
    qcar_mod.QCarGPS = MockQCarGPS
    qcar_mod.IS_PHYSICAL_QCAR = is_physical
    pal_products.qcar = qcar_mod
    pal.products = pal_products
    sys.modules["pal"] = pal
    sys.modules["pal.products"] = pal_products
    sys.modules["pal.products.qcar"] = qcar_mod

    qvl = types.ModuleType("qvl")
    qvl_ma = types.ModuleType("qvl.multi_agent")
    qvl_ma.readRobots = mock_readRobots
    qvl.multi_agent = qvl_ma
    sys.modules["qvl"] = qvl
    sys.modules["qvl.multi_agent"] = qvl_ma


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
        # ports. Wait until the ports that QCar2IO will use are actually open.
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
# Tests — Physical QCar (mocked)
# ===========================================================================

class TestQCar2IOPhysical(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        _inject_mocks(is_physical=True)
        from utils.io_utils.io_qcar2 import QCar2IO
        cls.QCar2IO = QCar2IO

    def setUp(self):
        self.config = {
            "write": {"max_throttle": 0.10, "max_steering": 0.48},
            "read": {"sensor_rate_hz": 100, "gps_rate_hz": 10},
            "timing": {"loop_rate_hz": 100},
        }
        sys.modules["pal.products.qcar"].QCar = MockQCar
        sys.modules["pal.products.qcar"].QCarGPS = MockQCarGPS
        self.io = self.QCar2IO(self.config, vehicle_id=0)

    def test_init_physical_no_ports(self):
        gps = self.io._ensure_gps()
        self.assertTrue(self.io._is_physical)
        self.assertEqual(self.io._qcar.readMode, 1)
        self.assertEqual(self.io._qcar.frequency, 100)
        self.assertIsNone(self.io._qcar.hilPort)
        self.assertEqual(gps.initialPose, [0, 0, 0])
        self.assertIsNone(gps.gpsPort)

    def test_poll_sensors_writes_to_cache(self):
        self.io._qcar.motorTach = 42.0
        self.io._qcar.gyroscope = [0.1, 0.2, 3.14]
        self.io._qcar.accelerometer = [9.81, 0.0, 0.0]
        self.io._poll_sensors()
        self.assertEqual(self.io._qcar._read_count, 1)
        self.assertEqual(self.io._sensor_data_cache.motor_tach, 42.0)
        self.assertAlmostEqual(self.io._sensor_data_cache.gyro_z, 3.14)

    def test_poll_sensors_exception_keeps_previous(self):
        self.io._qcar.motorTach = 10.0
        self.io._poll_sensors()
        self.io._qcar._read_should_fail = True
        self.io._qcar.motorTach = 99.0
        self.io._poll_sensors()
        self.assertEqual(self.io._sensor_data_cache.motor_tach, 10.0)

    def test_poll_gps_valid(self):
        gps = self.io._ensure_gps()
        gps.position = [12.0, 34.0]
        gps.orientation = [0.0, 0.0, 1.57]
        self.io._poll_gps()
        self.assertTrue(self.io._sensor_data_cache.gps_valid)
        np.testing.assert_array_almost_equal(
            self.io._sensor_data_cache.gps_position, [12.0, 34.0, 1.57])

    def test_poll_gps_stale_within_window(self):
        with patch.object(self.io, "_reading_gps_rate_hz", 10):
            gps = self.io._ensure_gps()
            gps.position = [1.0, 2.0]
            gps.orientation = [0.0, 0.0, 0.5]
            self.io._poll_gps()
            gps._read_gps_result = False
            with patch("time.time", return_value=time.time() + 0.05):
                self.io._poll_gps()
            self.assertTrue(self.io._sensor_data_cache.gps_valid)

    def test_poll_gps_expired(self):
        with patch.object(self.io, "_reading_gps_rate_hz", 10):
            gps = self.io._ensure_gps()
            gps.position = [1.0, 2.0]
            self.io._poll_gps()
            gps._read_gps_result = False
            with patch("time.time", return_value=time.time() + 0.15):
                self.io._poll_gps()
            self.assertFalse(self.io._sensor_data_cache.gps_valid)

    def test_poll_gps_exception_fallback(self):
        with patch.object(self.io, "_reading_gps_rate_hz", 10):
            gps = self.io._ensure_gps()
            gps.position = [3.0, 4.0]
            self.io._poll_gps()
            gps._read_gps_should_fail = True
            with patch("time.time", return_value=time.time() + 0.05):
                self.io._poll_gps()
            self.assertTrue(self.io._sensor_data_cache.gps_valid)

    def test_hardware_write(self):
        self.io._hardware_write(0.05, -0.30)
        self.assertEqual(self.io._qcar._last_write, (0.05, -0.30))

    def test_stop(self):
        self.io.stop()
        self.assertEqual(self.io._qcar._last_write, (0.0, 0.0))


# ===========================================================================
# Tests — QLabs Simulation (mocked ports)
# ===========================================================================

class TestQCar2IOQLabs(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        _inject_mocks(is_physical=False)
        from utils.io_utils.io_qcar2 import QCar2IO
        cls.QCar2IO = QCar2IO

    def setUp(self):
        self.config = {
            "write": {"max_throttle": 0.10, "max_steering": 0.48},
            "read": {"sensor_rate_hz": 100, "gps_rate_hz": 10},
            "timing": {"loop_rate_hz": 100},
        }
        sys.modules["pal.products.qcar"].QCar = MockQCar
        sys.modules["pal.products.qcar"].QCarGPS = MockQCarGPS

    def test_is_not_physical(self):
        io = self.QCar2IO(self.config, vehicle_id=0)
        self.assertFalse(io._is_physical)

    def test_hil_port_passed(self):
        io = self.QCar2IO(self.config, vehicle_id=0)
        self.assertEqual(io._qcar.hilPort, 18080)

    def test_gps_lidar_ports_passed(self):
        io = self.QCar2IO(self.config, vehicle_id=0)
        gps = io._ensure_gps()
        self.assertEqual(gps.gpsPort, 18081)
        self.assertEqual(gps.lidarIdealPort, 18082)

    def test_different_vehicle_id(self):
        io = self.QCar2IO(self.config, vehicle_id=1)
        gps = io._ensure_gps()
        self.assertEqual(io._qcar.hilPort, 18090)
        self.assertEqual(gps.gpsPort, 18091)


# ===========================================================================
# Tests — Live QLabs (auto-spawns if needed)
# ===========================================================================

@unittest.skipUnless(_qlabs_available(), "Not in QLabs environment")
class TestQCar2IOLiveQLabs(unittest.TestCase):

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
            "read": {"sensor_rate_hz": 100, "gps_rate_hz": 10},
            "timing": {"loop_rate_hz": 100},
        }

    @classmethod
    def tearDownClass(cls):
        pass

    def setUp(self):
        from utils.io_utils.io_qcar2 import QCar2IO
        self.io = QCar2IO(self.config, vehicle_id=self.vehicle_id)

    def tearDown(self):
        if hasattr(self, "io"):
            close = getattr(self.io, "close", None)
            if callable(close):
                close()
            else:
                self.io.stop()

    def test_read_sensors(self):
        self.io._poll_sensors()
        self.assertEqual(len(self.io._sensor_data_cache.accelerometer), 3)
        self.assertTrue(np.all(np.isfinite(self.io._sensor_data_cache.accelerometer)))

    def test_read_gps(self):
        for _ in range(30):
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
        gps = self.io._ensure_gps()
        lidar = getattr(gps, "lidar", None)
        if lidar is None or not hasattr(lidar, "_lidar"):
            self.skipTest(
                "QCarGPS did not create a usable virtual QCarLidar client; "
                f"lidarPort={self.car_cfg.get('lidarPort')}"
            )

        lidar_valid = False
        for _ in range(30):
            try:
                lidar_valid = bool(gps.readLidar())
            except Exception as e:
                self.skipTest(f"QCarGPS.readLidar() failed: {e}")
            if lidar_valid:
                break
            time.sleep(0.2)

        if not lidar_valid:
            self.skipTest(
                "Virtual lidar client connected object exists but produced no scan; "
                f"lidarPort={self.car_cfg.get('lidarPort')}"
            )

        self.assertEqual(len(gps.angles), len(gps.distances))
        self.assertTrue(np.all(np.isfinite(gps.distances)))

    def test_read_to_cache_then_read(self):
        self.io.read_to_cache()
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
            self.io.read_to_cache()
            data = self.io.read()
            self.assertIsNotNone(data)
            self.assertTrue(np.all(np.isfinite(data.accelerometer)))
            time.sleep(0.01)


if __name__ == "__main__":
    unittest.main()
