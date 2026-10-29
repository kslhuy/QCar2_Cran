"""Independent simulated range/range-rate/bearing sensing.

Plant poses are confined to this sensor's world interface. Only noisy, delayed
relative measurements leave it; V2V estimates and clean attack channels are
never sensor inputs. The loopback transport joins separately launched fake cars.
"""
from collections import deque
import json
import math
import socket
import time

import numpy as np


class MockRelativeSensor:
    def __init__(self, config, vehicle_id):
        self.config = dict(config)
        self.vehicle_id = int(vehicle_id)
        self.period_ns = int(1e9 / float(config.get("rate_hz", 20.0)))
        self.max_age_ns = int(float(config.get("world_max_age_s", 0.15)) * 1e9)
        self.latency_ns = int(float(config.get("latency_s", 0.03)) * 1e9)
        self.min_range = float(config.get("min_range_m", 0.1))
        self.max_range = float(config.get("max_range_m", 8.0))
        self.half_fov = math.radians(float(config.get("field_of_view_deg", 360.0))) / 2
        self.noise = np.array([config.get("range_noise_std_m", 0.02),
                               config.get("range_rate_noise_std_mps", 0.03),
                               config.get("bearing_noise_std_rad", 0.01)], dtype=float)
        self.dropout = float(config.get("drop_probability", 0.01))
        if (self.period_ns <= 0 or self.max_age_ns <= 0 or self.latency_ns < 0
                or not 0 <= self.dropout <= 1 or np.any(self.noise < 0)
                or not np.all(np.isfinite(self.noise))
                or not 0 < self.min_range < self.max_range or not 0 < self.half_fov <= math.pi):
            raise ValueError("Invalid independent relative sensor configuration")
        self.rng = np.random.default_rng(int(config.get("seed", 31000)) + self.vehicle_id)
        self.next_sample_ns = 0
        self.pending = deque()

    def sample(self, host, peers, now_ns):
        """World state is [x,y,heading,world_vx,world_vy], never an estimate."""
        now_ns = int(now_ns)
        host = np.asarray(host, dtype=float)
        if host.shape != (5,) or not np.all(np.isfinite(host)):
            raise ValueError("Relative sensor requires a finite five-element host world state")
        if now_ns >= self.next_sample_ns:
            self.next_sample_ns = now_ns + self.period_ns
            for target_id, (timestamp_ns, state) in peers.items():
                if int(target_id) == self.vehicle_id:
                    continue
                age_ns = now_ns - int(timestamp_ns)
                if age_ns < 0 or age_ns > self.max_age_ns:
                    continue
                target = np.asarray(state, dtype=float)
                if target.shape != (5,) or not np.all(np.isfinite(target)):
                    raise ValueError("Relative sensor received invalid world state")
                # Interpolate the asynchronous world transport at capture time.
                displacement = target[:2] + target[3:5] * (age_ns / 1e9) - host[:2]
                distance = float(np.linalg.norm(displacement))
                bearing = math.atan2(displacement[1], displacement[0]) - host[2]
                bearing = math.atan2(math.sin(bearing), math.cos(bearing))
                if not self.min_range <= distance <= self.max_range or abs(bearing) > self.half_fov:
                    continue
                if self.rng.random() < self.dropout:
                    continue
                rate = float(np.dot(target[3:5] - host[3:5], displacement) / distance)
                measured = np.array([distance, rate, bearing]) + self.rng.normal(0, self.noise)
                if measured[0] <= 0:
                    continue
                packet = dict(target_id=int(target_id), distance=float(measured[0]),
                              relative_velocity=float(measured[1]), relative_bearing=float(measured[2]),
                              timestamp_ns=now_ns, source="simulated_radar", confidence=0.95)
                self.pending.append((now_ns + self.latency_ns, packet))
        output = []
        while self.pending and self.pending[0][0] <= now_ns:
            output.append(self.pending.popleft()[1])
        return output

    def reset(self):
        self.pending.clear()
        self.next_sample_ns = 0


class LoopbackRelativeSensor:
    """Local world transport for fake-car processes; entirely separate from V2V."""
    def __init__(self, vehicle_id, config):
        self.vehicle_id = int(vehicle_id)
        self.sensor = MockRelativeSensor(config, vehicle_id)
        self.vehicle_ids = tuple(int(i) for i in config.get("vehicle_ids", range(5)))
        if self.vehicle_id not in self.vehicle_ids:
            raise ValueError("relative_sensor.vehicle_ids must contain this fake car")
        self.base_port = int(config.get("world_base_port", 49000))
        if not self.vehicle_ids or min(self.vehicle_ids) < 0 or self.base_port < 1024 or self.base_port + max(self.vehicle_ids) > 65535:
            raise ValueError("Invalid relative sensor loopback ports")
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.bind(("127.0.0.1", self.base_port + self.vehicle_id))
            self.socket.setblocking(False)
        except Exception:
            self.socket.close()
            raise
        self.peers = {}
        self.next_publish_ns = 0

    @staticmethod
    def plant_state(car):
        c, s = math.cos(car.heading), math.sin(car.heading)
        return [car.x, car.y, car.heading,
                car.velocity * c - car.lateral_velocity * s,
                car.velocity * s + car.lateral_velocity * c]

    def poll(self, car, now_ns=None):
        now_ns = time.time_ns() if now_ns is None else int(now_ns)
        host = self.plant_state(car)
        if now_ns >= self.next_publish_ns:
            self.next_publish_ns = now_ns + self.sensor.period_ns
            data = json.dumps(dict(protocol="relative_sensor_world_v1", vehicle_id=self.vehicle_id,
                                   timestamp_ns=now_ns, state=host), allow_nan=False).encode("utf-8")
            for peer in self.vehicle_ids:
                if peer != self.vehicle_id:
                    self.socket.sendto(data, ("127.0.0.1", self.base_port + peer))
        while True:
            try:
                data, address = self.socket.recvfrom(4096)
            except BlockingIOError:
                break
            # Windows reports ICMP port-unreachable when another fake car has
            # not started. This means no detection, never a substitute source.
            except ConnectionResetError:
                break
            packet = json.loads(data.decode("utf-8"))
            target = int(packet["vehicle_id"])
            if (address[0] != "127.0.0.1" or address[1] != self.base_port + target
                    or target not in self.vehicle_ids or packet["protocol"] != "relative_sensor_world_v1"):
                raise ValueError("Unexpected relative sensor world packet")
            self.peers[target] = (int(packet["timestamp_ns"]), packet["state"])
        self.peers = {target: item for target, item in self.peers.items()
                      if 0 <= now_ns - item[0] <= self.sensor.max_age_ns}
        return self.sensor.sample(host, self.peers, now_ns)

    def reset(self):
        self.sensor.reset()
        self.peers.clear()
        self.next_publish_ns = 0

    def close(self):
        self.socket.close()
