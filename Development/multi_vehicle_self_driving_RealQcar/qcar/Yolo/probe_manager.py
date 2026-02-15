
import time
import sys
import io
import cv2
import numpy as np
from contextlib import contextmanager
from typing import Optional

try:
    from pal.utilities.probe import Probe
except ImportError:
    Probe = None

@contextmanager
def _suppress_quanser_timeout():
    """Temporarily suppress stdout/stderr to silence Quanser 'The operation timed out.' spam."""
    old_stdout = sys.stdout
    old_stderr = sys.stderr
    sys.stdout = io.StringIO()
    sys.stderr = io.StringIO()
    try:
        yield
    finally:
        sys.stdout = old_stdout
        sys.stderr = old_stderr

class ProbeManager:
    """Manages probe connection with auto-reconnect and adaptive frame skipping.
    
    Designed to be fully silent when no Observer client is connected:
    - No exceptions propagate to the caller
    - Quanser timeout/stream messages are suppressed
    - Status is logged at throttled intervals only
    - Reconnection is only attempted after a prior successful connection is lost
    """
    
    DEAD_THRESHOLD = 5  # Consecutive failures before reconnection
    RECONNECT_COOLDOWN = 5.0  # Seconds between reconnection attempts
    LOG_INTERVAL = 30.0  # Seconds between "waiting for observer" messages
    
    def __init__(self, ip_host: str, car_id: int, height: int, width: int):
        self.ip_host = ip_host
        self.car_id = car_id
        self.height = height
        self.width = width
        
        # Connection state
        self.probe: Optional[Probe] = None
        self.frame_count = 0
        self.frame_skip = 4
        self.consecutive_failures = 0
        self.total_failures = 0
        self.last_success_time = 0.0  # 0 = never connected
        self.last_reconnect_time = 0.0
        self.last_log_time = 0.0
        self._ever_connected = False
        
        self._create_probe()
    
    def _create_probe(self):
        """Create and initialize probe instance. Never raises."""
        if Probe is None:
            print("[PROBE] Quanser Probe library not available.")
            return

        try:
            with _suppress_quanser_timeout():
                self.probe = Probe(ip=self.ip_host)
                # Use car_id directly as numDisplays for physical cars (as per original script)
                self.probe.numDisplays = self.car_id 
                self.probe.add_display(
                    imageSize=[self.height, self.width, 3],
                    name=f'YOLO Car {self.car_id}',
                    scalingFactor=1
                )
            print(f"[PROBE] Initialized for Car {self.car_id}")
        except Exception as e:
            print(f"[PROBE] Init failed for Car {self.car_id}: {e}")
            self.probe = None
    
    def send_frame(self, image: np.ndarray) -> bool:
        """Send frame with adaptive skip and auto-reconnect. Returns True if sent.
        
        Guaranteed to never raise an exception.
        """
        if self.probe is None:
            # Probe failed to init — try recreating at low frequency
            self._attempt_reconnect(time.time())
            return False
            
        self.frame_count += 1
        current_time = time.time()
        
        # Only attempt reconnect if we HAD a connection that died
        if self._ever_connected and self._is_connection_dead(current_time):
            self._attempt_reconnect(current_time)
            if self.probe is None:
                return False
        
        # Adaptive frame skipping
        self._adjust_frame_skip(current_time)
        
        # Skip frames to reduce load
        if self.frame_count % self.frame_skip != 0:
            return False
        
        # Check connection — suppress Quanser timeout spam
        try:
            with _suppress_quanser_timeout():
                self.probe.check_connection()
        except Exception:
            pass
        
        if not self.probe.connected:
            # Throttled log message so console is not spammed
            if current_time - self.last_log_time >= self.LOG_INTERVAL:
                self.last_log_time = current_time
                print(f"[PROBE] Car {self.car_id}: waiting for observer (connect multi_probing.py to see video)")
            return False
        
        try:
            # Resize for transmission
            resized = cv2.resize(image, (self.width, self.height))
            
            with _suppress_quanser_timeout():
                success = self.probe.send(name=f'YOLO Car {self.car_id}', imageData=resized)
            
            if success is True:
                self.last_success_time = current_time
                self._ever_connected = True
                self.consecutive_failures = 0
                self.total_failures = max(0, self.total_failures - 1)
                return True
            else:
                self._record_failure()
                return False
        except Exception as e:
            self._record_failure()
            if self.frame_count % 500 == 0:
                print(f"[PROBE] Send error Car {self.car_id}: {e}")
            return False
    
    def _is_connection_dead(self, current_time: float) -> bool:
        """Check if a previously working connection appears dead."""
        return (
            self.consecutive_failures >= self.DEAD_THRESHOLD or
            (self.last_success_time > 0 and
             current_time - self.last_success_time > 10.0 and
             self.consecutive_failures > 0)
        )
    
    def _attempt_reconnect(self, current_time: float):
        """Attempt to reconnect if cooldown has passed. Never raises."""
        if current_time - self.last_reconnect_time < self.RECONNECT_COOLDOWN:
            return
        
        self.last_reconnect_time = current_time
        print(f"[PROBE] Reconnecting Car {self.car_id}...")
        
        try:
            if self.probe:
                with _suppress_quanser_timeout():
                    try:
                        self.probe.terminate()
                    except Exception:
                        pass
            time.sleep(0.3)
            self._create_probe()
            self.consecutive_failures = 0
            self.total_failures = 0
        except Exception as e:
            print(f"[PROBE] Reconnection failed Car {self.car_id}: {e}")
            self.probe = None
    
    def _adjust_frame_skip(self, current_time: float):
        """Dynamically adjust frame skip based on success rate."""
        if self.total_failures > 10 and self.frame_skip < 8:
            self.frame_skip = min(8, self.frame_skip + 1)
            self.total_failures = 0
        elif self.last_success_time > 0 and current_time - self.last_success_time < 1.0 and self.frame_skip > 3:
            self.frame_skip = max(3, self.frame_skip - 1)
    
    def _record_failure(self):
        """Record a send failure."""
        self.consecutive_failures += 1
        self.total_failures += 1
    
    def terminate(self):
        """Clean shutdown. Never raises."""
        if self.probe:
            try:
                with _suppress_quanser_timeout():
                    self.probe.terminate()
            except Exception:
                pass
            self.probe = None
