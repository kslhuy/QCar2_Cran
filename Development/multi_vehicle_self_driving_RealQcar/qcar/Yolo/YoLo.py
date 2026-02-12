import numpy as np
import time
from dataclasses import dataclass, field
from typing import Optional
from quanser.common import Timeout
from pal.utilities.stream import BasicStream


@dataclass
class YOLOData:
    """
    Immutable container for all YOLO detection data per frame.
    Created fresh each update cycle - no stale data issues.
    """
    # Detection arrays (raw 7-element arrays)
    stop_sign: np.ndarray = field(default_factory=lambda: np.zeros(7))
    traffic_light: np.ndarray = field(default_factory=lambda: np.zeros(7))
    cars: np.ndarray = field(default_factory=lambda: np.zeros(7))
    yield_sign: np.ndarray = field(default_factory=lambda: np.zeros(7))
    person: np.ndarray = field(default_factory=lambda: np.zeros(7))
    
    # Computed velocity gain from YOLODriveLogic
    yolo_gain: float = 1.0
    
    # Distance to detected objects
    car_dist: Optional[float] = None
    person_dist: Optional[float] = None
    
    # Lane detection data
    lane_confidence: float = 0.0
    lane_steering: float = 0.0
    lane_curvature: float = 0.0
    lane_offset: float = 0.0
    lane_left_detected: bool = False
    lane_right_detected: bool = False
    
    # Metadata
    is_valid: bool = False
    timestamp: float = 0.0
    
    def to_dict(self) -> dict:
        """Convert to dictionary for backward compatibility with sensor_data['yolo_data']."""
        return {
            'stop_sign': self.stop_sign,
            'traffic_light': self.traffic_light,
            'cars': self.cars,
            'yield_sign': self.yield_sign,
            'person': self.person,
            'car_dist': self.car_dist,
            'person_dist': self.person_dist,
            'lane_confidence': self.lane_confidence,
            'lane_steering': self.lane_steering,
            'lane_slope': self.lane_curvature,  # Alias for backward compat
            'lane_intercept': self.lane_offset,  # Alias for backward compat
            'lane_left_detected': self.lane_left_detected,
            'lane_right_detected': self.lane_right_detected,
        }

class YOLOReceiver():
    def __init__(self,ip='localhost',nonBlocking=True,port="18666"):
        self.stopSign = np.zeros((7),dtype=np.float64)
        self.trafficlight = np.zeros((7),dtype=np.float64)
        self.cars = np.zeros((7),dtype=np.float64)
        self.yieldSign = np.zeros((7),dtype=np.float64)
        self.person = np.zeros((7),dtype=np.float64)
        self.lane = np.zeros((7),dtype=np.float64)  # Lane detection data: [confidence, steering, slope, intercept, 0, 0, 0]
        
        # Shutdown flag to prevent reads during termination (avoids timeout spam)
        self._shutting_down = False

        self.uri='tcpip://'+ip+':'+port
        self._timeout = Timeout(seconds=0, nanoseconds=100000)
        self._handle = BasicStream(uri=self.uri,
                                    agent='C',
                                    receiveBuffer=np.zeros((6,7),  # Expanded to 6 rows for lane data
                                                           dtype=np.float64),
                                    recvBufferSize=7*6*8,  # Updated buffer size
                                    nonBlocking=nonBlocking,
                                    reshapeOrder='C')
        self.status_check('', iterations=30)

    def status_check(self, message, iterations=30):
        # blocking method to establish connection to the server stream.
        # Use 500ms timeout per iteration to give the YOLO server time to start
        self._timeout = Timeout(seconds=0, nanoseconds=500000000)  # 500ms
        counter = 0
        while not self._handle.connected:
            self._handle.checkConnection(timeout=self._timeout)
            counter += 1
            if self._handle.connected:
                print(message)
                break
            elif counter >= iterations:
                print(f'YOLO Server error: status check failed after {iterations} attempts.')
                break
            # Sleep 500ms between retries to allow server startup
            time.sleep(0.5)

    def read(self):
        new = False
        
        # Skip reads if shutting down (prevents timeout spam during disable)
        if self._shutting_down:
            return False
        
        # Increased timeout from 10ns to 100us for more reliable reception
        self._timeout = Timeout(seconds=0, nanoseconds=50)
        if self._handle.connected:
            new, bytesReceived = self._handle.receive(timeout=self._timeout, iterations=5)
            # print('read:',new, bytesReceived)
            # if new is True, full packet was received
            if new:
                self.stopSign[:] = self._handle.receiveBuffer[0,:]
                self.trafficlight[:] = self._handle.receiveBuffer[1,:]
                self.cars[:] = self._handle.receiveBuffer[2,:]
                self.yieldSign[:]= self._handle.receiveBuffer[3,:]
                self.person[:]= self._handle.receiveBuffer[4,:]
                self.lane[:] = self._handle.receiveBuffer[5,:]  # Extract lane data
        else:
            # Only try to reconnect if not shutting down
            if not self._shutting_down:
                self.status_check('Reconnected to yolo Server',iterations=1)
        return new

    def terminate(self):
        """Terminate the YOLO receiver. Sets shutdown flag first to prevent timeout errors."""
        self._shutting_down = True
        self._handle.terminate()
    
    def graceful_shutdown(self):
        """Signal shutdown to stop reads before actual termination."""
        self._shutting_down = True
    
    def __enter__(self):
        """ Used for with statement. """
        return self
    
    def __exit__(self, type, value, traceback):
        """ Used for with statement. Terminates the YOLO receiver. """
        self.terminate()

class YOLOPublisher():
    def __init__(self,ip='localhost',nonBlocking=False,port="18666"):

        self.uri='tcpip://'+ip+':'+port
        self._timeout = Timeout(seconds=0, nanoseconds=100000)
        self._handle = BasicStream(uri=self.uri,
                                    agent='S',
                                    sendBufferSize=7*6*8,  # Updated for 6 rows (added lane data)
                                    nonBlocking=nonBlocking,
                                    reshapeOrder='C')
        self.status_check('', iterations=20)

    def status_check(self, message, iterations=10):
        # blocking method to establish connection to the server stream.
        self._timeout = Timeout(seconds=0, nanoseconds=100000) #1000000
        counter = 0
        while not self._handle.connected:
            self._handle.checkConnection(timeout=self._timeout)
            counter += 1
            if self._handle.connected:
                print(message)
                break
            elif counter >= iterations:
                print('YOLO client error: status check failed.')
                break

    def send(self,yolodata):

        # data received flag
        new = False
        # 1 us timeout parameter
        self._timeout = Timeout(seconds=0, nanoseconds=100000)
        # set remaining packet to send
        self._sendPacket = yolodata
        # if connected to driver, send/receive
        if self._handle.connected:
            new = True
            self._handle.send(self._sendPacket)

        else:
            self.status_check('Reconnected to yolo client.')

        # if new is False, data is stale, else all is good
        return new

    def terminate(self):
        self._handle.terminate()

    def __enter__(self):
        """ Used for with statement. """
        return self
    
    def __exit__(self, type, value, traceback):
        """ Used for with statement. Terminates the YOLO publisher. """
        self.terminate()

class YOLOManager:
    """
    YOLOManager class manages YOLO components and provides high-level interface
    for vehicle_logic.py to reduce YOLO-related code in the main controller.
    
    Refactored API:
    - update(loop_counter) -> YOLOData: Reads, processes, and returns fresh data each cycle
    - get_data() -> YOLOData: Returns cached YOLOData from last update
    - get_gain() -> float: Convenience getter for velocity gain
    """
    
    def __init__(self, logger=None):
        self.logger = logger
        self.yolo = None
        self.yolo_drive = None
        self.loop_counter = 0
        self.yolo_enabled = False
        
        # Cached YOLOData - created fresh each update cycle
        self._cached_data: YOLOData = YOLOData()
        
    def initialize(self, yolo_receiver: 'YOLOReceiver', yolo_drive_logic: 'YOLODriveLogic'):
        """Initialize YOLO components"""
        if yolo_receiver is None or yolo_drive_logic is None:
            self.yolo_enabled = False  # Disable YOLO if components are missing
        else:
            self.yolo_enabled = True   # Enable YOLO only when both components are provided
        self.yolo = yolo_receiver
        self.yolo_drive = yolo_drive_logic
    
    def update(self, loop_counter: int = 0) -> YOLOData:
        """
        Update YOLO detection data - reads, processes, and returns fresh YOLOData.
        This is the primary method to call each control loop iteration.
        
        Returns:
            YOLOData: Fresh instance containing all detection data for this cycle
        """
        self.loop_counter = loop_counter
        
        # Create new YOLOData instance each cycle (user requested fresh instances)
        data = YOLOData(timestamp=time.time())
        
        try:
            if self.yolo is None:
                if self.loop_counter % 1000 == 0 and self.logger:
                    self.logger.log_error("YOLO receiver is None")
                self._cached_data = data
                return data
            
            # Read from YOLO receiver
            new_data = self.yolo.read()
            
            # # Debug logging
            # if self.loop_counter % 100 == 0 and self.logger:
            #     connection_status = "Connected" if self.yolo._handle.connected else "NOT Connected"
            #     self.logger.logger.info(f"[YOLO] Receiver status: {connection_status}, New data: {new_data}")
            
            # Copy detection arrays (create new arrays to avoid stale references)
            data.stop_sign = self.yolo.stopSign.copy()
            data.traffic_light = self.yolo.trafficlight.copy()
            data.cars = self.yolo.cars.copy()
            data.yield_sign = self.yolo.yieldSign.copy()
            data.person = self.yolo.person.copy()
            data.is_valid = new_data
            
            # Extract lane data from receiver
            lane = self.yolo.lane
            if len(lane) >= 6:
                data.lane_confidence = float(lane[0])
                data.lane_steering = float(lane[1])
                data.lane_curvature = float(lane[2])
                data.lane_offset = float(lane[3])
                data.lane_left_detected = lane[4] > 0.5
                data.lane_right_detected = lane[5] > 0.5
            
            # Process through YOLODriveLogic to get velocity gain
            if self.yolo_drive is not None:
                try:
                    data.yolo_gain = self.yolo_drive.check_yolo(
                        self.yolo.stopSign,
                        self.yolo.trafficlight,
                        self.yolo.cars,
                        self.yolo.yieldSign,
                        self.yolo.person
                    )
                    # Get computed distances from drive logic
                    data.car_dist = getattr(self.yolo_drive, 'carDist', None)
                    data.person_dist = getattr(self.yolo_drive, 'personDist', None)
                except Exception as e:
                    if self.loop_counter % 100 == 0 and self.logger:
                        self.logger.log_error("YOLO drive error", e)
                    data.yolo_gain = 1.0
            
        except Exception as e:
            if self.logger:
                self.logger.log_error("YOLO update error", e)
            data.yolo_gain = 1.0
        
        self._cached_data = data
        return data
    
    def get_data(self) -> YOLOData:
        """Get cached YOLOData from last update() call."""
        return self._cached_data
    
    def get_yolo_data(self) -> dict:
        """
        Get current YOLO detection data as dict.
        DEPRECATED: Use update() or get_data().to_dict() instead.
        Kept for backward compatibility with existing code.
        """
        return self._cached_data.to_dict()
    
    def get_default_yolo_data(self) -> dict:
        """Get default YOLO data when YOLO is not available"""
        return YOLOData().to_dict()
    
    def get_gain(self) -> float:
        """Get velocity gain from last update."""
        return self._cached_data.yolo_gain
    
    def get_yolo_gain(self) -> float:
        """Get current YOLO velocity gain (alias for backward compat)."""
        return self._cached_data.yolo_gain
        
    def is_yolo_active(self) -> bool:
        """Check if YOLO components are active"""
        return self.yolo is not None and self.yolo_drive is not None
    
    # Legacy method - redirects to update() 
    def update_yolo_data(self, loop_counter: int = 0) -> float:
        """
        DEPRECATED: Use update() instead.
        Kept for backward compatibility - returns only yolo_gain.
        """
        data = self.update(loop_counter)
        return data.yolo_gain
    
    def disable(self):
        """Disable YOLO system and clean up resources"""
        if self.logger:
            self.logger.logger.info("[YOLO] Disabling YOLO system...")
        
        # Signal graceful shutdown first to prevent timeout spam during disable
        if self.yolo is not None:
            try:
                self.yolo.graceful_shutdown()
            except Exception:
                pass  # Ignore errors during shutdown signal
        
        # Terminate YOLO receiver if it exists
        if self.yolo is not None:
            try:
                self.yolo.terminate()
                if self.logger:
                    self.logger.logger.info("[YOLO] YOLO receiver terminated")
            except Exception as e:
                if self.logger:
                    self.logger.logger.warning(f"[YOLO] Error terminating receiver: {e}")
        
        # Clear references
        self.yolo = None
        self.yolo_drive = None
        self.yolo_enabled = False
        self._cached_data = YOLOData()  # Reset to default
        
        if self.logger:
            self.logger.logger.info("[YOLO] YOLO system disabled")


class YOLODriveLogic():
    """
    YOLODriveLogic class implements the logic for processing YOLO predictions
    and determining the vehicle's velocity gain based on detected objects.
    
    Arguments:
        stopSignThreshold (float): Distance threshold for stop sign detection.
        trafficThreshold (float): Distance threshold for traffic light detection.
        carThreshold (float): Distance threshold for car detection.
        yieldThreshold (float): Distance threshold for yield sign detection.
        personThreshold (float): Distance threshold for person detection.
        pulseLength (int): Lengh of the pulse generated after detecting an object in number of frames.
    
    Methods:
        check_yolo(stopSign, trafficLight, QCar, yieldSign, person):
            Processes YOLO predictions and returns the velocity gain.
        stopSignPulse(stopSign):
            Handles stop sign detection logic.
        trafficPulse(trafficLight):
            Handles traffic light detection logic.
        yieldPulse(yieldSign):
            Handles yield sign detection logic.
        carPulse(car):
            Handles car detection logic.
        personPulse(person):
            Handles person detection logic.
    """
    def __init__(self,
                 stopSignThreshold = 0.6,
                 trafficThreshold = 1.7,
                 carThreshold = 0.3,
                 yieldThreshold = 1,
                 personThreshold = 0.6,
                 pulseLength = 300
                 ):
        self.counter = 0
        self.counter_yield = 0
        self.counter_traffic = 0
        self.counterStart_traffic = False
        self.counterStart = False
        self.counterStart_yield = False
        self.stopSignTrigger = 0
        self.carTrigger = 0
        self.trafficTrigger = 0
        self.yieldTrigger = 0
        self.personTrigger = 0
        self.vGain_person = 1
        self.vGain_yield = 1 
        self.vGain_stop = 1 
        self.vGain_car = 1 
        self.vGain = 1 
        self.stopSignThreshold = stopSignThreshold
        self.trafficThreshold = trafficThreshold
        self.carThreshold = carThreshold
        self.yieldThreshold = yieldThreshold
        self.personThreshold = personThreshold
        self.pulseLength = pulseLength

        self.carDist=100
        self.stopSignDist=100
        self.trafficLightDist=100
        self.yieldDist=100
        self.personDist=100


    def check_yolo(self,stopSign,trafficLight,QCar,yieldSign,person):
        ''' processes the YOLO predictions and returns the velocity gain.'''

        self.stopSignPulse(stopSign)
        self.trafficPulse(trafficLight)

        if self.stopSignTrigger == 1 or self.trafficTrigger == 1:
            self.vGain_stop = 0
            # return self.vGain

        self.carPulse(QCar)
        self.personPulse(person)
        # if self.carTrigger ==1 or self.personTrigger == 1:
        #     return self.vGain

        self.yieldPulse(yieldSign)
        if self.yieldTrigger ==1:
            self.vGain_yield = 0.5
            # return self.vGain
        
        self.vGain = min([self.vGain_yield,self.vGain_stop,self.vGain_car,self.vGain_person])
        self.vGain_yield = 1
        self.vGain_stop = 1
        self.vGain_car = 1
        self.vGain_person = 1
        return self.vGain
    
    def stopSignPulse(self,stopSign):
        ''' If a stop sign is closer than the threshold, a pulse with the length
        of self.pulseLength is generated, reducing the velocity gain to 0 for the 
        during of the pulse. After the pulse, the detection for stop sign is paused 
        for half the pulse time.'''

        stopSignCount = stopSign[0]
        stopSign[np.isnan(stopSign)]=10
        if stopSignCount>0:
            self.stopSignDist = stopSign[1:][stopSign[1:]!=0].min()
        else:
            self.stopSignDist=100
        if not self.counterStart:
            if stopSignCount>0 \
                and self.stopSignDist<self.stopSignThreshold:
                self.counterStart=True
                self.stopSignTrigger=1
            else:
                self.counterStart=False
                self.stopSignTrigger=0
        else:
            self.counter+=1
            if self.counter < self.pulseLength:
                self.stopSignTrigger = 1
            elif self.counter < self.pulseLength+int(self.pulseLength/2):
                self.stopSignTrigger = 0
            else:
                self.counter = 0
                self.counterStart = False
                self.stopSignTrigger = 0

    
    def trafficPulse(self,trafficLight):
        ''' If a red traffic light is closer than the threshold, a pulse with the length
        of self.pulseLength/6 is generated, reducing the velocity gain to 0 for the 
        during of the pulse. '''

        trafficLightCount = trafficLight[0]
        trafficLight[np.isnan(trafficLight)]=10

        if trafficLightCount>0:
            self.trafficLightDist = trafficLight[1:][trafficLight[1:]!=0].min()
        else:
            self.trafficLightDist=100
            self.trafficTrigger=0

        if not self.counterStart_traffic:
            if trafficLightCount>0 \
                and self.trafficLightDist<self.trafficThreshold \
                    and self.trafficLightDist> self.trafficThreshold - 0.6:
                self.trafficTrigger=1
                self.counterStart_traffic=True
            else:
                self.counterStart_traffic=False
                self.trafficTrigger=0
        else:
            self.counter_traffic+=1
            if self.counter_traffic < int(self.pulseLength/6):
                self.trafficTrigger = 1
            else:
                self.counter_traffic = 0
                self.counterStart_traffic = False
    
    def yieldPulse(self,yieldSign):
        ''' If a yeild sign is closer than the threshold, a pulse with the length
        of self.pulseLength is generated, reducing the velocity gain to 0.5 for the 
        during of the pulse. '''

        yieldSignCount = yieldSign[0]
        yieldSign[np.isnan(yieldSign)]=10
        if yieldSignCount>0:
            self.yieldDist = yieldSign[1:][yieldSign[1:]!=0].min()
        else:
            self.yieldDist=100
            self.yieldTrigger=0
        
        if not self.counterStart_yield:
            if yieldSignCount>0 \
                and self.yieldDist<self.yieldThreshold:
                self.counterStart_yield=True
                self.yieldTrigger=1
            else:
                self.counterStart_yield=False
                self.yieldTrigger=0
        else:
            self.counter_yield+=1
            if self.counter_yield < self.pulseLength:
                self.yieldTrigger = 1
            else:
                self.counter_yield = 0
                self.counterStart_yield = False

    def carPulse(self, car):
        ''' If a car is closer than the threshold, the velocity gain will be
        reduced to a value between 0 and 1, depending on the distance to the car.
        The speed will start decresing at the distance of 1.2, and will be 0
        at the distance of se;f.carThreshold.'''

        carCount = car[0]
        detect_threshold = 1.2
        car[np.isnan(car)]=10
        if carCount>0:
            self.carDist = car[1:][car[1:]!=0].min()
        else:
            self.carDist=100
            self.carTrigger=0
        if carCount>0 \
            and self.carDist<detect_threshold:
            self.carTrigger=1
            m=1/(detect_threshold-self.carThreshold)
            b=-m*self.carThreshold
            self.vGain_car = np.clip(m*self.carDist+b,0,1)
        else:
            self.carTrigger=0
    
    def personPulse(self, person):
        ''' If a person is closer than the threshold, the velocity gain will be
        reduced to a value between 0 and 1, depending on the distance to the car.
        The speed will start decresing at the distance of 1.5, and will be 0
        at the distance of self.personThreshold.'''

        personCount = person[0]
        detect_threshold = 1.5
        person[np.isnan(person)]=10
        if personCount>0:
            self.personDist = person[1:][person[1:]!=0].min()
        else:
            self.personDist=100
            self.personTrigger=0
        if personCount>0 \
            and self.personDist<detect_threshold:
            self.personTrigger=1
            m=1/(detect_threshold-self.personThreshold)
            b=-m*self.personThreshold
            self.vGain_person = np.clip(m*self.personDist+b,0,1)
        else:
            self.personTrigger=0


class YOLODriveLogicNew():
    """
    Improved YOLODriveLogic with time-based logic and better observability.
    
    Key improvements:
    - Time-based pulse logic (seconds) instead of frame-based
    - All thresholds configurable (no hard-coded values)
    - Logging support for detection reasons
    - Returns (gain, reason) tuple for better telemetry
    - Velocity-aware braking (optional)
    
    Arguments:
        stopSignThreshold (float): Distance threshold for stop sign detection (m).
        trafficThreshold (float): Distance threshold for traffic light detection (m).
        carThreshold (float): Minimum distance threshold for car detection (m).
        carDetectThreshold (float): Distance at which car detection starts (m).
        yieldThreshold (float): Distance threshold for yield sign detection (m).
        personThreshold (float): Minimum distance threshold for person detection (m).
        personDetectThreshold (float): Distance at which person detection starts (m).
        pulseDuration (float): Duration of pulse after detecting an object (seconds).
        trafficPulseDuration (float): Duration of pulse for traffic light (seconds).
        velocityAwareThresholds (bool): Scale thresholds by velocity if True.
        logger: Optional logger for detection events.
    """
    def __init__(self,
                 stopSignThreshold=0.6,
                 trafficThreshold=1.7,
                 carThreshold=0.3,
                 carDetectThreshold=1.2,
                 yieldThreshold=1.0,
                 personThreshold=0.6,
                 personDetectThreshold=1.5,
                 pulseDuration=1.5,
                 trafficPulseDuration=0.25,
                 velocityAwareThresholds=False,
                 logger=None
                 ):
        # Timers (all in seconds now)
        self.timer_stop = 0.0
        self.timer_yield = 0.0
        self.timer_traffic = 0.0
        
        # State flags
        self.pulse_active_stop = False
        self.pulse_active_yield = False
        self.pulse_active_traffic = False
        
        # Trigger states
        self.stopSignTrigger = 0
        self.carTrigger = 0
        self.trafficTrigger = 0
        self.yieldTrigger = 0
        self.personTrigger = 0
        
        # Velocity gains per detection type
        self.vGain_person = 1.0
        self.vGain_yield = 1.0 
        self.vGain_stop = 1.0 
        self.vGain_car = 1.0 
        self.vGain = 1.0
        
        # Configurable thresholds
        self.stopSignThreshold = stopSignThreshold
        self.trafficThreshold = trafficThreshold
        self.carThreshold = carThreshold
        self.carDetectThreshold = carDetectThreshold
        self.yieldThreshold = yieldThreshold
        self.personThreshold = personThreshold
        self.personDetectThreshold = personDetectThreshold
        
        # Time-based pulse durations
        self.pulseDuration = pulseDuration
        self.trafficPulseDuration = trafficPulseDuration
        
        # Distances to detected objects
        self.carDist = 100.0
        self.stopSignDist = 100.0
        self.trafficLightDist = 100.0
        self.yieldDist = 100.0
        self.personDist = 100.0
        
        # Velocity-aware mode
        self.velocityAwareThresholds = velocityAwareThresholds
        
        # Observability
        self.logger = logger
        self.last_detection_reason = "none"
        self.detection_active = False
        
    def check_yolo(self, stopSign, trafficLight, QCar, yieldSign, person, dt, current_velocity=0.75):
        """
        Process YOLO predictions and return velocity gain.
        
        Args:
            stopSign, trafficLight, QCar, yieldSign, person: YOLO detection arrays
            dt (float): Time delta since last call (seconds)
            current_velocity (float): Current vehicle velocity for adaptive thresholds
            
        Returns:
            tuple: (velocity_gain, detection_reason)
        """
        # Reset detection reason
        reasons = []
        
        # Process each detection type
        self.stopSignPulse(stopSign, dt)
        self.trafficPulse(trafficLight, dt)
        
        if self.stopSignTrigger == 1 or self.trafficTrigger == 1:
            self.vGain_stop = 0.0
            if self.stopSignTrigger == 1:
                reasons.append(f"stop_sign@{self.stopSignDist:.2f}m")
            if self.trafficTrigger == 1:
                reasons.append(f"red_light@{self.trafficLightDist:.2f}m")
        
        self.carPulse(QCar, current_velocity)
        self.personPulse(person, current_velocity)
        
        if self.carTrigger == 1:
            reasons.append(f"car@{self.carDist:.2f}m(gain={self.vGain_car:.2f})")
        if self.personTrigger == 1:
            reasons.append(f"person@{self.personDist:.2f}m(gain={self.vGain_person:.2f})")
        
        self.yieldPulse(yieldSign, dt)
        if self.yieldTrigger == 1:
            self.vGain_yield = 0.5
            reasons.append(f"yield@{self.yieldDist:.2f}m")
        
        # Combine all gains (most restrictive wins)
        self.vGain = min([self.vGain_yield, self.vGain_stop, self.vGain_car, self.vGain_person])
        
        # Store detection reason
        self.last_detection_reason = "; ".join(reasons) if reasons else "clear"
        self.detection_active = len(reasons) > 0
        
        # Log if detection is active and logger available
        if self.logger and self.detection_active:
            self.logger.logger.debug(f"[YOLO] Detection: {self.last_detection_reason} → gain={self.vGain:.2f}")
        
        # Reset individual gains for next iteration
        self.vGain_yield = 1.0
        self.vGain_stop = 1.0
        self.vGain_car = 1.0
        self.vGain_person = 1.0
        
        return self.vGain, self.last_detection_reason
    
    def stopSignPulse(self, stopSign, dt):
        """
        Time-based stop sign detection with pulse logic.
        Stops for pulseDuration seconds, then pauses detection for pulseDuration/2.
        """
        stopSignCount = stopSign[0]
        stopSign_clean = stopSign.copy()
        stopSign_clean[np.isnan(stopSign_clean)] = 10.0
        
        if stopSignCount > 0:
            self.stopSignDist = stopSign_clean[1:][stopSign_clean[1:] != 0].min()
        else:
            self.stopSignDist = 100.0
        
        if not self.pulse_active_stop:
            # Check for new detection
            if stopSignCount > 0 and self.stopSignDist < self.stopSignThreshold:
                self.pulse_active_stop = True
                self.timer_stop = 0.0
                self.stopSignTrigger = 1
                if self.logger:
                    self.logger.logger.info(f"[YOLO] Stop sign detected at {self.stopSignDist:.2f}m - stopping")
            else:
                self.stopSignTrigger = 0
        else:
            # Pulse is active - update timer
            self.timer_stop += dt
            
            if self.timer_stop < self.pulseDuration:
                # Still stopping
                self.stopSignTrigger = 1
            elif self.timer_stop < self.pulseDuration + (self.pulseDuration / 2):
                # Cooldown period - detection paused
                self.stopSignTrigger = 0
            else:
                # Reset
                self.timer_stop = 0.0
                self.pulse_active_stop = False
                self.stopSignTrigger = 0
    
    def trafficPulse(self, trafficLight, dt):
        """Time-based traffic light detection with short pulse."""
        trafficLightCount = trafficLight[0]
        trafficLight_clean = trafficLight.copy()
        trafficLight_clean[np.isnan(trafficLight_clean)] = 10.0
        
        if trafficLightCount > 0:
            self.trafficLightDist = trafficLight_clean[1:][trafficLight_clean[1:] != 0].min()
        else:
            self.trafficLightDist = 100.0
            self.trafficTrigger = 0
            return
        
        if not self.pulse_active_traffic:
            # Check for detection in valid range
            if (trafficLightCount > 0 and 
                self.trafficLightDist <= self.trafficThreshold and
                self.trafficLightDist > self.trafficThreshold - 0.6):
                
                self.trafficTrigger = 1
                self.pulse_active_traffic = True
                self.timer_traffic = 0.0
                if self.logger:
                    self.logger.logger.info(f"[YOLO] Red light detected at {self.trafficLightDist:.2f}m - stopping")
            else:
                self.trafficTrigger = 0
        else:
            # Pulse active
            self.timer_traffic += dt
            
            if self.timer_traffic < self.trafficPulseDuration:
                self.trafficTrigger = 1
            else:
                self.timer_traffic = 0.0
                self.pulse_active_traffic = False
                self.trafficTrigger = 0
    
    def yieldPulse(self, yieldSign, dt):
        """Time-based yield sign detection with pulse logic."""
        yieldSignCount = yieldSign[0]
        yieldSign_clean = yieldSign.copy()
        yieldSign_clean[np.isnan(yieldSign_clean)] = 10.0
        
        if yieldSignCount > 0:
            self.yieldDist = yieldSign_clean[1:][yieldSign_clean[1:] != 0].min()
        else:
            self.yieldDist = 100.0
            self.yieldTrigger = 0
            return
        
        if not self.pulse_active_yield:
            if yieldSignCount > 0 and self.yieldDist < self.yieldThreshold:
                self.pulse_active_yield = True
                self.timer_yield = 0.0
                self.yieldTrigger = 1
                if self.logger:
                    self.logger.logger.info(f"[YOLO] Yield sign detected at {self.yieldDist:.2f}m - slowing")
            else:
                self.pulse_active_yield = False
                self.yieldTrigger = 0
        else:
            self.timer_yield += dt
            
            if self.timer_yield < self.pulseDuration:
                self.yieldTrigger = 1
            else:
                self.timer_yield = 0.0
                self.pulse_active_yield = False
                self.yieldTrigger = 0
    
    def carPulse(self, car, current_velocity):
        """
        Car detection with smooth linear deceleration.
        Optionally velocity-aware (scales thresholds with speed).
        """
        carCount = car[0]
        car_clean = car.copy()
        car_clean[np.isnan(car_clean)] = 10.0
        
        if carCount > 0:
            self.carDist = car_clean[1:][car_clean[1:] != 0].min()
        else:
            self.carDist = 100.0
            self.carTrigger = 0
            return
        
        # Velocity-aware threshold adjustment
        if self.velocityAwareThresholds:
            velocity_factor = max(current_velocity / 0.75, 1.0)  # Scale up at higher speeds
            detect_threshold = self.carDetectThreshold * velocity_factor
        else:
            detect_threshold = self.carDetectThreshold
        
        if carCount > 0 and self.carDist < detect_threshold:
            self.carTrigger = 1
            # Linear interpolation from detect_threshold (gain=1) to carThreshold (gain=0)
            m = 1.0 / (detect_threshold - self.carThreshold)
            b = -m * self.carThreshold
            self.vGain_car = np.clip(m * self.carDist + b, 0.0, 1.0)
        else:
            self.carTrigger = 0
    
    def personPulse(self, person, current_velocity):
        """
        Person detection with smooth linear deceleration.
        Optionally velocity-aware (scales thresholds with speed).
        """
        personCount = person[0]
        person_clean = person.copy()
        person_clean[np.isnan(person_clean)] = 10.0
        
        if personCount > 0:
            self.personDist = person_clean[1:][person_clean[1:] != 0].min()
        else:
            self.personDist = 100.0
            self.personTrigger = 0
            return
        
        # Velocity-aware threshold adjustment
        if self.velocityAwareThresholds:
            velocity_factor = max(current_velocity / 0.75, 1.0)
            detect_threshold = self.personDetectThreshold * velocity_factor
        else:
            detect_threshold = self.personDetectThreshold
        
        if personCount > 0 and self.personDist < detect_threshold:
            self.personTrigger = 1
            # Linear interpolation
            m = 1.0 / (detect_threshold - self.personThreshold)
            b = -m * self.personThreshold
            self.vGain_person = np.clip(m * self.personDist + b, 0.0, 1.0)
        else:
            self.personTrigger = 0
