import numpy as np
from quanser.common import Timeout
from pal.utilities.stream import BasicStream

class YOLOReceiver():
    def __init__(self,ip='localhost',nonBlocking=True,port="18666"):
        self.stopSign = np.zeros((7),dtype=np.float64)
        self.trafficlight = np.zeros((7),dtype=np.float64)
        self.cars = np.zeros((7),dtype=np.float64)
        self.yieldSign = np.zeros((7),dtype=np.float64)
        self.person = np.zeros((7),dtype=np.float64)
        self.lane = np.zeros((7),dtype=np.float64)  # Lane detection data: [confidence, steering, slope, intercept, 0, 0, 0]

        self.uri='tcpip://'+ip+':'+port
        self._timeout = Timeout(seconds=0, nanoseconds=100000)
        self._handle = BasicStream(uri=self.uri,
                                    agent='C',
                                    receiveBuffer=np.zeros((6,7),  # Expanded to 6 rows for lane data
                                                           dtype=np.float64),
                                    recvBufferSize=7*6*8,  # Updated buffer size
                                    nonBlocking=nonBlocking,
                                    reshapeOrder='C')
        self.status_check('', iterations=20)

    def status_check(self, message, iterations=10):
        # blocking method to establish connection to the server stream.
        self._timeout = Timeout(seconds=0, nanoseconds=10000) #1000000
        counter = 0
        while not self._handle.connected:
            self._handle.checkConnection(timeout=self._timeout)
            counter += 1
            if self._handle.connected:
                print(message)
                break
            elif counter >= iterations:
                print('YOLO Server error: status check failed.')
                break

    def read(self):
        new = False
        self._timeout = Timeout(seconds=0, nanoseconds=10)
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
            self.status_check('Reconnected to yolo Server',iterations=1)
        return new

    def terminate(self):
        self._handle.terminate()
    
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
                                    reshapeOrder='F')
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
    """
    
    def __init__(self, logger=None):
        self.logger = logger
        self.yolo = None
        self.yolo_drive = None
        self.yolo_gain = 1.0
        self.loop_counter = 0
        self.yolo_enabled = False
        
    def initialize(self, yolo_receiver: 'YOLOReceiver', yolo_drive_logic: 'YOLODriveLogic'):
        """Initialize YOLO components"""
        if yolo_receiver is None or yolo_drive_logic is None:
            self.yolo_enabled = False  # Disable YOLO if components are missing
        else:
            self.yolo_enabled = True   # Enable YOLO only when both components are provided
        self.yolo = yolo_receiver
        self.yolo_drive = yolo_drive_logic
        
    def update_yolo_data(self, loop_counter: int = 0):
        """Update YOLO detection data and return velocity gain"""
        self.loop_counter = loop_counter
        try:
            if self.yolo is not None:
                self.yolo.read()
                
                # Process YOLO and get velocity gain
                if self.yolo_drive is not None:
                    try:
                        self.yolo_gain = self.yolo_drive.check_yolo(
                            self.yolo.stopSign,
                            self.yolo.trafficlight,
                            self.yolo.cars,
                            self.yolo.yieldSign,
                            self.yolo.person
                        )
                    except Exception as e:
                        if self.loop_counter % 100 == 0:  # Log occasionally
                            if self.logger:
                                self.logger.log_error("YOLO drive error", e)
                        self.yolo_gain = 1.0  # Default gain
                else:
                    self.yolo_gain = 1.0
            else:
                if self.loop_counter % 1000 == 0:  # Log occasionally
                    if self.logger:
                        self.logger.log_error("YOLO drive is None")
                self.yolo_gain = 1.0
                
        except Exception as e:
            if self.logger:
                self.logger.log_error("YOLO data update error", e)
            self.yolo_gain = 1.0
            
        return self.yolo_gain
    
    def get_yolo_data(self) -> dict:
        """Get current YOLO detection data"""
        try:
            if self.yolo is not None:
                # Extract lane data: [confidence, steering, slope, intercept, ...]
                lane_confidence = self.yolo.lane[0] if len(self.yolo.lane) > 0 else 0.0
                lane_steering = self.yolo.lane[1] if len(self.yolo.lane) > 1 else 0.0
                lane_slope = self.yolo.lane[2] if len(self.yolo.lane) > 2 else 0.0
                lane_intercept = self.yolo.lane[3] if len(self.yolo.lane) > 3 else 0.0
                
                return {
                    'stop_sign': self.yolo.stopSign,
                    'traffic_light': self.yolo.trafficlight,
                    'cars': self.yolo.cars,
                    'yield_sign': self.yolo.yieldSign,
                    'person': self.yolo.person,
                    'car_dist': getattr(self.yolo_drive, 'carDist', 0.0),
                    'person_dist': getattr(self.yolo_drive, 'personDist', 0.0),
                    'lane_confidence': lane_confidence,
                    'lane_steering': lane_steering,
                    'lane_slope': lane_slope,
                    'lane_intercept': lane_intercept
                }
            else:
                return self.get_default_yolo_data()
        except Exception as e:
            if self.logger:
                self.logger.log_error("YOLO data retrieval error", e)
            return self.get_default_yolo_data()
    
    def get_default_yolo_data(self) -> dict:
        """Get default YOLO data when YOLO is not available"""
        return {
            'stop_sign': [0]*7, 'traffic_light': [0]*7, 'cars': [0]*7,
            'yield_sign': [0]*7, 'person': [0]*7, 'car_dist': None, 'person_dist': None,
            'lane_confidence': 0.0, 'lane_steering': 0.0, 'lane_slope': 0.0, 'lane_intercept': 0.0
        }
        
    def get_yolo_gain(self) -> float:
        """Get current YOLO velocity gain"""
        return self.yolo_gain
        
    def is_yolo_active(self) -> bool:
        """Check if YOLO components are active"""
        return self.yolo is not None and self.yolo_drive is not None
    
    def disable(self):
        """Disable YOLO system and clean up resources"""
        if self.logger:
            self.logger.logger.info("[YOLO] Disabling YOLO system...")
        
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
        self.yolo_gain = 1.0
        
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
