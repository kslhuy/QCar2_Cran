import numpy as np
import time
import cv2
# from pit.YOLO.utils import QCar2DepthAligned  # Replaced with custom alignment class
from QCar2DepthAlignedCamera import QCar2DepthAlignedCamera
from pal.utilities.probe import Probe
from pit.YOLO.nets import YOLOv8
from qvl.multi_agent import readRobots
from hal.utilities.image_processing import ImageProcessing

from YoLo import YOLOPublisher
import argparse

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# Collect command line arguments
parser = argparse.ArgumentParser(prog='Vehicle control')
parser.add_argument('-i','--ip_host', default='localhost')
parser.add_argument('-p','--probing', default="False")
parser.add_argument('-w','--width', default=320, help="wide of to image to be displayed in the observer")
parser.add_argument('-ht','--height', default=200, help="height of to image to be displayed in the observer")
parser.add_argument('-idx','--caridx', type=int, default=0, help="Car ID for port assignment")
parser.add_argument('-s','--show-image', action='store_true', help="Show annotated image directly in a window")
args = parser.parse_args()
ipHost = args.ip_host
probing = args.probing=="True"
width = int(args.width)
height = int(args.height)
car_id = args.caridx

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# Instantiate YOLOv8 model
imageWidth  = 640
imageHeight = 480
myYolo  = YOLOv8(
                 # modelPath = 'path/to/model', 
                 imageHeight= imageHeight,
                 imageWidth = imageWidth,
                )

# Initialize Depth/RGB alignment RT model, YOLO server, and probe
# Use different ports for each car to avoid conflicts
# camera_port = f'1877{car_id}'  # Car 0: 18770, Car 1: 18771, etc.
yolo_port = f'1866{car_id}'    # Car 0: 18660, Car 1: 18661, etc.

robotsDir = readRobots()
name = f"QC2_{car_id}"
car_config = robotsDir[name]

# Initialize the custom depth-aligned camera
QCarImg = QCar2DepthAlignedCamera(
    imageWidth=imageWidth,
    imageHeight=imageHeight,
    use_intrinsics=True,  # Set to False to use simple mode
    clipping_distance=10.0,
    video3dPort=car_config['video3dPort'],
    load_settings=True,  # Load saved alignment settings if available
    use_fast_alignment=True  # Use fast alignment method
)

YOLOserver = YOLOPublisher(port=yolo_port)


## for virtual car , use cv2.imshow better 
##
def create_probe(ip_host, car_id, height, width):
    """Create and initialize a new probe instance for reconnection support."""
    new_probe = Probe(ip=ip_host)
    # Use car_id as display ID to avoid port conflicts
    # Car 0 → display ID 0 → port 18851
    # Car 1 → display ID 1 → port 18852, etc.
    new_probe.numDisplays = car_id + 50  # Set the counter to car_id
    new_probe.add_display(imageSize=[height, width, 3], name=f'YOLO Car {car_id}', scalingFactor=1)
    print(f"[PROBE] Initialized probe for Car {car_id} on port {18851 + car_id}")
    return new_probe

if probing:
    probe = create_probe(ipHost, car_id, height, width)
else:
    probe = None
    
probe_count = 0
probe_reconnect_counter = 0
probe_reconnect_interval = 100  # Try to reconnect every 100 frames if disconnected
probe_frame_skip = 4  # Send every 4th frame to reduce load (adjustable: 2-10)
last_successful_send = time.time()
send_failure_count = 0
consecutive_send_failures = 0  # Track consecutive send failures to detect dead connection
last_reconnect_attempt = 0  # Timestamp of last reconnection attempt
reconnect_cooldown = 2.0  # Seconds to wait between reconnection attempts
CONNECTION_DEAD_THRESHOLD = 5  # Number of consecutive failures before considering connection dead

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# Lane detection helper function
def detect_lane(rgb_image):
    """
    Extract lane parameters from RGB image using HSV thresholding.
    Returns: (steering_correction, confidence, slope, intercept)
    """
    try:
        # Crop region of interest - bottom portion for lane detection
        height = rgb_image.shape[0]
        cropped_rgb = rgb_image[int(height*0.6):, :, :]  # Bottom 40% of image
        
        # Convert to HSV and threshold for yellow lane
        hsv_buf = cv2.cvtColor(cropped_rgb, cv2.COLOR_BGR2HSV)
        binary_image = ImageProcessing.binary_thresholding(
            frame=hsv_buf,
            lowerBounds=np.array([10, 50, 100]),
            upperBounds=np.array([45, 255, 255])
        )
        
        # Calculate slope and intercept from binary image
        slope, intercept = ImageProcessing.find_slope_intercept_from_binary(binary=binary_image)
        
        # Check if valid lane detected
        if np.isnan(slope) or np.isnan(intercept):
            return 0.0, 0.0, 0.0, 0.0
        
        # Calculate steering correction (same formula as lane following script)
        raw_steering = 1.5 * (slope - 0.3419) + (1/150) * (intercept + 5)
        steering_correction = np.clip(raw_steering, -0.5, 0.5)
        
        # Lane confidence based on detected pixels
        lane_confidence = np.sum(binary_image) / binary_image.size
        
        return steering_correction, lane_confidence, slope, intercept
        
    except Exception as e:
        # If lane detection fails, return zeros
        return 0.0, 0.0, 0.0, 0.0

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# Main loop for YOLO server
try:
    while True:
        # Instantiate YOLO prediction send buffers
        stopSignBuffers = np.zeros((7),dtype=np.float64)
        trafficBuffers = np.zeros((7),dtype=np.float64)
        carBuffer = np.zeros((7),dtype=np.float64)
        yieldBuffer = np.zeros((7),dtype=np.float64)
        personBuffer = np.zeros((7),dtype=np.float64)
        laneBuffer = np.zeros((7),dtype=np.float64)  # Lane detection buffer
        
        # Get aligned RGB and Depth images
        QCarImg.read()
        
        # Crop 40 pixels from bottom and resize back to original dimensions
        cropped_rgb = QCarImg.rgb[:-40, :, :]
        cropped_depth = QCarImg.depth[:-40, :]
        resized_rgb = cv2.resize(cropped_rgb, (imageWidth, imageHeight))
        resized_depth = cv2.resize(cropped_depth, (imageWidth, imageHeight))
            
        # Get YOLO predictions and post-process results
        rgbProcessed = myYolo.pre_process(resized_rgb)
        predecion = myYolo.predict(inputImg = rgbProcessed,
                                   classes = [0,2,9,11,33],
                                   confidence = 0.4,
                                   half = True,
                                   verbose = False
                                   )
        processedResults=myYolo.post_processing(alignedDepth = resized_depth,
                                                clippingDistance = 10)
        annotatedImg=myYolo.post_process_render(showFPS = True)
        
        # Show image directly if option is enabled
        if args.show_image:
            cv2.imshow('YOLO Server', annotatedImg)
            key = cv2.waitKey(1)
            if key == ord('q'):
                break
        
        # Resize the annotated image and send to observer if probing is enabled
        if probing:
            probe_reconnect_counter += 1
            current_time = time.time()
            
            # Check if connection appears dead (too many consecutive send failures or long time since success)
            connection_appears_dead = (
                consecutive_send_failures >= CONNECTION_DEAD_THRESHOLD or
                (current_time - last_successful_send > 5.0 and consecutive_send_failures > 0)
            )
            
            # Attempt reconnection if connection is dead
            if connection_appears_dead and (current_time - last_reconnect_attempt) > reconnect_cooldown:
                last_reconnect_attempt = current_time
                print(f"[PROBE] Connection dead for Car {car_id} (failures: {consecutive_send_failures}) - attempting reconnection...")
                try:
                    # Terminate old probe gracefully
                    try:
                        probe.terminate()
                    except Exception as e:
                        pass  # Ignore termination errors on dead connection
                    
                    # Small delay to allow port to be released
                    time.sleep(0.3)
                    
                    # Create new probe instance
                    probe = create_probe(ipHost, car_id, height, width)
                    consecutive_send_failures = 0
                    send_failure_count = 0
                    print(f"[PROBE] New probe created for Car {car_id} - waiting for observer to connect...")
                except Exception as e:
                    print(f"[PROBE] Reconnection failed for Car {car_id}: {e}")
            
            # Adaptive frame skipping based on send success rate
            if send_failure_count > 10 and probe_frame_skip < 8:
                probe_frame_skip = min(8, probe_frame_skip + 1)
                send_failure_count = 0
            elif current_time - last_successful_send < 1.0 and probe_frame_skip > 3:
                probe_frame_skip = max(3, probe_frame_skip - 1)
            
            # Only send data on skipped frames
            if probe_count % probe_frame_skip == 0:
                # Try to connect/check connection
                probe.check_connection()
                
                if probe.connected:
                    try:
                        # Compress image to JPEG to reduce bandwidth
                        _, buffer = cv2.imencode('.jpg', annotatedImg, [cv2.IMWRITE_JPEG_QUALITY, 85])
                        compressed_img = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
                        resizedImg = cv2.resize(compressed_img, (width, height))
                        
                        success = probe.send(name=f'YOLO Car {car_id}', imageData=resizedImg)
                        # Note: success can be True, False, or None (if not connected)
                        if success is True:
                            last_successful_send = current_time
                            consecutive_send_failures = 0
                            send_failure_count = max(0, send_failure_count - 1)
                            if probe_reconnect_counter % 500 == 0:
                                print(f"[PROBE] Car {car_id} streaming OK")
                        else:
                            # Either False (send failed) or None (not connected)
                            consecutive_send_failures += 1
                            send_failure_count += 1
                            print(f"[PROBE] Send failed for Car {car_id} (failure #{consecutive_send_failures}, success={success})")
                    except Exception as e:
                        consecutive_send_failures += 1
                        send_failure_count += 1
                        if probe_reconnect_counter % 100 == 0:
                            print(f"[PROBE] Error sending to Car {car_id}: {e}")
                else:
                    # Not connected yet - this is normal when waiting for observer
                    if probe_reconnect_counter % 200 == 0:
                        print(f"[PROBE] Car {car_id} waiting for observer connection...")
        
        probe_count += 1

        # process the prediction results and send to vehicle control server
        stopSignCount=0
        trafficCount=0
        carCount=0
        yieldCount=0
        personCount = 0
        if len(processedResults)>0:
            for i in processedResults:
                # print(i.name)
                if 'car' in i.name:
                    carBuffer[carCount+1]=i.distance
                    carCount+=1
                elif 'stop sign' in i.name:
                    stopSignBuffers[stopSignCount+1]=i.distance
                    stopSignCount+=1
                elif 'red' in i.name:
                    trafficBuffers[trafficCount+1]=i.distance
                    trafficCount+=1
                elif 'yield' in i.name:
                    yieldBuffer[yieldCount+1]=i.distance
                    yieldCount+=1
                elif 'person' in i.name:
                    if personCount <= 5:
                        personBuffer[personCount+1]=i.distance
                    personCount+=1
        carBuffer[0] = carCount
        trafficBuffers[0] = trafficCount
        stopSignBuffers[0] = stopSignCount
        yieldBuffer[0] = yieldCount
        personBuffer[0] = personCount
        
        # Perform lane detection on the same RGB image
        lane_steering, lane_confidence, lane_slope, lane_intercept = detect_lane(resized_rgb)
        laneBuffer[0] = lane_confidence
        laneBuffer[1] = lane_steering
        laneBuffer[2] = lane_slope
        laneBuffer[3] = lane_intercept
        # laneBuffer[4:7] remain zeros (reserved for future use)
        
        # Send all detection data including lane (6 rows total)
        sendPacket = np.vstack((stopSignBuffers, trafficBuffers, carBuffer, yieldBuffer, personBuffer, laneBuffer))
        YOLOserver.send(sendPacket)

except KeyboardInterrupt:
    print("User interrupted!")
    
finally:
    QCarImg.terminate()
    if probing:
        probe.terminate()

