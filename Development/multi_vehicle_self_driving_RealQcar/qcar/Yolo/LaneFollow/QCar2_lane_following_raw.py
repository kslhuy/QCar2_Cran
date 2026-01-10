## task_lane_following.py
# This example combines both the left csi and motor commands to
# allow the QCar to follow a yellow lane. Use the joystick to manually drive the QCar
# to a starting position and enable the line follower by holding the X button on the LogitechF710
# To troubleshoot your camera use the hardware_test_csi_camera_single.py found in the hardware tests

from pal.utilities.vision import Camera2D
from pal.products.qcar import QCar,QCarCameras
from pal.utilities.math import Filter
from pal.utilities.probe import Probe
from hal.utilities.image_processing import ImageProcessing
from qvl.multi_agent import readRobots
from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR

import time
import numpy as np
import cv2
import math

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
## Timing Parameters and methods
sampleRate = 60
sampleTime = 1/sampleRate
print('Sample Time: ', sampleTime)

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# Additional parameters
imageWidth  = 1640
imageHeight = 820
cameraID 	= "2@tcpip://localhost:18813"

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
#Setting Filter
steeringFilter = Filter().low_pass_first_order_variable(25, 0.1)
next(steeringFilter)
dt = 0.033

if not IS_PHYSICAL_QCAR:
    robotsDir = readRobots()

    # THIS VERSION OF VEHICLE CONTROL NEEDS THE CARS TO BE INITIALIZED USING THE MULTIAGENT CLASS
    Car1 = robotsDir["QC2_1"]
    calibrate=False
else:
    calibrate =  'y' in input('do you want to recalibrate?(y/n)')

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
## Initialize the CSI cameras
myCam = Camera2D(cameraId=cameraID, frameWidth=imageWidth, frameHeight=imageHeight, frameRate=sampleRate)
# myCam = QCarCameras( frameWidth=imageWidth, frameHeight=imageHeight , frameRate=sampleRate, enableFront=True, videoPort=Car1["videoPort"])

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
## QCar Initialization
myCar = QCar(readMode=1, frequency=100, hilPort=Car1["hilPort"])

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
## Keyboard Control State Variables
throttle = 0.0
manual_steering = 0.0
lane_follow_enabled = False

print("\n=== Keyboard Controls ===")
print("HOLD W/S: Throttle forward/backward (continuous)")
print("HOLD A/D: Manual steering left/right (continuous)")
print("X: Toggle lane following / assist mode")
print("Spacebar: Emergency stop")
print("ESC or Q: Quit")
print("========================\n")


# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
## Main Loop
try:
	while True:
		start = time.time()
		# Capture RGB Image from CSI
		myCam.read()
		# Crop out a piece of the RGB to improve performance
		croppedRGB = myCam.imageData[524:674, 0:820]

		# Convert to HSV and then threshold it for yellow
		hsvBuf = cv2.cvtColor(croppedRGB, cv2.COLOR_BGR2HSV)

		binaryImage = ImageProcessing.binary_thresholding(frame= hsvBuf,
													lowerBounds=np.array([10, 50, 100]),
													upperBounds=np.array([45, 255, 255]))


		# Overlay detected yellow lane over raw RGB image
		binaryImage=binaryImage/255
		processed = myCam.imageData.copy()
		processed[524:674, 0:820,2]=processed[524:674, 0:820,2]+(255-processed[524:674, 0:820,2])*binaryImage
		processed[524:674, 0:820,1]=processed[524:674, 0:820,1]*(1-binaryImage)
		processed[524:674, 0:820,0]=processed[524:674, 0:820,0]*(1-binaryImage)
		
		# Draw detection region rectangle (green box)
		cv2.rectangle(processed, (0, 524), (820, 674), (0, 255, 0), 3)
		cv2.putText(processed, "Detection Zone", (10, 510), 
					cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

		# Find slope and intercept of linear fit from the binary image
		slope, intercept = ImageProcessing.find_slope_intercept_from_binary(binary=binaryImage)

		# steering from slope and intercept
		rawSteering = 1.5*(slope - 0.3419) + (1/150)*(intercept+5)
		steering = steeringFilter.send((np.clip(rawSteering, -0.5, 0.5), dt))

		# Keyboard input handling - CONTINUOUS (hold-based)
		key = cv2.waitKey(1) & 0xFF
		
		# Continuous throttle control (hold to apply)
		if key == ord('w'):
			throttle = 0.08  # Full forward when holding W
		elif key == ord('s'):
			throttle = -0.08  # Full backward when holding S
		else:
			# Decay throttle when not pressing W/S
			throttle *= 0.8  # Gradual decay
			if abs(throttle) < 0.01:
				throttle = 0.0
		
		# Continuous steering control (hold to apply)
		if key == ord('a'):
			manual_steering = 0.4  # Steer left when holding A
		elif key == ord('d'):
			manual_steering = -0.4  # Steer right when holding D
		else:
			# Decay steering when not pressing A/D
			manual_steering *= 0.5  # Quick decay for responsive steering
			if abs(manual_steering) < 0.02:
				manual_steering = 0.0
		
		# Toggle lane following
		if key == ord('x'):
			lane_follow_enabled = not lane_follow_enabled
			print(f"Lane following: {'ENABLED' if lane_follow_enabled else 'DISABLED'}")
		
		# Quit
		if key == 27 or key == ord('q'):  # ESC or Q to quit
			print("Quitting...")
			break
		
		# Spacebar to emergency stop
		if key == ord(' '):
			throttle = 0.0
			manual_steering = 0.0

		# Build QCar command - Assist mode with manual influence
		if lane_follow_enabled:
			# Lane following with manual steering assist
			QCarCommand = np.array([0.05 + throttle , 0.0])
			if math.isnan(steering):
				QCarCommand[1] = manual_steering  # Fall back to manual if no lane detected
			else:
				# Blend lane steering with manual input (70% lane, 30% manual)
				QCarCommand[1] =  steering +  manual_steering
			QCarCommand[0] = QCarCommand[0] * np.cos(QCarCommand[1])
		else:
			# Pure manual control
			QCarCommand = np.array([throttle, manual_steering])
		
		# Add telemetry overlay on the processed image
		y_offset = 30
		line_height = 35
		font = cv2.FONT_HERSHEY_SIMPLEX
		font_scale = 0.7
		font_thickness = 2
		
		# Mode indicator with color coding
		mode_color = (0, 255, 0) if lane_follow_enabled else (0, 165, 255)  # Green if enabled, orange if manual
		mode_text = "LANE FOLLOW" if lane_follow_enabled else "MANUAL"
		cv2.putText(processed, f"Mode: {mode_text}", (850, y_offset), 
					font, font_scale, mode_color, font_thickness)
		
		# Steering values - Always show lane detection
		cv2.putText(processed, f"Lane Steering: {steering:.3f}", (850, y_offset + line_height), 
					font, font_scale, (255, 255, 255), font_thickness)
		cv2.putText(processed, f"Manual Input: {manual_steering:.3f}", (850, y_offset + 2*line_height), 
					font, font_scale, (200, 150, 255), font_thickness)
		
		# Command values
		cv2.putText(processed, f"Throttle: {QCarCommand[0]:.3f}", (850, y_offset + 3*line_height), 
					font, font_scale, (100, 200, 255), font_thickness)
		cv2.putText(processed, f"Cmd Steering: {QCarCommand[1]:.3f}", (850, y_offset + 4*line_height), 
					font, font_scale, (100, 200, 255), font_thickness)
		
		# Lane detection parameters
		cv2.putText(processed, f"Slope: {slope:.3f}", (850, y_offset + 5*line_height), 
					font, font_scale, (200, 200, 200), font_thickness)
		cv2.putText(processed, f"Intercept: {intercept:.1f}", (850, y_offset + 6*line_height), 
					font, font_scale, (200, 200, 200), font_thickness)
		
		# FPS counter
		fps = 1.0 / dt if dt > 0 else 0
		cv2.putText(processed, f"FPS: {fps:.1f}", (850, y_offset + 7*line_height), 
					font, font_scale, (255, 255, 0), font_thickness)
		
		# Display the processed image with all overlays
		cv2.imshow('Detection Overlay', cv2.resize(processed, (820, 410)))
		
		LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])
		myCar.read_write_std(QCarCommand[0], QCarCommand[1])
		end = time.time()
		dt = end - start

except KeyboardInterrupt:
	print("User interrupted!")
		
finally:
	# Terminate camera and QCar
	myCam.terminate()
	myCar.terminate()
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
