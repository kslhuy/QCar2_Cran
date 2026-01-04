## task_lane_following.py
# This example combines both the left csi and motor commands to
# allow the QCar to follow a yellow lane. Use the joystick to manually drive the QCar
# to a starting position and enable the line follower by holding the X button on the LogitechF710
# To troubleshoot your camera use the hardware_test_csi_camera_single.py found in the hardware tests

from pal.utilities.vision import Camera2D
from pal.products.qcar import QCar
from pal.utilities.math import Filter
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
steeringFilter = Filter().low_pass_first_order_variable(25, 0.033)
next(steeringFilter)
dt = 0.033

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
## Initialize the CSI cameras
myCam = Camera2D(cameraId=cameraID, frameWidth=imageWidth, frameHeight=imageHeight, frameRate=sampleRate)



if not IS_PHYSICAL_QCAR:
    robotsDir = readRobots()

    # THIS VERSION OF VEHICLE CONTROL NEEDS THE CARS TO BE INITIALIZED USING THE MULTIAGENT CLASS
    Car1 = robotsDir["QC2_1"]
    calibrate=False
else:
    calibrate =  'y' in input('do you want to recalibrate?(y/n)')
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
## QCar and Keyboard Control Initialization
myCar = QCar(readMode=1, frequency=100, hilPort=Car1["hilPort"])

# Keyboard control state
throttle_cmd = 0.0
steering_cmd = 0.0
auto_mode_enabled = False  # True = autonomous lane following, False = manual control
target_speed = 0.02  # Default speed for autonomous mode

print("\n=== KEYBOARD CONTROLS ===")
print("MANUAL MODE:")
print("  W/S - Throttle forward/backward")
print("  A/D - Steer left/right")
print("  SPACE - Emergency stop")
print("\nAUTONOMOUS MODE:")
print("  X - Toggle autonomous lane following ON/OFF")
print("  +/- - Increase/decrease autonomous speed")
print("  Q - Quit")
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
		processed = myCam.imageData
		processed[524:674, 0:820,2]=processed[524:674, 0:820,2]+(255-processed[524:674, 0:820,2])*binaryImage
		processed[524:674, 0:820,1]=processed[524:674, 0:820,1]*(1-binaryImage)
		processed[524:674, 0:820,0]=processed[524:674, 0:820,0]*(1-binaryImage)

		# Display the RGB cropped RGB Image and the resized binary image
		cv2.imshow('Detection Overlay', cv2.resize(processed, (820,410)) )

		# Find slope and intercept of linear fit from the binary image
		slope, intercept = ImageProcessing.find_slope_intercept_from_binary(binary=binaryImage)

		# steering from slope and intercept
		rawSteering = 1.5*(slope - 0.3419) + (1/150)*(intercept+5)
		steering = steeringFilter.send((np.clip(rawSteering, -0.5, 0.5), dt))

		# Keyboard control - read key press (1ms wait)
		key = cv2.waitKey(1) & 0xFF
		
		# Manual controls (W/A/S/D)
		if key == ord('w'):
			throttle_cmd = min(throttle_cmd + 0.02, 0.3)
		elif key == ord('s'):
			throttle_cmd = max(throttle_cmd - 0.02, -0.3)
		elif key == ord('a'):
			steering_cmd = max(steering_cmd + 0.05, 0.5)
		elif key == ord('d'):
			steering_cmd = min(steering_cmd - 0.05, -0.5)
		
		# Autonomous mode toggle
		elif key == ord('x'):
			auto_mode_enabled = not auto_mode_enabled
			if auto_mode_enabled:
				print(f"🚗 AUTONOMOUS MODE ON - Speed: {target_speed:.2f}")
			else:
				print("🎮 MANUAL MODE ON")
				throttle_cmd = 0.0
				steering_cmd = 0.0
		
		# Speed adjustment for autonomous mode
		elif key == ord('=') or key == ord('+'):
			target_speed = min(target_speed + 0.02, 0.3)
			print(f"Target speed: {target_speed:.2f}")
		elif key == ord('-') or key == ord('_'):
			target_speed = max(target_speed - 0.02, 0.05)
			print(f"Target speed: {target_speed:.2f}")
		
		# Emergency stop
		elif key == ord(' '):
			throttle_cmd = 0.0
			steering_cmd = 0.0
			auto_mode_enabled = False
			print("⛔ EMERGENCY STOP")
		
		# Quit
		elif key == ord('q'):
			print("Quitting...")
			break
		
		# Determine final commands based on mode
		if auto_mode_enabled:
			# Autonomous lane following
			if not math.isnan(steering):
				final_steering = steering
				# Reduce speed when turning sharply
				speed_factor = np.cos(steering)
				final_throttle = target_speed * max(0.5, speed_factor)
			else:
				# No lane detected - stop
				final_steering = 0.0
				final_throttle = 0.0
			LEDs = np.array([1, 1, 0, 0, 0, 0, 0, 0])  # Front LEDs = autonomous
		else:
			# Manual control
			final_steering = steering_cmd
			final_throttle = throttle_cmd
			LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])  # Rear LEDs = manual
		
		# Write commands to QCar
		myCar.read_write_std(final_throttle, final_steering, LEDs)
		end = time.time()
		dt = end - start

except KeyboardInterrupt:
	print("User interrupted!")
		
finally:
	# Terminate camera and QCar
	myCam.terminate()
	myCar.terminate()
	# gpad.terminate()
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
