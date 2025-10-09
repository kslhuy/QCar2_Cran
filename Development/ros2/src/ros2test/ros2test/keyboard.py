#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from qcar2_interfaces.msg import MotorCommands
from sensor_msgs.msg import Imu

import time
import numpy as np
from pynput import keyboard

from pal.utilities.math import Calculus

class KeyboardTest(Node):
    key_states = {
        'z': False,
        's': False,
        'q': False,
        'd': False,
        'p': False
    }

    def on_press(self, key):
        """Handles key press events."""
        try:
            if key.char in self.key_states:
                self.key_states[key.char] = True
        except AttributeError:
            pass  # Ignore special keys

    def on_release(self, key):
        """Handles key release events."""
        try:
            if key.char in self.key_states:
                self.key_states[key.char] = False
        except AttributeError:
            pass  # Ignore special keys

    def __init__(self):
        super().__init__("test_node")
        self.get_logger().info("log in test node")
        self.pub = self.create_publisher(MotorCommands, "/qcar2_motor_speed_cmd", 10)
        #self.pub = self.create_publisher(MotorCommands, "/qcar_1/cmd_vel", 10)
        # self.sub = self.create_subscription(Imu, "/qcar2_imu", self.receive, 10)

            # Timing parameters
        sampleRate = 50
        sampleTime = 1.0 / sampleRate

        startTime = time.time()
        def elapsed_time():
            return time.time() - startTime

        # Initialize QCar2
        # readMode = 0
        # myCar = QCar(readMode=readMode)

        # Optional: differentiator (not strictly used here)
        diff = Calculus().differentiator_variable(sampleTime)
        _ = next(diff)

        # Motor command array: [throttle, steering]
        # QCarCommand = np.array([0.0, 0.0])
        steering = 0.0
        throttle = 0.0
        
        # Constants for gradual change
        THROTTLE_INCREMENT = 0.02  # Speed increase per frame when key held
        STEERING_INCREMENT = 0.03  # Steering increase per frame when key held
        DECAY_RATE = 0.95          # Multiplicative decay when no keys held
        MAX_THROTTLE = 1           # Maximum throttle value
        MAX_STEERING = 0.7         # Maximum steering value

        # Start listening for keyboard input
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

        try:
            # while elapsed_time() < simulationTime:
            while True:
                loopStart = elapsed_time()

                # Handle throttle based on key states
                throttle_input = 0.0
                if self.key_states['z']:
                    throttle_input += THROTTLE_INCREMENT  # Increase forward
                if self.key_states['s']:
                    throttle_input -= THROTTLE_INCREMENT  # Increase reverse
                
                # Apply throttle input and decay
                if throttle_input != 0.0:
                    throttle = np.clip(throttle + throttle_input, -MAX_THROTTLE, MAX_THROTTLE)
                else:
                    throttle *= DECAY_RATE
                    if abs(throttle) < 0.01:
                        throttle = 0.0

                # Handle steering based on key states
                steering_input = 0.0
                if self.key_states['q']:
                    steering_input += STEERING_INCREMENT  # Increase left
                if self.key_states['d']:
                    steering_input -= STEERING_INCREMENT  # Increase right
                
                # Apply steering input and decay
                if steering_input != 0.0:
                    steering = np.clip(steering + steering_input, -MAX_STEERING, MAX_STEERING)
                else:
                    steering *= DECAY_RATE
                    if abs(steering) < 0.01:
                        steering = 0.0

                # Check for quit
                if self.key_states['p']:
                    break

                # Update the command array

                # Prepare LED array (8 LEDs)
                # LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])
                # if steering > 0.3:
                #     LEDs[0] = 1
                #     LEDs[2] = 1
                # elif steering < -0.3:
                #     LEDs[1] = 1
                #     LEDs[3] = 1
                # if throttle < 0:
                #     LEDs[5] = 1

                # Send commands to QCar2

                msg = MotorCommands()
                msg.motor_names.append("steering_angle")
                msg.motor_names.append("motor_throttle")
                msg.values.append(steering)
                msg.values.append(throttle)
                self.pub.publish(msg)
                self.get_logger().info(str(msg))

                # Retrieve battery voltage and speed
                # batteryVoltage = myCar.batteryVoltage
                # linearSpeed = myCar.motorTach

                # Print status to console
                # print(f"\rCar Speed: {linearSpeed:.2f} m/s | "
                #     f"Battery: {100 - (batteryVoltage - 10.5) * 100 / (12.6 - 10.5):.2f}% | "
                #     f"Throttle: {QCarCommand[0]:.2f} | Steering: {QCarCommand[1]:.2f}",
                #     end="", flush=True)

                # Real-time sleep to maintain ~50 Hz
                loopEnd = elapsed_time()
                computationTime = loopEnd - loopStart
                sleepTime = sampleTime - (computationTime % sampleTime)
                if sleepTime > 0:
                    time.sleep(sleepTime)

        except KeyboardInterrupt:
            print("\nUser interrupted!")

        finally:
            # Cleanup
            # myCar.terminate()
            listener.stop()



    def send(self):
        msg = MotorCommands()
        msg.motor_names.append("steering_angle")
        msg.motor_names.append("motor_throttle")
        msg.values.append(-1)
        msg.values.append(3)
        self.pub.publish(msg)
        self.get_logger().info(str(msg))
    
    def receive(self, msg: Imu):
        self.get_logger().info(str(msg))

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTest()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()