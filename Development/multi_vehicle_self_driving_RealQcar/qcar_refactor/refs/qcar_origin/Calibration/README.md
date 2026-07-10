# QCar Calibration Suite



Start worker: python Calibration/online_calibration_zmq_worker.py
Send 
enable_online_calibration
 command from Ground Station
Drive the vehicle normally (any state)
Data streams automatically via ZMQ
Send SET_PARAMS with category: "online_calibration", action: "analyse", calibration_type: "throttle_velocity" to trigger analysis


Launch the GS GUI (python main_qcar_gui.py or your standard launch script).
Connect to at least one vehicle.
On the right-hand panel corresponding to your QCar, verify the 📐 Calibration pane is available.
Test Passive Controls: Click Collect and Pause; watch the GS terminal or application logs to confirm "Started/Stopped passive calibration collection" commands are dispatched.
Select a calibration mode (e.g., 
throttle_velocity
) and click Analyse. Confirm that an Analysis trigger command is similarly logged.
Test Active Controls: Make sure it is safe for the car to move! Click Trigger under the Active section, and check if the vehicle enters the CALIBRATING state and correctly starts the selected maneuver sequence.

 Clear button to the Passive Calibration control panel right next to "Analyse". Clicking this button will explicitly empty the sample buffer, allowing you to start a fresh data collection run without needing to restart the vehicles or the service!