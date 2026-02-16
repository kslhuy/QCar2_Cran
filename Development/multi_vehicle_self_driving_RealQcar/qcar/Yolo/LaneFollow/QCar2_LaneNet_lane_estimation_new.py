import numpy as np
import cv2
import time
from scnn_wrapper import SCNNWrapper
from pal.products.qcar import QCarRealSense


## Timing Parameters and methods
def elapsed_time():
    return time.time() - startTime


sampleRate = 30.0
sampleTime = 1 / sampleRate
simulationTime = 1000.0
print("Sample Time: ", sampleTime)

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# Additional parameters
imageWidth = 640
imageHeight = 480

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# Initialize the LaneNet model
# myLaneNet = LaneNet(
#                     # modelPath = 'path/to/model',
#                     imageHeight = imageHeight,
#                     imageWidth = imageWidth,
#                     rowUpperBound = 200
#                     )
myLaneNet = SCNNWrapper(
    weight_path=r"C:\Users\Quang Huy Nugyen\Documents\Quanser\libraries\resources\pretrained_models\resnet18_scnn_tusimple.pt",
    strict_weights=True,
    temporal_alpha=0.6,
)

# Initialize the RealSense camera for RGB
# myCamRGB  = Camera3D(mode='RGB', frameWidthRGB=imageWidth, frameHeightRGB=imageHeight)
myCamRGB = QCarRealSense(
    mode="RGB", frameWidthRGB=imageWidth, frameHeightRGB=imageHeight, video3dPort=18805
)

# Adaptive ROI controls
crop_start = 120
crop_end = imageHeight
min_crop_start = 60
max_crop_start = 180
crop_adjust_period = 5
frame_counter = 0

try:
    startTime = time.time()
    while elapsed_time() < simulationTime:
        start = time.time()
        myCamRGB.read_RGB()
        raw_rgb = myCamRGB.imageBufferRGB

        # Clamp current crop to valid bounds
        crop_start = int(np.clip(crop_start, min_crop_start, max_crop_start))
        cropped_rgb = raw_rgb[crop_start:crop_end, :, :]

        binaryPred, instancePred = myLaneNet.predict(cropped_rgb)
        metrics = myLaneNet.get_metrics()

        # Render SCNN output in the crop, then place it back into full frame.
        annotated_crop = myLaneNet.render(showFPS=True, showMetrics=True)
        annotated_full = raw_rgb.copy()
        annotated_full[crop_start:crop_end, :, :] = annotated_crop
        cv2.rectangle(
            annotated_full,
            (0, crop_start),
            (imageWidth - 1, crop_end - 1),
            (255, 255, 0),
            2,
        )

        # Adjust ROI every few frames using lane occupancy at crop top and bottom.
        frame_counter += 1
        if frame_counter % crop_adjust_period == 0:
            lane_ratio = metrics.get("lane_ratio", 0.0)
            top_ratio = metrics.get("top_lane_ratio", 0.0)
            bottom_ratio = metrics.get("bottom_lane_ratio", 0.0)

            if top_ratio > 0.012 and crop_start > min_crop_start:
                crop_start -= 4
            elif lane_ratio < 0.002 and crop_start > min_crop_start:
                crop_start -= 2
            elif top_ratio < 0.001 and bottom_ratio > 0.01 and crop_start < max_crop_start:
                crop_start += 2

        cv2.putText(
            annotated_full,
            f"ROI top: {crop_start}px",
            (10, imageHeight - 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 0),
            2,
        )
        cv2.putText(
            annotated_full,
            f"Lane ratio: {metrics.get('lane_ratio', 0.0):.3f}",
            (10, imageHeight - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 0),
            2,
        )

        # Convert RGB to BGR for OpenCV display
        displayImg = cv2.cvtColor(annotated_full, cv2.COLOR_RGB2BGR)
        cv2.imshow("Extracted Lane Markings", displayImg)
        cv2.imshow("Lane Binary Mask", binaryPred)

        # End timing this iteration
        end = time.time()

        # Calculate the computation time, and the time that the thread should pause/sleep for
        computationTime = end - start
        sleepTime = sampleTime - (computationTime % sampleTime)

        # Pause/sleep for sleepTime in milliseconds
        msSleepTime = int(1000 * sleepTime)
        if msSleepTime <= 0:
            msSleepTime = 1

        key = cv2.waitKey(msSleepTime) & 0xFF
        if key == ord("s"):
            print("Saving debug images...")
            cv2.imwrite(
                "debug_input_crop.png", cv2.cvtColor(cropped_rgb, cv2.COLOR_RGB2BGR)
            )
            cv2.imwrite("debug_output.png", displayImg)
            cv2.imwrite("debug_binary.png", binaryPred)
            cv2.imwrite(
                "debug_raw.png",
                cv2.cvtColor(myCamRGB.imageBufferRGB, cv2.COLOR_RGB2BGR),
            )
        elif key == ord("r"):
            print("Reset temporal filter and ROI.")
            myLaneNet.reset_temporal_filter()
            crop_start = 120
        elif key == ord("q"):
            print("Quit requested.")
            break

except KeyboardInterrupt:
    print("User interrupted!")

finally:
    cv2.destroyAllWindows()
    myCamRGB.terminate()
