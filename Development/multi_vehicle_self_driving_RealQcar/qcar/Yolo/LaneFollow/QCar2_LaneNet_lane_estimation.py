import numpy as np
import cv2
import time
from pit.LaneNet.nets import LaneNet
from pal.utilities.vision import Camera3D
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
myLaneNet = LaneNet(
    # modelPath = 'path/to/model',
    imageHeight=imageHeight,
    imageWidth=imageWidth,
    rowUpperBound=240,
)

# Initialize the RealSense camera for RGB
# myCamRGB  = Camera3D(mode='RGB', frameWidthRGB=imageWidth, frameHeightRGB=imageHeight)
myCamRGB = QCarRealSense(
    mode="RGB", frameWidthRGB=imageWidth, frameHeightRGB=imageHeight, video3dPort=18805
)

try:
    startTime = time.time()
    while elapsed_time() < simulationTime:
        start = time.time()
        # Read the RGB
        myCamRGB.read_RGB()
        # Crop 40 pixels from bottom and resize back to original dimensions
        cropped_rgb = myCamRGB.imageBufferRGB[:-40, :, :]
        resized_rgb = cv2.resize(cropped_rgb, (imageWidth, imageHeight))
        # Process the image
        rgbProcessed = myLaneNet.pre_process(resized_rgb)
        binaryPred, instancePred = myLaneNet.predict(rgbProcessed)

        # # # Optimized clustering for better lane separation (10-20 FPS)
        # isolatedLane = myLaneNet.post_process(
        #     eps=0.4,  # Lower = stricter clustering
        #     min_samples=20,  # Lower = faster, more sensitive
        #     min_area=50,
        # )  # Remove small noise
        # annotatedImg = myLaneNet.post_process_render(showFPS=True)

        # Fast visualization without lane separation (100 FPS)
        annotatedImg = myLaneNet.render(showFPS=True)

        cv2.imshow("Extracted Lane Markings", annotatedImg)

        # End timing this iteration
        end = time.time()

        # Calculate the computation time, and the time that the thread should pause/sleep for
        computationTime = end - start
        sleepTime = sampleTime - (computationTime % sampleTime)

        # Pause/sleep for sleepTime in milliseconds
        msSleepTime = int(1000 * sleepTime)
        if msSleepTime <= 0:
            msSleepTime = 1
        cv2.waitKey(msSleepTime)

except KeyboardInterrupt:
    print("User interrupted!")

finally:
    myCamRGB.terminate()
