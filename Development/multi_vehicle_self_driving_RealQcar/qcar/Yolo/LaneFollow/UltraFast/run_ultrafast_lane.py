"""
Run Ultra-Fast-Lane-Detection-v2 on QCar RealSense Camera

Standalone demo script for real-time lane detection using the QCar's
RealSense camera. Modelled after QCar2_LaneNet_lane_estimation.py.

Usage:
    python run_ultrafast_lane.py

Controls:
    - Press 'q' or Ctrl+C to quit
"""

import time
import cv2
from pal.products.qcar import QCarRealSense
from ultrafast_wrapper import UltraFastV2Wrapper

# ============================================================================
#  Configuration
# ============================================================================

# Camera parameters
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

# Model configuration
MODEL_PATH = r"C:\Users\Quang Huy Nugyen\Documents\Quanser\libraries\resources\pretrained_models\curvelanes_res18.pth"
MODEL_TYPE = "curvelanes_res18"  # Options: culane_res18, culane_res34, tusimple_res18, tusimple_res34, curvelanes_res18, curvelanes_res34

# Timing
SAMPLE_RATE = 30.0
SAMPLE_TIME = 1.0 / SAMPLE_RATE
SIMULATION_TIME = 1000.0  # seconds

# Crop bottom pixels (same as LaneNet script — removes car hood)
CROP_BOTTOM_PX = 40

# ============================================================================
#  Main
# ============================================================================


def elapsed_time():
    return time.time() - startTime


if __name__ == "__main__":
    print(f"[UltraFast Demo] Initializing...")
    print(f"  Model:  {MODEL_TYPE}")
    print(f"  Weight: {MODEL_PATH}")
    print(f"  Camera: {IMAGE_WIDTH}x{IMAGE_HEIGHT} @ {SAMPLE_RATE} Hz")

    # Initialize model
    model = UltraFastV2Wrapper(
        model_path=MODEL_PATH,
        model_type=MODEL_TYPE,
    )

    # Initialize the RealSense camera for RGB
    myCamRGB = QCarRealSense(
        mode="RGB",
        frameWidthRGB=IMAGE_WIDTH,
        frameHeightRGB=IMAGE_HEIGHT,
        video3dPort=18805,
    )

    try:
        startTime = time.time()

        while elapsed_time() < SIMULATION_TIME:
            start = time.time()

            # Read the RGB frame
            myCamRGB.read_RGB()

            # Crop bottom pixels and resize back to original dimensions
            cropped_rgb = myCamRGB.imageBufferRGB[:-CROP_BOTTOM_PX, :, :]
            resized_rgb = cv2.resize(cropped_rgb, (IMAGE_WIDTH, IMAGE_HEIGHT))

            # Run lane detection (expects BGR, QCarRealSense returns RGB)
            bgr_frame = cv2.cvtColor(resized_rgb, cv2.COLOR_RGB2BGR)
            coords = model.predict(bgr_frame)

            # Render annotated image
            annotated = model.render(showFPS=True)

            cv2.imshow("UltraFast Lane Detection v2", annotated)

            # Timing control
            end = time.time()
            computationTime = end - start
            sleepTime = SAMPLE_TIME - (computationTime % SAMPLE_TIME)
            msSleepTime = int(1000 * sleepTime)
            if msSleepTime <= 0:
                msSleepTime = 1

            key = cv2.waitKey(msSleepTime)
            if key == ord("q"):
                print("[UltraFast Demo] Quit requested.")
                break

    except KeyboardInterrupt:
        print("[UltraFast Demo] User interrupted!")

    finally:
        myCamRGB.terminate()
        cv2.destroyAllWindows()
        print("[UltraFast Demo] Cleaned up.")
