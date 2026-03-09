import cv2
import zmq
import numpy as np

# REPLACE THIS WITH YOUR LIMO's IP ADDRESS
LIMO_IP = "192.168.137.175"
PORT = "18760"

def main():
    # Setup ZMQ Subscriber
    ctx = zmq.Context.instance()
    socket = ctx.socket(zmq.SUB)
    socket.setsockopt(zmq.SUBSCRIBE, b"")
    socket.setsockopt(zmq.CONFLATE, 1)  # Keep only the latest frame to avoid lag
    
    uri = f"tcp://{LIMO_IP}:{PORT}"
    socket.connect(uri)
    print(f"Connecting to YOLO Probe stream at {uri}...")

    try:
        while True:
            # Receive data
            data = socket.recv()
            
            # Decode JPEG buffer to OpenCV Image
            frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
            
            if frame is not None:
                cv2.imshow("YOLO Ground Station Probe", frame)
                
            # Press 'q' to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        socket.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
