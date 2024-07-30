from multiprocessing import Queue
import cv2
import time

class CameraStreamer:
    def __init__(self, ip, queue):
        self.ip = ip
        self.queue = queue
        self.pipeline = (
            f"rtspsrc location={self.ip} latency=0 buffer-mode=auto ! "
            "rtph264depay ! avdec_h264 ! videoconvert ! appsink"
        )
        self.cap = None

    def stream(self):
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            print(f"Failed to open stream: {self.ip}")
            return

        while True:
            ret, img = self.cap.read()
            if not ret:
                print(f"Failed to read frame from stream: {self.ip}")
                break

            # Put the frame in the queue
            self.queue.put(img)

            time.sleep(0.05)

        self.cap.release()

if __name__ == "__main__":
    queue = Queue()
    ip = "rtsp://ip:port/video0_unicast"
    
    streamer = CameraStreamer(ip, queue)
    streamer.stream()
