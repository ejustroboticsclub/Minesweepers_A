import cv2
from multiprocessing import Queue, Process
from streamer import CameraStreamer

class DisplayNode:
    def __init__(self, queue):
        self.queue = queue

    def display(self):
        while True:
            if not self.queue.empty():
                img = self.queue.get()
                cv2.imshow("Stream", img)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        cv2.destroyAllWindows()

if __name__ == "__main__":
    queue = Queue()
    ip = "rtsp://ip:port/video0_unicast"

    streamer = CameraStreamer(ip, queue)
    display_node = DisplayNode(queue)

    streamer_process = Process(target=streamer.stream)
    display_process = Process(target=display_node.display)

    streamer_process.start()
    display_process.start()

    streamer_process.join()
    display_process.join()
