import cv2
import socket
import struct
import numpy as np
from ultralytics import YOLO
from PIL import Image


class YOLOProcessor:
    def __init__(self, model_path):
        self.yolo_model = YOLO(model_path)  # Load a pre-trained YOLOv8 model

    def process_frame(self, frame):
        results = self.yolo_model(frame)
        detections = results[0].boxes.xyxy.cpu().numpy()
        confidences = results[0].boxes.conf.cpu().numpy()
        return detections, confidences

    @staticmethod
    def draw_boxes(image, detections, confidences, color):
        for (x1, y1, x2, y2), conf in zip(detections, confidences):
            cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), color, 1)
            cv2.putText(
                image,
                f"Confidence: {conf:.2f}",
                (int(x1), int(y1) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
            )
        return image


class FrameProcessor:
    def __init__(self, y_line=300):
        self.y_line = y_line

    def flip_vertical(self, frame):
        return cv2.flip(frame, 0)

    def draw_horizontal_line(self, frame):
        height, width, _ = frame.shape
        self.y_line = min(
            max(0, self.y_line), height - 1
        )  # Ensure y_line is within boundaries
        cv2.line(
            frame, (0, self.y_line), (width, self.y_line), (255, 0, 0), 2
        )  # Blue line with thickness 2
        return frame


class VideoServer:
    def __init__(
        self,
        host="0.0.0.0",
        port=5000,
        model_path="/home/ahmed/catkin_ws/src/yolo/src/best.pt",
    ):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((self.host, self.port))
        self.sock.listen(1)
        self.conn = None
        self.addr = None
        self.yolo_processor = YOLOProcessor(model_path)
        self.frame_processor = FrameProcessor()

    def start(self):
        print("Waiting for a connection...")
        self.conn, self.addr = self.sock.accept()
        print("Connected by", self.addr)
        self.process_video_stream()

    def process_video_stream(self):
        while True:
            try:
                # Receive size of the frame
                data = self.conn.recv(4)
                if not data:
                    break

                size = struct.unpack(">I", data)[0]

                # Receive the frame data
                data = b""
                while len(data) < size:
                    packet = self.conn.recv(size - len(data))
                    if not packet:
                        break
                    data += packet

                # Check if we have received the complete frame
                if len(data) != size:
                    print("Received incomplete data")
                    continue

                # Decode JPEG frame
                frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)

                # YOLOv8 inference
                detections, confidences = self.yolo_processor.process_frame(frame)

                # Check if something was detected and write to a file
                detection_status = len(detections) > 0
                self.write_detection_status(detection_status)

                # Flip the frame vertically
                flipped_frame = self.frame_processor.flip_vertical(frame)

                # Draw a horizontal line at the specified y_line position
                frame_with_line = self.frame_processor.draw_horizontal_line(
                    flipped_frame
                )

                # Draw YOLO detections on the flipped frame
                yolo_frame = self.yolo_processor.draw_boxes(
                    frame_with_line, detections, confidences, (0, 0, 255)
                )

                # Display the frames
                cv2.imshow("YOLOv8 Detections", yolo_frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            except Exception as e:
                print(f"An error occurred: {e}")
                break

        self.cleanup()

    def write_detection_status(self, status):
        with open("/home/ahmed/catkin_ws/src/yolo/src/detection_status.txt", "w") as f:
            f.write(str(status))

    def cleanup(self):
        self.conn.close()
        self.sock.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    server = VideoServer()
    server.start()
