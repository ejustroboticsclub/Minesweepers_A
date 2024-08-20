import cv2
import socket
import struct
import numpy as np
from ultralytics import YOLO
from PIL import Image

# Configuration
host = '0.0.0.0'  # Listen on all interfaces
port = 5000

# Load YOLOv8 model
yolo_model = YOLO("/home/ahmed/catkin_ws/src/yolo/src/best.pt")  # Load a pre-trained YOLOv8 model

def draw_boxes(image, detections, confidences, color):
    for (x1, y1, x2, y2), conf in zip(detections, confidences):
        cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), color, 1)
        cv2.putText(image, f"Confidence: {conf:.2f}",
                    (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    return image

# Initialize socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((host, port))
sock.listen(1)

print("Waiting for a connection...")
conn, addr = sock.accept()
print("Connected by", addr)

while True:
    try:
        # Receive size of the frame
        data = conn.recv(4)
        if not data:
            break

        size = struct.unpack('>I', data)[0]
        
        # Receive the frame data
        data = b''
        while len(data) < size:
            packet = conn.recv(size - len(data))
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
        results = yolo_model(frame)
        detections = results[0].boxes.xyxy.cpu().numpy()
        confidences = results[0].boxes.conf.cpu().numpy()

        # Check if something was detected and write to a file
        if len(detections) > 0:
            detection_status = True
        else:
            detection_status = False

        # Write the detection status to a file
        with open("/home/ahmed/catkin_ws/src/yolo/src/detection_status.txt", "w") as f:
            f.write(str(detection_status))

        # Draw YOLO detections on the frame
        yolo_frame = frame.copy()
        yolo_frame = draw_boxes(yolo_frame, detections, confidences, (0, 0, 255))

        # Display the frames
        cv2.imshow('YOLOv8 Detections', yolo_frame)
        #cv2.imshow('Video Feed', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    except Exception as e:
        print(f"An error occurred: {e}")
        break

# Clean up
conn.close()
sock.close()
cv2.destroyAllWindows()