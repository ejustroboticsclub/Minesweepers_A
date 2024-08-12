import cv2
import socket
import struct
import numpy as np
#import torch
from ultralytics import YOLO
from PIL import Image
#from torchvision.transforms import Compose, Resize, ToTensor, Normalize, InterpolationMode

# Configuration
host = '0.0.0.0'  # Listen on all interfaces
port = 5000

# Load YOLOv8 model
yolo_model = YOLO("/media/ahmed/Work/Robotics Club/Minesweepers/Minesweepers_A/YOLO/best.pt")  # Load a pre-trained YOLOv8 model

"""# Load MiDaS model
midas = torch.hub.load("intel-isl/MiDaS", "MiDaS_small")  # Load MiDaS model
midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
midas.eval()
"""

"""def preprocess_midas(img):
    midas_transform = Compose([
        Resize((384, 384), interpolation=InterpolationMode.BICUBIC),
        ToTensor(),
        Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    input_batch = midas_transform(Image.fromarray(img_rgb)).unsqueeze(0)
    return input_batch
"""

def draw_boxes(image, detections, confidences, color):
    for (x1, y1, x2, y2), conf in zip(detections, confidences):
        cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), color, 1)
        cv2.putText(image, f"Confidence: {conf:.2f}",
                    (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    return image

"""
def draw_boxes_and_depth(image, detections, confidences, depth_map):
    for (x1, y1, x2, y2), conf in zip(detections, confidences):
        depth_value = depth_map[int((y1 + y2) / 2), int((x1 + x2) / 2)]
        distance = depth_value  # Direct distance
        cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
        cv2.putText(image, f"Depth: {distance/240:.2f} cm, Conf: {conf:.2f}",
                    (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    return image
"""
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

        # MiDaS inference
        # midas_input = preprocess_midas(frame)
        # with torch.no_grad():
        #     depth_prediction = midas(midas_input)
        #     depth_prediction = torch.nn.functional.interpolate(
        #         depth_prediction.unsqueeze(1),
        #         size=frame.shape[:2],
        #         mode="bicubic",
        #         align_corners=False,
        #     ).squeeze()
        # depth_map = depth_prediction.cpu().numpy()

        # Draw YOLO detections on the frame
        yolo_frame = frame.copy()
        yolo_frame = draw_boxes(yolo_frame, detections, confidences, (0, 0, 0))

        # # Draw depth map on the frame
        # midas_frame = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
        # midas_frame = cv2.applyColorMap(midas_frame, cv2.COLORMAP_JET)

        # # Draw combined results on the frame
        # combined_frame = draw_boxes_and_depth(frame.copy(), detections, confidences, depth_map)

        # Display the frames
        cv2.imshow('YOLOv8 Detections', yolo_frame)
        #cv2.imshow('MiDaS Depth Map', midas_frame)
        #cv2.imshow('Combined YOLOv8 + MiDaS', combined_frame)
        cv2.imshow('Video Feed', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    except Exception as e:
        print(f"An error occurred: {e}")
        break

# Clean up
conn.close()
sock.close()
cv2.destroyAllWindows()
