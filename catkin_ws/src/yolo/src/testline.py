import cv2
import numpy as np
from ultralytics import YOLO

# Load YOLOv8 model
yolo_model = YOLO("/home/ahmed/catkin_ws/src/yolo/src/best.pt")  # Load a pre-trained YOLOv8 model

# Line position (y-coordinate)
line_y = 300  # Adjust this value as needed

def draw_boxes(image, detections, confidences, line_y):
    line_color = (0, 0, 255)  # Default line color (red)

    for (x1, y1, x2, y2), conf in zip(detections, confidences):
        cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 1)
        cv2.putText(
            image,
            f"Confidence: {conf:.2f}",
            (int(x1), int(y1) - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )

        # Check if the bottom of the object touches or crosses the line
        if int(y2) >= line_y:
            line_color = (0, 255, 0)  # Change line color to green if touched

    # Draw the line on the frame
    cv2.line(image, (0, line_y), (image.shape[1], line_y), line_color, 2)

    return image

# Open webcam
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # YOLOv8 inference
    results = yolo_model(frame)
    detections = results[0].boxes.xyxy.cpu().numpy()
    confidences = results[0].boxes.conf.cpu().numpy()

    # Draw YOLO detections and the line on the frame
    yolo_frame = draw_boxes(frame, detections, confidences, line_y)

    # Display the frames
    cv2.imshow("YOLOv8 Detections", yolo_frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Clean up
cap.release()
cv2.destroyAllWindows()
