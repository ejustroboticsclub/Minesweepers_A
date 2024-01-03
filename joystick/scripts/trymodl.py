
import cv2
from ultralytics import YOLO

model = YOLO(f"/home/anas/catkin_ws/src/joystick/scripts/yolo/small_best.pt")
results = model.predict("/home/anas/catkin_ws/src/joystick/scripts/captured_images/data/images/train/test_image_136.png", show=True)

