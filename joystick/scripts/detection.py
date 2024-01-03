import rospy
from std_msgs.msg import Int8


import cv2
from ultralytics import YOLO

model = YOLO(f"/home/anas/catkin_ws/src/joystick/scripts/yolo/small_best.pt")

def main():
    rospy.init_node("detect_camera",anonymous=True)
    mine_theta = rospy.Publisher('/mine_theta', Int8,queue_size=10)

    cap = cv2.VideoCapture("rtspsrc location=rtsp://192.168.0.140:8554/unicast latency=0 buffer-mode=auto ! decodebin ! videoconvert ! appsink max-buffers=1 drop=True",cv2.CAP_GSTREAMER)
    # cap = cv2.VideoCapture(0)
    while cap.isOpened():
        ret, frame = cap.read()
        frame = cv2.line(frame, (20,500),(780,500),(0,255,0),9)
        detected = 2
        results=model(frame,show = True, conf=0.5)
        for result in results:

            detection_count = result.boxes.shape[0]

            for i in range(detection_count):
                cls = int(result.boxes.cls[i].item())
                name = result.names[cls]
                confidence = float(result.boxes.conf[i].item())
                # print(f"{i}: {name} {confidence:.4f}")
                if name == 'cube' and confidence > 0.83:
                    print("Mine Detected")
                    detected = 1
        mine_theta.publish(detected)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break 
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


