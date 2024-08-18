import cv2
import socket
import struct

# Configuration
server_ip = '192.168.165.108'  # PC's IP address
server_port = 5000  # Port to send the video stream

# Initialize socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((server_ip, server_port))

# Initialize camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FPS, 40)  # Set frame rate

# Set lower resolution
width = 640   # Set desired width
height = 480  # Set desired height
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Encode frame as JPEG with lower quality
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 20]  # Quality range is 0-100, 50 is a good balance
    ret, jpeg = cv2.imencode('.jpg', frame, encode_param)
    if not ret:
        continue

    # Send frame size and data
    data = jpeg.tobytes()
    sock.sendall(struct.pack('>I', len(data)) + data)

# Clean up
cap.release()
sock.close()
