import cv2
import base64
import socketio
import numpy as np

# Connect to the server
sio = socketio.Client()
server_url = "http://localhost:8000"  # Change this to your server address

@sio.event
def connect():
    print("Connected to server")

@sio.event
def disconnect():
    print("Disconnected from server")

# Function to capture and send camera frames
def send_camera_feed():
    cap = cv2.VideoCapture(0)  # Open default webcam (change index if needed)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture image")
            break
        
        # Convert frame to JPEG format
        _, buffer = cv2.imencode('.jpg', frame)
        encoded_frame = base64.b64encode(buffer).decode('utf-8')

        # Send frame to server
        sio.emit('image_stream', {'image': encoded_frame})

        # Display the frame locally
        cv2.imshow('Camera Feed', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Function to send OpenVLA instructions
def send_instruction(command):
    sio.emit('vla_instruction', {'command': command})

# Connect to the server
sio.connect(server_url)

# Start the camera feed
send_camera_feed()

# Disconnect after execution
sio.disconnect()

