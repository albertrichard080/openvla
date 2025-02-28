import cv2
import numpy as np
import requests
import json_numpy as json

# Open a connection to the camera
cap = cv2.VideoCapture(0)
unnorm_key="bridge_orig"
instruction = "pick up the white object"

def get_action(image, instruction):
    # Serialize the payload
    payload = json.dumps({"image": image, "instruction": instruction, "unnorm_key": unnorm_key})

    # Send the request to the server
    response = requests.post(
        "http://0.0.0.0:8000/act",
        data=payload,
        headers={"Content-Type": "application/json"}
    )

    # Parse the response
    action = json.loads(response.text)
    return action
# Continuously capture frames and send to the model
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break
    
    # Convert the frame to a numpy array
    image = np.array(frame)
    
    # Get the action from the model
    action = get_action(image, "pick up the white object")
    print("Action:", action)

    # Use the action to control the robot
    # For example, assuming you have a function robot.act():
    #robot.act(action)
    
    # Display the frame
    cv2.imshow("Camera Feed", frame)
    
    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()

