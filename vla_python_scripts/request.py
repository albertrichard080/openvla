import requests
import json_numpy as json
import numpy as np

# Prepare the image and instruction
image = np.zeros((256, 256, 3), dtype=np.uint8)  # Example image, replace with actual image data
instruction = "do something"

# Serialize the payload
payload = json.dumps({"image": image, "instruction": instruction})

# Send the request to the server
response = requests.post(
    "http://0.0.0.0:8000/act",
    data=payload,
    headers={"Content-Type": "application/json"}
)

# Parse the response
action = json.loads(response.text)
print(action)

