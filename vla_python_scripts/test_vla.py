import requests
import json_numpy as json
import numpy as np
import rclpy
import cv2
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import threading

class ActionPublisher(Node):
    def __init__(self):
        super().__init__('action_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'action_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.url = "http://0.0.0.0:8000/act"
        self.image = None
    
    def set_image(self, image):
        self.image = image
    
    def timer_callback(self):
        if self.image is None:
            self.get_logger().info("No image set, skipping callback.")
            return

        instruction = "pick up the white object"
        payload = json.dumps({"image": self.image, "instruction": instruction})
        
        try:
            response = requests.post(self.url, data=payload, headers={"Content-Type": "application/json"})
            response.raise_for_status()
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Request failed: {e}")
            return

        try:
            action = json.loads(response.text)
            print("Action:", action)
            
            if not isinstance(action, list) or not all(isinstance(i, (int, float)) for i in action):
                self.get_logger().error("Received invalid action data.")
                return

            msg = Float64MultiArray()
            msg.data = action
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published action: {action}")
        except Exception as e:
            self.get_logger().error(f"Error parsing response: {e}")
            self.get_logger().error(f"Response content: {response.content}")

def capture_frames(node):
    cap = cv2.VideoCapture(0)
    while rclpy.ok():
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        # Convert the frame to a numpy array and resize to 256x256
        resized_frame = cv2.resize(frame, (256, 256))
        image = np.array(resized_frame)
        
        # Update the image in the ROS node
        node.set_image(image)
        
        # Display the frame
        cv2.imshow("Camera Feed", frame)
        
        # Exit on pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    action_publisher = ActionPublisher()
    
    # Start the camera capture in a separate thread
    thread = threading.Thread(target=capture_frames, args=(action_publisher,))
    thread.start()
    
    # Spin the ROS2 node
    rclpy.spin(action_publisher)
    
    # Clean up
    action_publisher.destroy_node()
    rclpy.shutdown()
    thread.join()

if __name__ == '__main__':
    main()

