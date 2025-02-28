import cv2
import numpy as np
import requests
import json_numpy as json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import threading

unnorm_key = "bridge_orig"
instruction = "pick up the white object"

class ActionPublisher(Node):
    def __init__(self):
        super().__init__('action_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'action_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.action = None
        self.action_lock = threading.Lock()

    def timer_callback(self):
     # with self.action_lock:
            if self.action is not None:
                msg = Float64MultiArray()
                for i in self.action:   
                 msg.data.append(i)
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published action: {self.action}')
               # print("hello")
            else:
                self.get_logger().info('No action to publish')

def get_action(image, instruction):
    payload = json.dumps({"image": image, "instruction": instruction, "unnorm_key": unnorm_key})
    response = requests.post(
        "http://0.0.0.0:8000/act",
        data=payload,
        headers={"Content-Type": "application/json"}
    )
    if response.status_code == 200:
        action = json.loads(response.text)
        return action
    else:
        print(f"Failed to get action: {response.status_code}, {response.text}")
        return None

def capture_frames(node):
    cap = cv2.VideoCapture(0)
    while rclpy.ok():
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        resized_frame = cv2.resize(frame, (256, 256))
        image = np.array(resized_frame)

        try:
            action = get_action(image, instruction)
            if action is not None:
                print("Action:", action)
                with node.action_lock:
                    node.action = action
            else:
                print("No action received")
        except Exception as e:
            print(f"Error getting action: {e}")

        cv2.imshow("Camera Feed", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    action_publisher = ActionPublisher()

    thread = threading.Thread(target=capture_frames, args=(action_publisher,))
    thread.start()

    rclpy.spin(action_publisher)

    action_publisher.destroy_node()
    rclpy.shutdown()
    thread.join()

if __name__ == '__main__':
    main()

