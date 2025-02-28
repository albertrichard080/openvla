import cv2
import numpy as np
import requests
import json_numpy as json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import threading
import time

class ActionSubscriber(Node):
    def __init__(self):
        super().__init__('action_subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'action_topic',
            self.action_callback,
            10)
        self.publisher_ = self.create_publisher(JointState, 'joint_command', 10)
        self.joint_state = JointState()
        self.joint_state.name = [
            "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7", "panda_finger_joint1", "panda_finger_joint2"
        ]
        self.action = None
        self.action_lock = threading.Lock()
        self.timer = self.create_timer(0.05, self.timer_callback)

    def action_callback(self, msg):
        with self.action_lock:
            self.action = msg.data
        self.get_logger().info(f'Received action: {self.action}')

    def timer_callback(self):
     if self.action is not None:
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        num_joints = len(self.joint_state.name)
        self.joint_state.position = self.action[:num_joints]  # Ensure correct size
        self.joint_state.velocity = [0.0] * num_joints  # Set velocity to zero
        self.joint_state.effort = [0.0] * num_joints  # Set effort to zero
        self.publisher_.publish(self.joint_state)
        self.get_logger().info(f'Published joint states: {self.joint_state.position}')


def main(args=None):
    rclpy.init(args=args)
    action_subscriber = ActionSubscriber()
    rclpy.spin(action_subscriber)
    action_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

