#!/usr/bin/env python3
"""
Example of using MoveIt 2 Servo to perform a circular motion.
`ros2 run pymoveit2 ex_servo.py`
"""


from math import cos, sin
import math, time
from copy import deepcopy
import rclpy
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from pymoveit2.robots import ur5
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

# Initialize message based on passed arguments

__twist_msg = TwistStamped()
__twist_msg.header.frame_id = "ee_link"
__twist_msg.twist.linear.x = 0.0
__twist_msg.twist.linear.y = 0.0
__twist_msg.twist.linear.z = 0.0
__twist_msg.twist.angular.x = 0.0
__twist_msg.twist.angular.y = 0.0
__twist_msg.twist.angular.z = 0.0


angle = 0.0


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_servo")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()
    __twist_pub = node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
    
    def servo_circular_motion():
        """Move in a circular motion using Servo"""
        # global angle
        # angle += 0.1
       
        # __twist_msg.twist.linear.y = cos(angle)
        # __twist_msg.twist.linear.z = sin(angle)
        # __twist_msg.header.stamp = node.get_clock().now().to_msg()
        # __twist_pub.publish(__twist_msg)
        __twist_msg.twist.linear.x = 0.2
        __twist_msg.twist.linear.y = 0.0
        __twist_msg.twist.linear.z = 0.0
        __twist_msg.header.stamp = node.get_clock().now().to_msg()
        __twist_pub.publish(__twist_msg)

    # Create timer for moving in a circular motion
    node.create_timer(0.02, servo_circular_motion)

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
