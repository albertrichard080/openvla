import rclpy
from rclpy.node import Node

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.pose_publisher = self.create_publisher(PoseStamped, 'target_pose', 10)
        self.gripper_publisher = self.create_publisher(Float64, 'gripper_command', 10)
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'action_topic',
            self.listener_callback,
            10
        )
    
    def listener_callback(self, msg):
        action = msg.data
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "px100/base_link"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = action[0]
        pose_msg.pose.position.y = action[1]
        pose_msg.pose.position.z = action[2]
        
        quaternion = self.euler_to_quaternion(action[3], action[4], action[5])
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        self.pose_publisher.publish(pose_msg)
        
        gripper_msg = Float64()
        gripper_msg.data = action[6]
        self.gripper_publisher.publish(gripper_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        q = np.zeros(4)
        q[0] = cr * cp * cy + sr * sp * sy
        q[1] = sr * cp * cy - cr * sp * sy
        q[2] = cr * sp * cy + sr * cp * sy
        q[3] = cr * cp * sy - sr * sp * cy

        return q

