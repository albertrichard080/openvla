import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit2 import MoveGroupInterface

class PoseGoalExample(Node):

    def __init__(self):
        super().__init__('pose_goal_example')
        self.declare_parameter('position', [0.2, 0.0, 0.2])  # Example position [x, y, z]
        self.declare_parameter('quat_xyzw', [0.0, 0.0, 0.0, 1.0])  # Example orientation [x, y, z, w]

        # Initialize the MoveGroupInterface
        self.move_group = MoveGroupInterface("arm", "base_link")  # Replace "arm" with your robot's move group name

        self.pose_goal()

    def pose_goal(self):
        # Create the goal pose
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = 'base_link'  # Change as per your setup
        pose_goal.pose.position.x, pose_goal.pose.position.y, pose_goal.pose.position.z = self.get_parameter('position').get_parameter_value().double_array_value
        pose_goal.pose.orientation.x, pose_goal.pose.orientation.y, pose_goal.pose.orientation.z, pose_goal.pose.orientation.w = self.get_parameter('quat_xyzw').get_parameter_value().double_array_value

        # Set the pose target and execute the plan
        self.move_group.set_pose_target(pose_goal.pose)
        success = self.move_group.plan_and_execute()

        if success:
            self.get_logger().info("Motion plan successfully executed.")
        else:
            self.get_logger().error("Failed to execute motion plan.")

def main(args=None):
    rclpy.init(args=args)
    node = PoseGoalExample()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

