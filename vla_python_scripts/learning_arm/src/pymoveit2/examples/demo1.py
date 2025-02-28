#!/usr/bin/env python3
"""
Example of moving to a joint configuration.
`ros2 run pymoveit2 ex_joint_goal.py --ros-args -p joint_positions:="[1.57, -1.57, 0.0, -1.57, 0.0, 1.57]"`
"""

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5

from threading import Thread

def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_joint_goal")

    # Declare parameter for joint positions
    node.declare_parameter(
        "joint_positions",
        [
            -0.03403341798766224,
            -1.2848632387872256,
            -1.8567441129914095,
            -3.185621281551551,
            -1.545888364367352,
            3.1498768354918307
        ],
    )

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Get parameter
    joint_positions = (
        node.get_parameter("joint_positions").get_parameter_value().double_array_value
    )

    # Move to joint configuration
    node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions)}}}")
    moveit2.move_to_configuration(joint_positions)
    moveit2.wait_until_executed()

    right_position = [-1.5668058110446967, -2.3899325357900847, 2.3999901086739235, -3.150098364710264, -1.5799238441208967, 3.1499310123204802]
    moveit2.move_to_configuration(right_position)
    moveit2.wait_until_executed()




    print("hi")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
