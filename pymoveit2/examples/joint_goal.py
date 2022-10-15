#!/usr/bin/env python3
"""
Example of moving to a pose goal.
`ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False`
"""

from ast import List
from threading import Thread
from turtle import position
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2

def home_pose():
    return [0.0000, 0.0000, 0.0000, 0.0000, 0.0000]

def simple_place_prep():
    return [0.0000, -0.8505, 1.6489, -1.3017, 0.0000]

def simple_place_exec():
    return [0.0000, -0.3645, 1.6489, -1.3017, 0.0000]

def pickup_prep_pose():
    return [2.5167, -1.3500, 1.1900, 0.1562, -0.1709]

def pickup_exec_pose():
    return [2.5861, -1.0800, 1.7050, -0.6300, -0.1709]

def pickup_after_pose():
    return [2.5861, -0.9200, 0.9300, 0.0000, 0.0000]


home_joints = home_pose()
place_prep_joints = simple_place_prep()
place_exec_joints = simple_place_exec()
pickup_prep_joints = pickup_prep_pose()
pickup_exec_joints = pickup_exec_pose()
pickup_after_joints = pickup_after_pose()


def main(args=None):

    rclpy.init(args=args)

    node = Node("joint_goal")

    node.declare_parameter(
        "joint_positions",
        home_joints
    )

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=["joint12", "joint23", "joint34", "joint45", "joint56"],
        base_link_name="base_link",
        end_effector_name="Gripper",
        group_name="moveit_arm_controller",
        callback_group=callback_group,
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # joint_positions = (
    #     node.get_parameter("joint_positions").get_parameter_value().double_array_value
    # )
    # node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions)}}}")

    
    node.get_logger().info(f"Moving to {{joint_positions: {list(home_joints)}}}")
    moveit2.move_to_configuration(home_joints)
    moveit2.wait_until_executed()

    rclpy.spin(node)

    # rclpy.shutdown()
    # exit(0)


if __name__ == "__main__":
    main()
