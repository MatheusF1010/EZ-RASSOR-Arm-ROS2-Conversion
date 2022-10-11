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
from std_msgs.msg import (
    Float32,
    Float64,
    Float64MultiArray,
    MultiArrayDimension,
)

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

NODE = "paver_arm_auto_function_listener"
PARTIAL_AUTONOMY_EXTERNAL_TOPIC = "/ezrassor/partial_autonomy"

QUEUE_SIZE = 10

# Dictionary values set after publishers get created in main()
publishers = {}

def handle_autonomy_functions(data):
    
    data_check = Float64()

    data_check = data.data

    if data_check == 1.0:
        node_home = Node("joint_goal_home")

        node_home.get_logger().info("Autonomy node_home created successfully")

        node_home.declare_parameter(
            "joint_positions",
            home_joints,
        )

        # Create callback group that allows execution of callbacks in parallel without restrictions
        callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 interface
        moveit2_prep = MoveIt2(
            node=node_home,
            joint_names=["joint12", "joint23", "joint34", "joint45", "joint56"],
            base_link_name="base_link",
            end_effector_name="Gripper",
            group_name="moveit_arm_controller",
            callback_group=callback_group,
        )

        # Spin the node in background thread(s)
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(node_home)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        
        node_home.get_logger().info(f"Moving to {{joint_positions: {list(home_joints)}}}")
        moveit2_prep.move_to_configuration(home_joints)
        moveit2_prep.wait_until_executed()
        
        rclpy.spin_once(node_home)

    if data_check == 2.0:
        node_place_prep = Node("joint_goal_place_prep")

        node_place_prep.get_logger().info("Autonomy node_place_prep created successfully")

        node_place_prep.declare_parameter(
            "joint_positions",
            place_prep_joints,
        )

        # Create callback group that allows execution of callbacks in parallel without restrictions
        callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 interface
        moveit2_prep = MoveIt2(
            node=node_place_prep,
            joint_names=["joint12", "joint23", "joint34", "joint45", "joint56"],
            base_link_name="base_link",
            end_effector_name="Gripper",
            group_name="moveit_arm_controller",
            callback_group=callback_group,
        )

        # Spin the node in background thread(s)
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(node_place_prep)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        
        node_place_prep.get_logger().info(f"Moving to {{joint_positions: {list(place_prep_joints)}}}")
        moveit2_prep.move_to_configuration(place_prep_joints)
        moveit2_prep.wait_until_executed()
        
        rclpy.spin_once(node_place_prep)

        node_place_exec = Node("joint_goal_place_exec")

        node_place_exec.get_logger().info("Autonomy node_place_exec created successfully")
        
        node_place_exec.declare_parameter(
            "joint_positions",
            place_exec_joints,
        )
        
        callback_group = ReentrantCallbackGroup()

        moveit2_exec = MoveIt2(
            node=node_place_exec,
            joint_names=["joint12", "joint23", "joint34", "joint45", "joint56"],
            base_link_name="base_link",
            end_effector_name="Gripper",
            group_name="moveit_arm_controller",
            callback_group=callback_group,
        )

        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(node_place_exec)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        
        node_place_exec.get_logger().info(f"Moving to {{joint_positions: {list(place_exec_joints)}}}")
        moveit2_exec.move_to_configuration(place_exec_joints)
        moveit2_exec.wait_until_executed()
        
        rclpy.spin_once(node_place_exec)

    if data_check == 3.0: 
        node_pickup_prep = Node("joint_goal_pickup_prep")

        node_pickup_prep.get_logger().info("Autonomy node_pickup_prep created successfully")

        node_pickup_prep.declare_parameter(
            "joint_positions",
            pickup_prep_joints,
        )

        # Create callback group that allows execution of callbacks in parallel without restrictions
        callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 interface
        moveit2_prep = MoveIt2(
            node=node_pickup_prep,
            joint_names=["joint12", "joint23", "joint34", "joint45", "joint56"],
            base_link_name="base_link",
            end_effector_name="Gripper",
            group_name="moveit_arm_controller",
            callback_group=callback_group,
        )

        # Spin the node in background thread(s)
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(node_pickup_prep)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        
        node_pickup_prep.get_logger().info(f"Moving to {{joint_positions: {list(pickup_prep_joints)}}}")
        moveit2_prep.move_to_configuration(pickup_prep_joints)
        moveit2_prep.wait_until_executed()
        
        rclpy.spin_once(node_pickup_prep)

        node_pickup_exec = Node("joint_goal_pickup_exec")

        node_pickup_exec.get_logger().info("Autonomy node_pickup_exec created successfully")
        
        node_pickup_exec.declare_parameter(
            "joint_positions",
            pickup_exec_joints,
        )
        
        callback_group = ReentrantCallbackGroup()

        moveit2_exec = MoveIt2(
            node=node_pickup_exec,
            joint_names=["joint12", "joint23", "joint34", "joint45", "joint56"],
            base_link_name="base_link",
            end_effector_name="Gripper",
            group_name="moveit_arm_controller",
            callback_group=callback_group,
        )

        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(node_pickup_exec)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        
        node_pickup_exec.get_logger().info(f"Moving to {{joint_positions: {list(pickup_exec_joints)}}}")
        moveit2_exec.move_to_configuration(pickup_exec_joints)
        moveit2_exec.wait_until_executed()
        
        rclpy.spin_once(node_pickup_exec)

        node_pickup_after = Node("joint_goal_pickup_after")

        node_pickup_after.get_logger().info("Autonomy node_pickup_after created successfully")
        
        node_pickup_after.declare_parameter(
            "joint_positions",
            pickup_after_joints,
        )
        
        callback_group = ReentrantCallbackGroup()

        moveit2_after = MoveIt2(
            node=node_pickup_after,
            joint_names=["joint12", "joint23", "joint34", "joint45", "joint56"],
            base_link_name="base_link",
            end_effector_name="Gripper",
            group_name="moveit_arm_controller",
            callback_group=callback_group,
        )

        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(node_pickup_after)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        
        node_pickup_after.get_logger().info(f"Moving to {{joint_positions: {list(pickup_after_joints)}}}")
        moveit2_after.move_to_configuration(pickup_after_joints)
        moveit2_after.wait_until_executed()
        
        rclpy.spin_once(node_pickup_after)


        


def main(args=None):

    """Main entry point for ROS node."""
    try:
        rclpy.init()
        node = rclpy.create_node(NODE)

        node.create_subscription(
            Float64,
            PARTIAL_AUTONOMY_EXTERNAL_TOPIC,
            handle_autonomy_functions,
            QUEUE_SIZE,
        )

        node.get_logger().info("paver_arm_driver node created successfully")
        
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass





# Works ros2 launch ezrassor_sim_description ezrassor_auto_functions.py
# def main(args=None):

#     rclpy.init(args=args)

#     node = Node("joint_goal_PLEASE_WORK")

#     node.get_logger().info("Autonomy node created successfully")

#     node.declare_parameter(
#         "joint_positions",
#         home_joints
#     )

#     # Create callback group that allows execution of callbacks in parallel without restrictions
#     callback_group = ReentrantCallbackGroup()

#     # Create MoveIt 2 interface
#     moveit2 = MoveIt2(
#         node=node,
#         joint_names=["joint12", "joint23", "joint34", "joint45", "joint56"],
#         base_link_name="base_link",
#         end_effector_name="Gripper",
#         group_name="moveit_arm_controller",
#         callback_group=callback_group,
#     )

#     # Spin the node in background thread(s)
#     executor = rclpy.executors.MultiThreadedExecutor(2)
#     executor.add_node(node)
#     executor_thread = Thread(target=executor.spin, daemon=True, args=())
#     executor_thread.start()

#     # joint_positions = (
#     #     node.get_parameter("joint_positions").get_parameter_value().double_array_value
#     # )
#     # node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions)}}}")

    
#     node.get_logger().info(f"IM Moving to {{joint_positions: {list(home_joints)}}}")
#     moveit2.move_to_configuration(home_joints)
#     moveit2.wait_until_executed()
    
#     rclpy.spin(node)

#     # rclpy.spin(node)

#     # handle_autonomy_functions()

#     # rclpy.shutdown()
#     # exit(0)


if __name__ == "__main__":
    main()
