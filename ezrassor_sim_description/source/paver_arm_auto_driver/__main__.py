"""Execute a ROS node using the arms_driver module.

This node reads messages from a custom topic (external), parses them into
the value that Gazebo expects, and then publishes these messages
to Gazebo (internal). For the paver arms driver, these messages are
Float64 values that represent the velocity that should be applied to
raising or lowering the arm joints.

The abstraction done by this node is necessary to be able to switch
between hardware and software; a similarly written hardware node could
listen to the same istructions topic and command actuators, instead of the sim.
"""

import rclpy
from std_msgs.msg import (
    Float32,
    Float64,
    Float64MultiArray,
    MultiArrayDimension,
)
from trajectory_msgs.msg import JointTrajectory

from ast import List
from threading import Thread
from turtle import position

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2

import paver_arm_auto_functions as autonomy
# import pymoveit2.examples.joint_goal as autonomy

NODE = "paver_arm_driver"
CLAW_EXTERNAL_TOPIC = "claw_action"
CLAW_INTERNAL_TOPIC = "claw_effort_controller/commands"
PARTIAL_AUTONOMY_EXTERNAL_TOPIC = "partial_autonomy"

QUEUE_SIZE = 10

# Dictionary values set after publishers get created in main()
publishers = {}

# def handle_joint_autonomous_movements(data):
#     print()
#     # autonomy.joint_goal.handle_autonomy_functions()
#     # autonomy.main()


def handle_claw_movements(data):
    claw_msg = Float64MultiArray()
    claw_msg.layout.dim.append(MultiArrayDimension())
    claw_msg.layout.dim[0].size = 2
    claw_msg.layout.dim[0].stride = 1
    claw_msg.layout.dim[0].label = "x"
    claw_msg.data = [data.data] * 2
    
    publishers[CLAW_INTERNAL_TOPIC].publish(claw_msg)



def main(passed_args=None):
    """Main entry point for ROS node."""
    try:
        rclpy.init(args=passed_args)
        node = rclpy.create_node(NODE)

        # Create publishers to Gazebo velocity managers.
        publishers[CLAW_INTERNAL_TOPIC] = node.create_publisher(
            Float64MultiArray, CLAW_INTERNAL_TOPIC, QUEUE_SIZE
        )

        # Create subscriptions to listen for specific robot actions from users
        node.create_subscription(
            Float64,
            CLAW_EXTERNAL_TOPIC,
            handle_claw_movements,
            QUEUE_SIZE,
        )
        # node.create_subscription(
        #     Float64,
        #     PARTIAL_AUTONOMY_EXTERNAL_TOPIC,
        #     handle_joint_autonomous_movements,
        #     QUEUE_SIZE,
        # )

        node.get_logger().info("paver_arm_driver node created successfully")
        
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass