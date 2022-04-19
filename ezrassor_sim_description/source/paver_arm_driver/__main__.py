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

NODE = "paver_arm_driver"
FIRST_JOINT_EXTERNAL_TOPIC = "joint_1_action"
FIRST_JOINT_INTERNAL_TOPIC = "joint_1_trajectory_controller/commands"
SECOND_JOINT_EXTERNAL_TOPIC = "joint_2_action"
SECOND_JOINT_INTERNAL_TOPIC = "joint_2_trajectory_controller/commands"
THIRD_JOINT_EXTERNAL_TOPIC = "joint_3_action"
THIRD_JOINT_INTERNAL_TOPIC = "joint_3_trajectory_controller/commands"
FOURTH_JOINT_EXTERNAL_TOPIC = "joint_4_action"
FOURTH_JOINT_INTERNAL_TOPIC = "joint_4_trajectory_controller/commands"
FIFTH_JOINT_EXTERNAL_TOPIC = "joint_5_action"
FIFTH_JOINT_INTERNAL_TOPIC = "joint_5_trajectory_controller/commands"
CLAW_EXTERNAL_TOPIC = "claw_action"
CLAW_INTERNAL_TOPIC = "claw_velocity_controller/commands"

QUEUE_SIZE = 10

# Dictionary values set after publishers get created in main()
publishers = {}

def handle_first_joint_movements(data):

    first_joint_msg = Float64()
    first_joint_msg.data = [data.data]

    publishers[FIRST_JOINT_INTERNAL_TOPIC].publish(first_joint_msg)

def handle_second_joint_movements(data):

    second_joint_msg = Float64()
    second_joint_msg.data = [data.data]

    publishers[SECOND_JOINT_INTERNAL_TOPIC].publish(second_joint_msg)

def handle_third_joint_movements(data):

    third_joint_msg = Float64()
    third_joint_msg.data = [data.data]

    publishers[THIRD_JOINT_INTERNAL_TOPIC].publish(third_joint_msg)

def handle_fourth_joint_movements(data):

    fourth_joint_msg = Float64()
    fourth_joint_msg.data = [data.data]

    publishers[FOURTH_JOINT_INTERNAL_TOPIC].publish(fourth_joint_msg)

def handle_fifth_joint_movements(data):

    fifth_joint_msg = Float64()
    fifth_joint_msg.data = [data.data]

    publishers[FIFTH_JOINT_INTERNAL_TOPIC].publish(fifth_joint_msg)

def handle_claw_movements(data):
    claw_msg = Float64MultiArray()
    claw_msg.layout.dim.append(MultiArrayDimension())
    claw_msg.layout.dim[0].size = 2
    claw_msg.layout.dim[0].stride = 1
    claw_msg.layout.dim[0].label = "x"
    claw_msg.data = [data.data * 2]
    
    publishers[CLAW_INTERNAL_TOPIC].publish(claw_msg)

def main(passed_args=None):
    """Main entry point for ROS node."""
    try:
        rclpy.init(args=passed_args)
        node = rclpy.create_node(NODE)

        # Create publishers to Gazebo velocity managers.
        publishers[FIRST_JOINT_INTERNAL_TOPIC] = node.create_publisher(
            Float64, FIRST_JOINT_INTERNAL_TOPIC, QUEUE_SIZE
        )
        publishers[SECOND_JOINT_INTERNAL_TOPIC] = node.create_publisher(
            Float64, SECOND_JOINT_INTERNAL_TOPIC, QUEUE_SIZE
        )
        publishers[THIRD_JOINT_INTERNAL_TOPIC] = node.create_publisher(
            Float64, THIRD_JOINT_INTERNAL_TOPIC, QUEUE_SIZE
        )
        publishers[FOURTH_JOINT_INTERNAL_TOPIC] = node.create_publisher(
            Float64, FOURTH_JOINT_INTERNAL_TOPIC, QUEUE_SIZE
        )
        publishers[FIFTH_JOINT_INTERNAL_TOPIC] = node.create_publisher(
            Float64, FIFTH_JOINT_INTERNAL_TOPIC, QUEUE_SIZE
        )
        publishers[CLAW_INTERNAL_TOPIC] = node.create_publisher(
            Float64MultiArray, CLAW_INTERNAL_TOPIC, QUEUE_SIZE
        )

        # Create subscriptions to listen for specific robot actions from users.
        node.create_subscription(
            Float64,
            FIRST_JOINT_EXTERNAL_TOPIC,
            handle_first_joint_movements,
            QUEUE_SIZE,
        )
        node.create_subscription(
            Float64,
            SECOND_JOINT_EXTERNAL_TOPIC,
            handle_second_joint_movements,
            QUEUE_SIZE,
        )
        node.create_subscription(
            Float64,
            THIRD_JOINT_EXTERNAL_TOPIC,
            handle_third_joint_movements,
            QUEUE_SIZE,
        )
        node.create_subscription(
            Float64,
            FOURTH_JOINT_EXTERNAL_TOPIC,
            handle_fourth_joint_movements,
            QUEUE_SIZE,
        )
        node.create_subscription(
            Float64,
            FIFTH_JOINT_EXTERNAL_TOPIC,
            handle_fifth_joint_movements,
            QUEUE_SIZE,
        )
        node.create_subscription(
            Float32,
            CLAW_EXTERNAL_TOPIC,
            handle_claw_movements,
            QUEUE_SIZE,
        )

        node.get_logger().info("paver_arm_driver node created successfully")
        
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass