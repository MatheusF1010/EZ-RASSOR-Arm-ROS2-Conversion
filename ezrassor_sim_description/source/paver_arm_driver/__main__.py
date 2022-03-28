"""Execute a ROS node using the drums_driver module.

This node reads messages from a custom topic (external), parses them into
the value that Gazebo expects, and then publishes these messages to
Gazebo (internal). 'For the paver arm driver, these messages are Float64 values
that represent the trajectory that should be applied to raising or lowering the joints?'

The abstraction done by this node is necessary to be able to switch
between hardware and software; a similarly written hardware node could
listen to the same instructions topic and command actuators, instead of the sim.
"""


from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

import rclpy
from rclpy.node import Node

NODE = "paver_arm_driver"

PAVER_ARM_FIRST_JOINT_EXTERNAL_TOPIC = "paver_arm_joint_1_instructions"
PAVER_ARM_SECOND_JOINT_EXTERNAL_TOPIC = "paver_arm_joint_2_instructions"
PAVER_ARM_THIRD_JOINT_EXTERNAL_TOPIC = "paver_arm_joint_3_instructions"
PAVER_ARM_FOURTH_JOINT_EXTERNAL_TOPIC = "paver_arm_joint_4_instructions"
PAVER_ARM_FIFTH_JOINT_EXTERNAL_TOPIC = "paver_arm_joint_5_instructions"
PAVER_ARM_CLAW_EXTERNAL_TOPIC = "paver_arm_claw_instructions"

PAVER_ARM_FIRST_JOINT_INTERNAL_TOPIC = "paver_arm_joint_1_trajectory_controller/commands"
PAVER_ARM_SECOND_JOINT_INTERNAL_TOPIC = "paver_arm_joint_2_trajectory_controller/commands"
PAVER_ARM_THIRD_JOINT_INTERNAL_TOPIC = "paver_arm_joint_3_trajectory_controller/commands"
PAVER_ARM_FOURTH_JOINT_INTERNAL_TOPIC = "paver_arm_joint_4_trajectory_controller/commands"
PAVER_ARM_FIFTH_JOINT_INTERNAL_TOPIC = "paver_arm_joint_5_trajectory_controller/commands"
PAVER_ARM_CLAW_INTERNAL_TOPIC = "paver_arm_claw_effort_controller/commands"

QUEUE_SIZE = 10
MAX_PAVER_ARM_SPEED = 5


class PaverArmSubscriber(Node):
    """Create a DRUMS_EXTERNAL_TOPIC subscriber node"""

    def __init__(self):
        """
        Class constructor for this subscriber node
        """
        super().__init__("paver_arm_driver")

        self.paver_arm_joint_1_subscription = self.create_subscription(
            Float64,
            PAVER_ARM_FIRST_JOINT_EXTERNAL_TOPIC,
            self.handle_paver_arm_joint_1_movements,
            QUEUE_SIZE,
        )

        self.paver_arm_joint_2_subscription = self.create_subscription(
            Float64,
            PAVER_ARM_SECOND_JOINT_EXTERNAL_TOPIC,
            self.handle_paver_arm_joint_2_movements,
            QUEUE_SIZE,
        )

        self.paver_arm_joint_3_subscription = self.create_subscription(
            Float64,
            PAVER_ARM_THIRD_JOINT_EXTERNAL_TOPIC,
            self.handle_paver_arm_joint_3_movements,
            QUEUE_SIZE,
        )

        self.paver_arm_joint_4_subscription = self.create_subscription(
            Float64,
            PAVER_ARM_FOURTH_JOINT_EXTERNAL_TOPIC,
            self.handle_paver_arm_joint_4_movements,
            QUEUE_SIZE,
        )

        self.paver_arm_joint_5_subscription = self.create_subscription(
            Float64,
            PAVER_ARM_FIFTH_JOINT_EXTERNAL_TOPIC,
            self.handle_paver_arm_joint_5_movements,
            QUEUE_SIZE,
        )

        self.paver_arm_claw_subscription = self.create_subscription(
            Float64,
            PAVER_ARM_CLAW_EXTERNAL_TOPIC,
            self.handle_paver_arm_claw_movements,
            QUEUE_SIZE,
        )

        self.paver_arm_joint_1_publisher = self.create_publisher(
            Float64, PAVER_ARM_FIRST_JOINT_INTERNAL_TOPIC, QUEUE_SIZE
        )

        self.paver_arm_joint_2_publisher = self.create_publisher(
            Float64, PAVER_ARM_SECOND_JOINT_INTERNAL_TOPIC, QUEUE_SIZE
        )

        self.paver_arm_joint_3_publisher = self.create_publisher(
            Float64, PAVER_ARM_THIRD_JOINT_INTERNAL_TOPIC, QUEUE_SIZE
        )

        self.paver_arm_joint_4_publisher = self.create_publisher(
            Float64, PAVER_ARM_FOURTH_JOINT_INTERNAL_TOPIC, QUEUE_SIZE
        )

        self.paver_arm_joint_5_publisher = self.create_publisher(
            Float64, PAVER_ARM_FIFTH_JOINT_INTERNAL_TOPIC, QUEUE_SIZE
        )

        self.paver_arm_claw_publisher = self.create_publisher(
            Float64MultiArray, PAVER_ARM_CLAW_INTERNAL_TOPIC, QUEUE_SIZE
        )

        self.get_logger().info("paver_arm_driver node created successfully")


    def handle_paver_arm_joint_1_movements(self, data):
        """Move the front drum of the robot per
        the commands encoded in the instruction
        """
        paver_arm_joint_1_msg = Float64MultiArray()
        paver_arm_joint_1_msg = [data.data * MAX_PAVER_ARM_SPEED]
        self.paver_arm_joint_1_publisher.publish(paver_arm_joint_1_msg)
    
    def handle_paver_arm_joint_2_movements(self, data):
        """Move the front drum of the robot per
        the commands encoded in the instruction
        """
        paver_arm_joint_2_msg = Float64MultiArray()
        paver_arm_joint_2_msg = [data.data * MAX_PAVER_ARM_SPEED]
        self.paver_arm_joint_2_publisher.publish(paver_arm_joint_2_msg)
    
    def handle_paver_arm_joint_3_movements(self, data):
        """Move the front drum of the robot per
        the commands encoded in the instruction
        """
        paver_arm_joint_3_msg = Float64MultiArray()
        paver_arm_joint_3_msg = [data.data * MAX_PAVER_ARM_SPEED]
        self.paver_arm_joint_3_publisher.publish(paver_arm_joint_3_msg)
    
    def handle_paver_arm_joint_4_movements(self, data):
        """Move the front drum of the robot per
        the commands encoded in the instruction
        """
        paver_arm_joint_4_msg = Float64MultiArray()
        paver_arm_joint_4_msg = [data.data * MAX_PAVER_ARM_SPEED]
        self.paver_arm_joint_4_publisher.publish(paver_arm_joint_4_msg)
    
    def handle_paver_arm_joint_5_movements(self, data):
        """Move the front drum of the robot per
        the commands encoded in the instruction
        """
        paver_arm_joint_5_msg = Float64MultiArray()
        paver_arm_joint_5_msg = [data.data * MAX_PAVER_ARM_SPEED]
        self.paver_arm_joint_5_publisher.publish(paver_arm_joint_5_msg)
    
    def handle_paver_arm_claw_movements(self, data):
        """Move the front drum of the robot per
        the commands encoded in the instruction
        """
        paver_arm_claw_msg = Float64MultiArray()
        paver_arm_claw_msg = [data.data * MAX_PAVER_ARM_SPEED]
        self.paver_arm_claw_publisher.publish(paver_arm_claw_msg)


def main(passed_args=None):
    """Main entry point for the ROS node"""
    try:
        rclpy.init(args=passed_args)
        paver_arm_subscriber = PaverArmSubscriber()
        rclpy.spin(paver_arm_subscriber)

    except KeyboardInterrupt:
        pass
