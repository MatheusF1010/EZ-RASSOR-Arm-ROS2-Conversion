"""Execute a ROS node using the drums_driver module.

This node reads messages from a custom topic (external), parses them into
the value that Gazebo expects, and then publishes these messages to
Gazebo (internal). For the drums driver, these messages are Float64 values
that represent the velocity that should be applied to raising or lowering the arms.

The abstraction done by this node is necessary to be able to switch
between hardware and software; a similarly written hardware node could
listen to the same instructions topic and command actuators, instead of the sim.
"""
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

import rclpy
from rclpy.node import Node

NODE = "drums_driver"
#FRONT_DRUMS_EXTERNAL_TOPIC = "front_drum_instructions"
#FRONT_DRUMS_INTERNAL_TOPIC = "drum_front_velocity_controller/commands"
BACK_DRUMS_EXTERNAL_TOPIC = "back_drum_instructions"
BACK_DRUMS_INTERNAL_TOPIC = "drum_back_velocity_controller/commands"

QUEUE_SIZE = 10
MAX_DRUM_SPEED = 5


class DrumsSubscriber(Node):
    """Create a DRUMS_EXTERNAL_TOPIC subscriber node"""

    def __init__(self):
        """
        Class constructor for this subscriber node
        """
        super().__init__("drums_driver")

        self.back_drums_subscription = self.create_subscription(
            Float64,
            BACK_DRUMS_EXTERNAL_TOPIC,
            self.handle_back_drum_movements,
            QUEUE_SIZE,
        )

        self.back_drums_publisher = self.create_publisher(
            Float64MultiArray, BACK_DRUMS_INTERNAL_TOPIC, QUEUE_SIZE
        )

        self.get_logger().info("drums_driver node created successfully")

    def handle_back_drum_movements(self, data):
        """Move the back drum of the robot per
        the commands encoded in the instruction
        """

        back_drum_msg = Float64MultiArray()
        back_drum_msg.data = [data.data * MAX_DRUM_SPEED]

        self.front_drums_publisher.publish(back_drum_msg)


def main(passed_args=None):
    """Main entry point for the ROS node"""
    try:
        rclpy.init(args=passed_args)
        drums_subscriber = DrumsSubscriber()
        rclpy.spin(drums_subscriber)

    except KeyboardInterrupt:
        pass
