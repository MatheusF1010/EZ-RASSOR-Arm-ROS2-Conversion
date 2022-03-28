"""Execute a ROS node using the wheels_driver module.

This node reads messages from a custom topic (external), parses them into
the value that Gazebo expects, and then publishes these messages
to Gazebo (internal). For the wheels driver, these messages are Twist values
that represent the linear and angular velocity that should be applied to wheels.

The abstraction done by this node is necessary to be able to switch
between hardware and software; a similarly written hardware node could
listen to the same instructions topic and command actuators, instead of the sim.
"""
import geometry_msgs.msg
import rclpy
from rclpy.node import Node

WHEELS_EXTERNAL_TOPIC = "wheel_instructions"
WHEELS_INTERNAL_TOPIC = "diff_drive_controller/cmd_vel_unstamped"
MAX_VELOCITY = 5
QUEUE_SIZE = 10


class WheelsSubscriber(Node):
    """Create a WHEELS_EXTERNAL_TOPIC subscriber node"""

    def __init__(self):
        """
        Class constructor for this subscriber node
        """
        super().__init__("wheels_driver")

        self.subscription = self.create_subscription(
            geometry_msgs.msg.Twist,
            WHEELS_EXTERNAL_TOPIC,
            self.handle_wheel_movements,
            QUEUE_SIZE,
        )

        self.simulation_publisher = self.create_publisher(
            geometry_msgs.msg.Twist, WHEELS_INTERNAL_TOPIC, QUEUE_SIZE
        )

        self.get_logger().info("wheels_driver node created successfully")

    def handle_wheel_movements(self, twist):
        """
        Callback function which places limits on the speed passed in before publishing
        to the simulation diff_drive topic to move the robot
        """

        diff_drive_twist = geometry_msgs.msg.Twist()

        # Although there are caps on the speed in the diff_drive, this helps the
        # rover not turn incredibly slow when making lefts, rights, donuts, etc.

        diff_drive_twist.linear.x = twist.linear.x * MAX_VELOCITY
        diff_drive_twist.linear.y = twist.linear.y
        diff_drive_twist.linear.z = twist.linear.z
        diff_drive_twist.angular.x = twist.angular.x
        diff_drive_twist.angular.y = twist.angular.y
        diff_drive_twist.angular.z = twist.angular.z * MAX_VELOCITY

        self.simulation_publisher.publish(diff_drive_twist)

    def publish_wheel_twist(self, twist):
        """Publishes the final twist message to the simulation topic
        to move the robot"""
        self.simulation_publisher.publish(twist)


def main(passed_args=None):
    """Main entry point for the ROS node"""
    try:
        rclpy.init(args=passed_args)
        wheels_subscriber = WheelsSubscriber()
        rclpy.spin(wheels_subscriber)

    except KeyboardInterrupt:
        pass
