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
from trajectory_msgs.msg import JointTrajectoryPoint

import serial
import time
import sys

NODE = "paver_arm_serial_driver"
FIRST_JOINT_EXTERNAL_TOPIC = "joint_1_action"
SECOND_JOINT_EXTERNAL_TOPIC = "joint_2_action"
THIRD_JOINT_EXTERNAL_TOPIC = "joint_3_action"
FOURTH_JOINT_EXTERNAL_TOPIC = "joint_4_action"
FIFTH_JOINT_EXTERNAL_TOPIC = "joint_5_action"
CLAW_EXTERNAL_TOPIC = "claw_action"
PARTIAL_AUTONOMY_EXTERNAL_TOPIC = "partial_autonomy"

QUEUE_SIZE = 10
# Dictionary values set after publishers get created in main()
publishers = {}

arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
arduino.reset_input_buffer()

global buttonPressed

# Joint Q
def handle_first_joint_movements(data):
    global buttonPressed

    first_joint_msg = Float64MultiArray()
    first_joint_msg.data = [data.data]

    if buttonPressed == False:
        print("Pressed", file=sys.stderr)

        if data.data > 0:
            arduino.write(b"M 0 0 0 0 1 \n")
        else:
            arduino.write(b"M 0 0 0 0 -1 \n")

        buttonPressed = True

    else:
        print("Depressed", file=sys.stderr)
        arduino.write(b"B \n")
        buttonPressed = False
        line = arduino.readline().decode('utf-8').rstrip()
        print(line)

# Joint Z
def handle_second_joint_movements(data):

    global buttonPressed

    second_joint_msg = Float64MultiArray()
    second_joint_msg.data = [data.data]

    if buttonPressed == False:
        print("Pressed", file=sys.stderr)

        if data.data > 0:
            arduino.write(b"M 0 0 1 0 0 \n")
        else:
            arduino.write(b"M 0 0 -1 0 0 \n")

        buttonPressed = True

    else:
        print("Depressed", file=sys.stderr)
        arduino.write(b"B \n")
        buttonPressed = False
        line = arduino.readline().decode('utf-8').rstrip()
        print(line)

# Joint Y
def handle_third_joint_movements(data):

    global buttonPressed

    third_joint_msg = Float64MultiArray()
    third_joint_msg.data = [data.data]

    if buttonPressed == False:
        print("Pressed", file=sys.stderr)

        if data.data > 0:
            arduino.write(b"M 0 1 0 0 0 \n")
        else:
            arduino.write(b"M 0 -1 0 0 0 \n")

        buttonPressed = True

    else:
        print("Depressed", file=sys.stderr)
        arduino.write(b"B \n")
        buttonPressed = False
        line = arduino.readline().decode('utf-8').rstrip()
        print(line)

# Joint X
def handle_fourth_joint_movements(data):

    global buttonPressed

    fourth_joint_msg = Float64MultiArray()
    fourth_joint_msg.data = [data.data]

    if buttonPressed == False:
        print("Pressed", file=sys.stderr)

        if data.data > 0:
            arduino.write(b"M 1 0 0 0 0 \n")
        else:
            arduino.write(b"M -1 0 0 0 0 \n")

        buttonPressed = True

    else:
        print("Depressed", file=sys.stderr)
        arduino.write(b"B \n")
        buttonPressed = False
        line = arduino.readline().decode('utf-8').rstrip()
        print(line)

# Joint E
def handle_fifth_joint_movements(data):

    global buttonPressed

    fifth_joint_msg = Float64MultiArray()
    fifth_joint_msg.data = [data.data]

    if buttonPressed == False:
        print("Pressed", file=sys.stderr)

        if data.data > 0:
            arduino.write(b"M 0 0 0 -1 0 \n")
        else:
            arduino.write(b"M 0 0 0 1 0 \n")

        buttonPressed = True

    else:
        print("Depressed", file=sys.stderr)
        arduino.write(b"B \n")
        buttonPressed = False
        line = arduino.readline().decode('utf-8').rstrip()
        print(line)

def handle_autonomy_functions(data):

    data_check = Float64()

    data_check = data.data

    if data_check == 1:
        arduino.write(b"home \n")
    elif data_check == 2:
        arduino.write(b"place \n")
    elif data_check == 3:
        arduino.write(b"pickup \n")


def handle_claw_movements(data):
    claw_msg = Float64MultiArray()
    claw_msg.layout.dim.append(MultiArrayDimension())
    claw_msg.layout.dim[0].size = 2
    claw_msg.layout.dim[0].stride = 1
    claw_msg.layout.dim[0].label = "x"
    claw_msg.data = [data.data] * 2
    
    # publishers[CLAW_INTERNAL_TOPIC].publish(claw_msg)





def main(passed_args=None):
    """Main entry point for ROS node."""
    try:
        global buttonPressed
        buttonPressed = False

        rclpy.init(args=passed_args)
        node = rclpy.create_node(NODE)

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
            Float64,
            CLAW_EXTERNAL_TOPIC,
            handle_claw_movements,
            QUEUE_SIZE,
        )
        node.create_subscription(
            Float64,
            PARTIAL_AUTONOMY_EXTERNAL_TOPIC,
            handle_autonomy_functions,
            QUEUE_SIZE,
        )

        node.get_logger().info("paver_arm_serial_driver node created successfully")
        
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass