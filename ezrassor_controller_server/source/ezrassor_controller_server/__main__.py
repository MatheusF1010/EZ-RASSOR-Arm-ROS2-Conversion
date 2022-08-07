"""Execute a ROS node using the ezrassor_controller_server module.

This node listens for HTTP messages over the wire that can be translated into
commands for the EZRASSOR. The translated commands are published to ROS topics
that the EZRASSOR understands.
"""
import ezrassor_controller_server as server
import flask
import geometry_msgs.msg
import rclpy
import std_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import random
from rclpy.duration import Duration
from rclpy.clock import Clock


NODE = "controller_server"
WHEEL_ACTION_TOPIC = "wheel_action"
BACK_ARM_ACTION_TOPIC = "back_arm_action"
BACK_DRUM_ACTION_TOPIC = "back_drum_action"
ROUTINE_ACTION_TOPIC = "routine_action"

JOINT_1_ACTION_TOPIC = "/ezrassor/joint_1_action"
JOINT_2_ACTION_TOPIC = "/ezrassor/joint_2_action"
JOINT_3_ACTION_TOPIC = "/ezrassor/joint_3_action"
JOINT_4_ACTION_TOPIC = "/ezrassor/joint_4_action"
JOINT_5_ACTION_TOPIC = "/ezrassor/joint_5_action"
CLAW_ACTION_TOPIC = "/ezrassor/claw_action"
PARTIAL_AUTONOMY_TOPIC = "/ezrassor/partial_autonomy"

QUEUE_SIZE = 10


def main(passed_args=None):
    """Main entry point for the ROS node."""
    try:
        rclpy.init(args=passed_args)
        node = rclpy.create_node(NODE)

        # Create publishers for each type of action.
        wheel_action_publisher = node.create_publisher(
            geometry_msgs.msg.Twist,
            WHEEL_ACTION_TOPIC,
            QUEUE_SIZE,
        )
        back_arm_action_publisher = node.create_publisher(
            std_msgs.msg.Float32,
            BACK_ARM_ACTION_TOPIC,
            QUEUE_SIZE,
        )
        back_drum_action_publisher = node.create_publisher(
            std_msgs.msg.Float32,
            BACK_DRUM_ACTION_TOPIC,
            QUEUE_SIZE,
        )
        routine_action_publisher = node.create_publisher(
            std_msgs.msg.Int8,
            ROUTINE_ACTION_TOPIC,
            QUEUE_SIZE,
        )
        joint_1_action_publisher = node.create_publisher(
            std_msgs.msg.Float64,
            JOINT_1_ACTION_TOPIC,
            QUEUE_SIZE,
        )
        joint_2_action_publisher = node.create_publisher(
            std_msgs.msg.Float64,
            JOINT_2_ACTION_TOPIC,
            QUEUE_SIZE,
        )
        joint_3_action_publisher = node.create_publisher(
            std_msgs.msg.Float64,
            JOINT_3_ACTION_TOPIC,
            QUEUE_SIZE,
        )
        joint_4_action_publisher = node.create_publisher(
            std_msgs.msg.Float64,
            JOINT_4_ACTION_TOPIC,
            QUEUE_SIZE,
        )
        joint_5_action_publisher = node.create_publisher(
            std_msgs.msg.Float64,
            JOINT_5_ACTION_TOPIC,
            QUEUE_SIZE,
        )
        claw_action_publisher = node.create_publisher(
            std_msgs.msg.Float64,
            CLAW_ACTION_TOPIC,
            QUEUE_SIZE,
        )
        partial_autonomy_publisher = node.create_publisher(
           JointTrajectory,
           PARTIAL_AUTONOMY_TOPIC,
           QUEUE_SIZE, 
        )

        def process_request(request):
            """Callback to create and publish a command from a request."""
            server.verify(request)
            command = server.create_command(request)

            if command.wheel_action is not None:
                wheel_action = geometry_msgs.msg.Twist()
                wheel_action.linear.x = command.wheel_action.linear_x
                wheel_action.angular.z = command.wheel_action.angular_z
                wheel_action_publisher.publish(wheel_action)

            if command.back_arm_action is not None:
                back_arm_action = std_msgs.msg.Float32()
                back_arm_action.data = command.back_arm_action.value
                back_arm_action_publisher.publish(back_arm_action)

            if command.back_drum_action is not None:
                back_drum_action = std_msgs.msg.Float32()
                back_drum_action.data = command.back_drum_action.value
                back_drum_action_publisher.publish(back_drum_action)

            if command.routine_action is not None:
                routine_action = std_msgs.msg.Int8()
                routine_action.data = command.routine_action.value
                routine_action_publisher.publish(routine_action)

            if command.joint_1_action is not None:
                joint_1_action = std_msgs.msg.Float64()
                joint_1_action.data = command.joint_1_action.value
                joint_1_action_publisher.publish(joint_1_action)

            if command.joint_2_action is not None:
                joint_2_action = std_msgs.msg.Float64()
                joint_2_action.data = command.joint_2_action.value
                joint_2_action_publisher.publish(joint_2_action)

            if command.joint_3_action is not None:
                joint_3_action = std_msgs.msg.Float64()
                joint_3_action.data = command.joint_3_action.value
                joint_3_action_publisher.publish(joint_3_action)

            if command.joint_4_action is not None:
                joint_4_action = std_msgs.msg.Float64()
                joint_4_action.data = command.joint_4_action.value
                joint_4_action_publisher.publish(joint_4_action)
            
            if command.joint_5_action is not None:
                joint_5_action = std_msgs.msg.Float64()
                joint_5_action.data = command.joint_5_action.value
                joint_5_action_publisher.publish(joint_5_action)

            if command.claw_action is not None:
                claw_action = std_msgs.msg.Float64()
                claw_action.data = command.claw_action.value
                claw_action_publisher.publish(claw_action)
            
            if command.partial_autonomy is not None:

                # rclpy.init_node('inverse_kinematics')
                # control_publisher = rclpy.Publisher('/ezrassor/partial_autonomy', JointTrajectory, queue_size=10)

                # while not rclpy.is_shutdown():

                msg = JointTrajectory()

                msg.header.stamp = Clock().now().to_msg()
                msg.header.frame_id = ''
                msg.joint_names = ['joint12', 'joint23', 'joint34', 'joint45', 'joint56']

                point = JointTrajectoryPoint()
                j1 = 2 * (random.random() - 0.5)  # 0 - 1 -> -0.5 - 0.5
                j2 = 2 * (random.random() - 0.5)
                j3 = 2 * (random.random() - 0.5)
                j4 = 2 * (random.random() - 0.5)
                j5 = 2 * (random.random() - 0.5)

                point.positions = [j1, j2, j3, j4, j5]
                point.velocities = []
                point.accelerations = []
                point.effort = []

                dur = Duration()
                point.time_from_start = Duration(seconds=5).to_msg()

                msg.points.append( point )

                partial_autonomy_publisher.publish( msg )
                # node.loginfo( msg ) 

                # partial_autonomy = std_msgs.msg.Float64MultiArray()
                # partial_autonomy.data = command.partial_autonomy.value
                # partial_autonomy_publisher.publish(partial_autonomy)
            
        

        # Create a Flask app to serve HTTP requests.
        app = flask.Flask(__name__)

        @app.route("/", methods=["POST"])
        def handle_request():
            """Handle HTTP requests and log any errors.

            This function is the glue between Flask/HTTP and ROS business logic
            in this package.
            """
            try:
                process_request(flask.request.get_json())

                return {"status": 200}
            except server.VerificationError as error:
                node.get_logger().error(str(error))

                return {"status": 400}

        # Run the app! Note that we don't spin the ROS node here. Only nodes
        # containing subscribers must be spun.
        app.run(host='0.0.0.0')
    except KeyboardInterrupt:
        pass
